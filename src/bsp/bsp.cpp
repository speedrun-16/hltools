#include "bsp.h"

#include <cstdio>
#include <cstring>

#include "../common/error.h"
#include "../common/limits.h"
#include "../common/log.h"
#include "../common/string_util.h"
#include "internal.h"

// the stage driver: reads each model's surfaces and detail brushes from the
// csg files, builds and fills the tree per hull, and emits the disk lumps in
// the reference processmodel order

namespace bsp
{
    const format::entity *entity_for_model(const bsp_state &state, int modnum)
    {
        char name[16];
        str::format(name, sizeof(name), "*%i", modnum);
        for (const format::entity &ent : state.entities)
        {
            if (std::strcmp(ent.value("model"), name) == 0)
                return &ent;
        }
        return state.entities.empty() ? nullptr : &state.entities[0];
    }

    namespace
    {
        void grow_model_bounds(bsp_state &state, format::dmodel_t *model,
                               const surfchain *surfs, int hullnum)
        {
            if (surfs->mins[0] > surfs->maxs[0])
                return; // empty hull

            math::vec3v mins, maxs;
            for (int i = 0; i < 3; i++)
            {
                mins[i] = surfs->mins[i] - state.hull_size[hullnum][0][i];
                maxs[i] = surfs->maxs[i] - state.hull_size[hullnum][1][i];
            }
            for (int i = 0; i < 3; i++)
            {
                if (mins[i] > maxs[i])
                {
                    vec_t tmp = (mins[i] + maxs[i]) / 2;
                    mins[i] = tmp;
                    maxs[i] = tmp;
                }
            }
            for (int i = 0; i < 3; i++)
            {
                if ((float)maxs[i] > model->maxs[i])
                    model->maxs[i] = (float)maxs[i];
                if ((float)mins[i] < model->mins[i])
                    model->mins[i] = (float)mins[i];
            }
        }

        bool process_model(bsp_state &state)
        {
            surfchain *surfs = read_surfs(state, 0);
            if (!surfs)
                return false; // all models are done
            brush *detailbrushes = read_brushes(state, 0);

            format::map_data &map = *state.map;
            if ((int)map.models.size() >= limits::max_map_models)
                err::fatal("exceeded max_map_models");

            int startleafs = (int)map.leafs.size();
            int modnum = (int)map.models.size();
            map.models.resize((size_t)modnum + 1);
            format::dmodel_t *model = &map.models[(size_t)modnum];
            state.nummodels = modnum + 1;

            state.hullnum = 0;
            model->mins[0] = model->mins[1] = model->mins[2] = 99999;
            model->maxs[0] = model->maxs[1] = model->maxs[2] = -99999;
            grow_model_bounds(state, model, surfs, 0);

            // the tree
            node *nodes = solid_bsp(state, surfs, detailbrushes, modnum == 0);

            // assume non world bmodels are simple
            if (modnum == 0 && !state.options.nofill)
            {
                if (!state.options.noinsidefill)
                    fill_inside(state, nodes);
                nodes = fill_outside(state, nodes, !state.leaked, 0); // leakfile if not yet leaked
            }
            delete surfs;

            free_portals(nodes);

            // fix tjunctions
            tjunc(state, nodes);

            make_face_edges(state);

            // emit the faces for the bsp file
            model->headnode[0] = (int)map.nodes.size();
            model->firstface = (int)map.faces.size();
            bool novisiblebrushes = false;
            // model->headnode[0] < 0 will crash the engine, so split it
            if (nodes->planenum == -1)
            {
                novisiblebrushes = true;
                if (nodes->markfaces && nodes->markfaces[0] != nullptr)
                    err::fatal("process_model: empty solid entity");
                if (state.planes.empty())
                    err::fatal("no valid planes");
                nodes->planenum = 0; // arbitrary plane
                for (int k = 0; k < 2; k++)
                {
                    node *child = alloc_node();
                    child->planenum = -1;
                    child->contents = contents_empty;
                    child->isdetail = false;
                    child->isportalleaf = true;
                    child->iscontentsdetail = false;
                    child->faces = nullptr;
                    child->markfaces = new face *[1]();
                    child->mins = {};
                    child->maxs = {};
                    nodes->children[k] = child;
                }
                nodes->contents = 0;
                nodes->isdetail = false;
                nodes->isportalleaf = false;
                nodes->faces = nullptr;
                nodes->markfaces = nullptr;
                nodes->mins = {};
                nodes->maxs = {};
            }
            write_draw_nodes(state, nodes);
            model->numfaces = (int)map.faces.size() - model->firstface;
            model->visleafs = (int)map.leafs.size() - startleafs;

            if (state.options.noclip)
            {
                // store the empty content type in the headnode pointers to
                // signify the lack of clipping information in a way that
                // doesn't crash the engine
                model->headnode[1] = contents_empty;
                model->headnode[2] = contents_empty;
                model->headnode[3] = contents_empty;
            }
            else
            {
                // the clipping hulls are simpler
                for (state.hullnum = 1; state.hullnum < num_hulls; state.hullnum++)
                {
                    surfs = read_surfs(state, state.hullnum);
                    detailbrushes = read_brushes(state, state.hullnum);
                    grow_model_bounds(state, model, surfs, state.hullnum);
                    nodes = solid_bsp(state, surfs, detailbrushes, modnum == 0);
                    if (modnum == 0 && !state.options.nofill)
                        nodes = fill_outside(state, nodes, !state.leaked, state.hullnum);
                    delete surfs;
                    free_portals(nodes);
                    // test that the head clip node isn't empty: if it is, store
                    // the content type of the head instead
                    if (nodes->planenum == -1)
                    {
                        model->headnode[state.hullnum] = nodes->contents;
                    }
                    else
                    {
                        model->headnode[state.hullnum] = (int)map.clipnodes.size();
                        write_clip_nodes(state, nodes);
                    }
                }
            }

            {
                const format::entity *ent = entity_for_model(state, modnum);
                if (ent && ent != &state.entities[0] && ent->value("zhlt_minsmaxs")[0])
                {
                    double origin[3] = {0, 0, 0}, mins[3], maxs[3];
                    std::sscanf(ent->value("origin"), "%lf %lf %lf",
                                &origin[0], &origin[1], &origin[2]);
                    if (std::sscanf(ent->value("zhlt_minsmaxs"), "%lf %lf %lf %lf %lf %lf",
                                    &mins[0], &mins[1], &mins[2], &maxs[0], &maxs[1], &maxs[2]) == 6)
                    {
                        for (int i = 0; i < 3; i++)
                        {
                            model->mins[i] = (float)(mins[i] - origin[i]);
                            model->maxs[i] = (float)(maxs[i] - origin[i]);
                        }
                    }
                }
            }
            if (model->mins[0] > model->maxs[0])
            {
                const format::entity *ent = entity_for_model(state, state.nummodels - 1);
                if (state.nummodels - 1 != 0 && ent == &state.entities[0])
                    ent = nullptr;
                logging::warn("Empty solid entity: model %d (entity: classname \"%s\", origin \"%s\", targetname \"%s\")",
                              state.nummodels - 1,
                              ent ? ent->value("classname") : "unknown",
                              ent ? ent->value("origin") : "unknown",
                              ent ? ent->value("targetname") : "unknown");
                // fix "backward minsmaxs" in the engine
                model->mins[0] = model->mins[1] = model->mins[2] = 0;
                model->maxs[0] = model->maxs[1] = model->maxs[2] = 0;
            }
            else if (novisiblebrushes)
            {
                const format::entity *ent = entity_for_model(state, state.nummodels - 1);
                if (state.nummodels - 1 != 0 && ent == &state.entities[0])
                    ent = nullptr;
                logging::warn("No visible brushes in solid entity: model %d (entity: classname \"%s\", origin \"%s\", targetname \"%s\", range (%.0f,%.0f,%.0f) - (%.0f,%.0f,%.0f))",
                              state.nummodels - 1,
                              ent ? ent->value("classname") : "unknown",
                              ent ? ent->value("origin") : "unknown",
                              ent ? ent->value("targetname") : "unknown",
                              (double)model->mins[0], (double)model->mins[1], (double)model->mins[2],
                              (double)model->maxs[0], (double)model->maxs[1], (double)model->maxs[2]);
            }
            return true;
        }
    }

    namespace
    {
        void init_state(bsp_state &state, format::map_data &map,
                        const std::string &base_path, const bsp_options &options)
        {
            state.map = &map;
            state.options = options;
            state.base_path = base_path;
            state.portfilename = base_path + ".prt";

            // delete previous outputs
            std::remove((base_path + ".prt").c_str());
            std::remove((base_path + ".pts").c_str());
            std::remove((base_path + ".lin").c_str());
            std::remove((base_path + ".ext").c_str());
        }

        void run_stage(bsp_state &state)
        {
            // init the tables shared by all models
            begin_bsp_file(state);

            // process each model individually; the world model prints the per hull
            // "building bsp tree" summary from inside solid_bsp
            while (process_model(state))
            {
            }

            // the budget is a whole-map one, so the check has to outlive the
            // model loop: failing inside it reported a total that excluded
            // every model after the first one to cross the line
            fail_if_clipnode_limit_exceeded(state);

            // one consolidated leaf content report after every model and hull
            print_leaf_content_conflicts(state);

            if (state.unsplittable_faces != 0)
                logging::warn("dropped %zu sliver faces that could not be subdivided "
                              "under the lightmap extent limit",
                              state.unsplittable_faces);

            // one consolidated leak block (if any hull leaked), before the chart
            print_leak_summary();

            finish_bsp_file(state);
        }
    }

    void run_bsp(format::map_data &map, const std::string &base_path, const bsp_options &options)
    {
        bsp_state state;
        init_state(state, map, base_path, options);

        open_input_files(state);
        load_hull_sizes(state);
        state.entities = format::parse_entities(map.entities);
        load_plane_file(state);

        run_stage(state);

        // the polyfiles are no longer valid because the bsp was updated
        for (int i = 0; i < num_hulls; i++)
        {
            std::remove((base_path + ".p" + std::to_string(i)).c_str());
            std::remove((base_path + ".b" + std::to_string(i)).c_str());
        }
        std::remove((base_path + ".hsz").c_str());
        std::remove((base_path + ".pln").c_str());
    }

    void run_bsp(format::map_data &map, const std::string &base_path, const bsp_options &options,
                 bsp_input input)
    {
        bsp_state state;
        init_state(state, map, base_path, options);

        set_input_data(state, input);
        state.entities = format::parse_entities(map.entities);

        run_stage(state);
    }
}
