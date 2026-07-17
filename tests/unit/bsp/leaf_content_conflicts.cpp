#include <iostream>
#include <sstream>

#include "bsp/internal.h"
#include "common/log.h"
#include "support/test.h"

namespace
{
    void link_faces(bsp::face *faces, size_t count)
    {
        for (size_t i = 0; i + 1 < count; i++)
            faces[i].next = &faces[i + 1];
    }
}

test("selects the highest ranked content and records every distinct content")
{
    bsp::face faces[4];
    faces[0].contents = bsp::contents_water;
    faces[1].contents = bsp::contents_empty;
    faces[2].contents = bsp::contents_solid;
    faces[3].contents = bsp::contents_empty;
    link_faces(faces, 4);

    bsp::node owner;
    bsp::surface surface;
    surface.onnode = &owner;
    surface.faces = faces;

    bsp::node leaf;
    leaf.mins = {1, 2, 3};
    leaf.maxs = {4, 5, 6};

    bsp::bsp_state state;
    state.nummodels = 5;
    state.hullnum = 2;
    bsp::classify_leaf_contents(state, &surface, &leaf);

    require(state.leaf_content_conflicts.size() == 1);
    const bsp::leaf_content_conflict &conflict = state.leaf_content_conflicts[0];
    expect(leaf.contents == bsp::contents_solid);
    expect(conflict.model == 4);
    expect(conflict.hull == 2);
    expect(conflict.contents == std::vector<int>{bsp::contents_empty, bsp::contents_water, bsp::contents_solid});
    expect(conflict.selected_content == bsp::contents_solid);
    expect(conflict.mins == leaf.mins);
    expect(conflict.maxs == leaf.maxs);
}

test("does not record a leaf when its structural faces agree")
{
    bsp::face faces[2];
    faces[0].contents = bsp::contents_water;
    faces[1].contents = bsp::contents_water;
    link_faces(faces, 2);

    bsp::node owner;
    bsp::surface surface;
    surface.onnode = &owner;
    surface.faces = faces;

    bsp::node leaf;
    bsp::bsp_state state;
    bsp::classify_leaf_contents(state, &surface, &leaf);

    expect(leaf.contents == bsp::contents_water);
    expect(state.leaf_content_conflicts.empty());
}

test("ignores detail faces when resolving a leaf")
{
    bsp::face faces[2];
    faces[0].contents = bsp::contents_empty;
    faces[1].contents = bsp::contents_solid;
    faces[1].detail_level = 1;
    link_faces(faces, 2);

    bsp::node owner;
    bsp::surface surface;
    surface.onnode = &owner;
    surface.faces = faces;

    bsp::node leaf;
    bsp::bsp_state state;
    bsp::classify_leaf_contents(state, &surface, &leaf);

    expect(leaf.contents == bsp::contents_empty);
    expect(state.leaf_content_conflicts.empty());
}

test("reports leaf content conflicts as one consolidated warning")
{
    format::entity world;
    world.set("classname", "worldspawn");
    format::entity wall;
    wall.set("classname", "func_wall");
    wall.set("model", "*4");

    bsp::bsp_state state;
    state.entities = {world, wall};

    bsp::leaf_content_conflict first;
    first.model = 4;
    first.hull = 0;
    first.mins = {1, 2, 3};
    first.maxs = {4, 5, 6};
    first.contents = {bsp::contents_empty, bsp::contents_water, bsp::contents_solid};
    first.selected_content = bsp::contents_solid;
    state.leaf_content_conflicts.push_back(first);

    bsp::leaf_content_conflict second = first;
    second.hull = 2;
    second.contents = {bsp::contents_empty, bsp::contents_solid};
    state.leaf_content_conflicts.push_back(second);

    unsigned warnings_before = logging::warning_count();
    std::ostringstream output;
    std::streambuf *previous = std::cout.rdbuf(output.rdbuf());
    bsp::print_leaf_content_conflicts(state);
    std::cout.rdbuf(previous);

    const std::string text = output.str();
    expect(logging::warning_count() == warnings_before + 1);
    expect(text.find("BSP LEAF CONTENT CONFLICTS") != std::string::npos);
    expect(text.find("2 affected BSP leaves across 1 brush model") != std::string::npos);
    expect(text.find("model 4  func_wall") != std::string::npos);
    expect(text.find("EMPTY / WATER / SOLID -> SOLID") != std::string::npos);
    expect(text.find("ambiguous leafnode content") == std::string::npos);
}
