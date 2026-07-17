#include "hull_file.h"

#include <cstdio>

#include "../common/error.h"
#include "../common/filesystem.h"
#include "../common/log.h"

namespace csg
{
    namespace
    {
        void read_old_hull_file(FILE *file, const std::string &path, brush_build_options &options)
        {
            for (int h = 0; h < num_hulls; h++)
            {
                float x1, y1, z1;
                float x2, y2, z2;
                int count = std::fscanf(file, "( %f %f %f ) ( %f %f %f )\n",
                                        &x1, &y1, &z1, &x2, &y2, &z2);
                if (count != 6)
                {
                    err::fatal("could not parse old hull definition file '%s' (%d, %d)",
                               path.c_str(), h, count);
                }

                options.hull_size[h][0] = {(vec_t)x1, (vec_t)y1, (vec_t)z1};
                options.hull_size[h][1] = {(vec_t)x2, (vec_t)y2, (vec_t)z2};
            }
        }

        void read_new_hull_file(FILE *file, const std::string &path, brush_build_options &options)
        {
            for (int h = 1; h < num_hulls; h++)
            {
                float x, y, z;
                int count = std::fscanf(file, "%f %f %f\n", &x, &y, &z);
                if (count != 3)
                {
                    err::fatal("could not parse new hull definition file '%s' (%d, %d)",
                               path.c_str(), h, count);
                }

                x *= 0.5f;
                y *= 0.5f;
                z *= 0.5f;
                options.hull_size[h][0] = {(vec_t)-x, (vec_t)-y, (vec_t)-z};
                options.hull_size[h][1] = {(vec_t)x, (vec_t)y, (vec_t)z};
            }
        }
    }

    void load_hull_file(const std::string &path, brush_build_options &options)
    {
        if (path.empty())
            return;

        if (!fs::exists(path))
            err::fatal("could not find hull definition file '%s'", path.c_str());

        logging::file("loading hull definitions from '%s'\n", path.c_str());

        FILE *file = std::fopen(path.c_str(), "r");
        if (!file)
            err::fatal("could not open hull definition file '%s'", path.c_str());

        int magic = std::fgetc(file);
        std::rewind(file);

        if (magic == '(')
            read_old_hull_file(file, path, options);
        else
            read_new_hull_file(file, path, options);

        std::fclose(file);
    }
}
