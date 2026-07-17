#include "texlights.h"

#include <cstdio>
#include <cstring>
#include <fstream>

#include "common/log.h"

namespace format
{
    int read_rad_file(const std::string &path, std::vector<texlight> &out)
    {
        std::ifstream f(path);
        if (!f)
        {
            logging::warn("could not open texlight file %s", path.c_str());
            return -1;
        }

        int parsed = 0;
        std::string line;
        while (std::getline(f, line))
        {
            // a "//" comment runs to end of line
            size_t comment = line.find("//");
            if (comment != std::string::npos)
                line.erase(comment);

            char name[260];
            float r, g, b, scale = 1;
            int argc = std::sscanf(line.c_str(), "%s %f %f %f %f", name, &r, &g, &b, &scale);

            if (argc == 2)
            {
                // one value: grayscale
                g = b = r;
            }
            else if (argc == 5)
            {
                // colour scaled by the fourth value, computed in double exactly
                // as the reference did, then narrowed to float
                r = (float)(r * (scale / 255.0));
                g = (float)(g * (scale / 255.0));
                b = (float)(b * (scale / 255.0));
            }
            else if (argc != 4)
            {
                if (line.size() > 4)
                    logging::warn("ignoring bad texlight '%s' in %s", line.c_str(), path.c_str());
                continue;
            }

            // a later definition overrides an earlier one by name
            for (auto it = out.begin(); it != out.end(); ++it)
            {
                if (it->name == name)
                {
                    if (it->value[0] != r || it->value[1] != g || it->value[2] != b)
                        logging::warn("overriding texlight '%s' from '%s' with '%s'",
                                      it->name.c_str(), it->source.c_str(), path.c_str());
                    out.erase(it);
                    break;
                }
            }

            texlight t;
            t.name = name;
            t.value[0] = r;
            t.value[1] = g;
            t.value[2] = b;
            t.source = path;
            out.push_back(std::move(t));
            parsed++;
        }

        logging::file("%d texlights parsed (%s)\n", parsed, path.c_str());
        return parsed;
    }
}
