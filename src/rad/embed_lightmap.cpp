#include <cmath>
#include <cstdlib>
#include <cstring>

#include "../common/binary.h"
#include "../common/error.h"
#include "../common/log.h"
#include "../common/string_util.h"
#include "format/bsp/face_extents.h"
#include "internal.h"

// zhlt_embedlightmap: bakes the computed lightmap of selected entity faces into
// brand new "?_rad" textures so translucent surfaces can appear lit in game
// includes the color quantization used to build each new texture's palette and
// the reverse pass that strips such textures from a previous compile

namespace rad
{
    namespace
    {
        // ===== color quantization =====

        constexpr int cq_dim = 3;

        template <class T, class T2, class T3>
        inline void cq_vector_subtract(const T a[cq_dim], const T2 b[cq_dim], T3 c[cq_dim])
        {
            for (int x = 0; x < cq_dim; x++)
            {
                c[x] = a[x] - b[x];
            }
        }

        template <class T, class T2, class T3>
        inline void cq_vector_add(const T a[cq_dim], const T2 b[cq_dim], T3 c[cq_dim])
        {
            for (int x = 0; x < cq_dim; x++)
            {
                c[x] = a[x] + b[x];
            }
        }

        template <class T, class T2>
        inline void cq_vector_scale(const T a[cq_dim], const T2 b, T c[cq_dim])
        {
            for (int x = 0; x < cq_dim; x++)
            {
                c[x] = a[x] * b;
            }
        }

        template <class T, class T2>
        inline void cq_vector_copy(const T a[cq_dim], T2 b[cq_dim])
        {
            for (int x = 0; x < cq_dim; x++)
            {
                b[x] = a[x];
            }
        }

        template <class T>
        inline void cq_vector_clear(T a[cq_dim])
        {
            for (int x = 0; x < cq_dim; x++)
            {
                a[x] = (T)0;
            }
        }

        template <class T>
        inline T cq_dot_product(const T a[cq_dim], const T b[cq_dim])
        {
            T dot = (T)0;
            for (int x = 0; x < cq_dim; x++)
            {
                dot += a[x] * b[x];
            }
            return dot;
        }

        // partitions the space into point[axis] < dist and point[axis] >= dist
        struct cq_splitter
        {
            int axis;
            int dist;
            double numpoints[2];
        };

        // a cuboid region; the root node is the entire cube of size 255
        struct cq_node
        {
            bool isleafnode;
            cq_node *parentnode;
            cq_node *childrennode[2];

            int numpoints; // numpoints > 0
            unsigned char (*refpoints)[cq_dim];
            double centerofpoints[cq_dim];

            bool needsplit;
            cq_splitter bestsplitter;
            double splitpriority;
        };

        struct cq_searchnode
        {
            bool isleafnode;
            cq_searchnode *childrennode[2];

            int planeaxis;
            int planedist;

            int result;
        };

        void cq_select_partition(cq_node *node)
        {
            cq_vector_clear(node->centerofpoints);
            for (int i = 0; i < node->numpoints; i++)
            {
                cq_vector_add(node->centerofpoints, node->refpoints[i], node->centerofpoints);
            }
            cq_vector_scale(node->centerofpoints, 1 / (double)node->numpoints, node->centerofpoints);

            node->needsplit = false;
            for (int k = 0; k < cq_dim; k++)
            {
                double count;
                double counts[256];
                double sum[cq_dim];
                double sums[256][cq_dim];

                double bucketsums[256][cq_dim];
                int bucketsizes[256];

                const unsigned char (*nodepoints)[cq_dim] = node->refpoints;
                const int nodenumpoints = node->numpoints;

                memset(bucketsums, 0, 256 * sizeof(double[cq_dim]));
                memset(bucketsizes, 0, 256 * sizeof(int));
                for (int i = 0; i < nodenumpoints; i++)
                {
                    int j = nodepoints[i][k];
                    bucketsizes[j]++;
                    cq_vector_add(bucketsums[j], nodepoints[i], bucketsums[j]);
                }

                int min = 256;
                int max = -1;
                count = 0;
                cq_vector_clear(sum);
                for (int j = 0; j < 256; j++)
                {
                    counts[j] = count;
                    cq_vector_copy(sum, sums[j]);
                    count += bucketsizes[j];
                    cq_vector_add(sum, bucketsums[j], sum);
                    if (bucketsizes[j] > 0)
                    {
                        if (j < min)
                        {
                            min = j;
                        }
                        if (j > max)
                        {
                            max = j;
                        }
                    }
                }
                if (max < min)
                {
                    err::fatal("cq_select_partition: internal error");
                }
                // sweep along the axis and find the plane maximizing the square error reduction
                for (int j = min + 1; j < max + 1; j++)
                {
                    double priority = 0; // the decrease in total square deviation
                    priority -= cq_dot_product(sum, sum) / count;
                    priority += cq_dot_product(sums[j], sums[j]) / counts[j];
                    double remain[cq_dim];
                    cq_vector_subtract(sum, sums[j], remain); // sums and counts are precise
                    priority += cq_dot_product(remain, remain) / (count - counts[j]);
                    if (node->needsplit == false ||
                        priority > node->splitpriority + 0.1 ||
                        (priority >= node->splitpriority - 0.1
                         && fabs(counts[j] - count / 2) < fabs(node->bestsplitter.numpoints[0] - count / 2)))
                    {
                        node->needsplit = true;
                        node->splitpriority = priority;
                        node->bestsplitter.axis = k;
                        node->bestsplitter.dist = j;
                        node->bestsplitter.numpoints[0] = counts[j];
                        node->bestsplitter.numpoints[1] = count - counts[j];
                    }
                }
            }
        }

        cq_searchnode *cq_alloc_search_tree(int maxcolors)
        {
            cq_searchnode *searchtree;
            searchtree = (cq_searchnode *)malloc((size_t)(2 * maxcolors - 1) * sizeof(cq_searchnode));
            err::require(searchtree != nullptr, "cq_alloc_search_tree: out of memory");
            return searchtree;
        }

        void cq_free_search_tree(cq_searchnode *searchtree)
        {
            free(searchtree);
        }

        void cq_create_palette(int numpoints, const unsigned char (*points)[cq_dim],
                               int maxcolors, unsigned char (*colors_out)[cq_dim], int &numcolors_out,
                               cq_searchnode *searchtree_out) // [2 * maxcolors - 1]
        {
            if (numpoints <= 0 || maxcolors <= 0)
            {
                numcolors_out = 0;
                return;
            }

            unsigned char (*pointarray)[cq_dim];
            pointarray = (unsigned char (*)[cq_dim])malloc((size_t)numpoints * sizeof(unsigned char[cq_dim]));
            err::require(pointarray != nullptr, "cq_create_palette: out of memory");
            memcpy(pointarray, points, (size_t)numpoints * sizeof(unsigned char[cq_dim]));

            cq_node *n;
            cq_searchnode *s;
            int numnodes = 0;
            int maxnodes = 2 * maxcolors - 1;
            cq_node *nodes = (cq_node *)malloc((size_t)maxnodes * sizeof(cq_node));
            err::require(nodes != nullptr, "cq_create_palette: out of memory");

            n = &nodes[0];
            numnodes++;

            n->isleafnode = true;
            n->parentnode = nullptr;
            n->numpoints = numpoints;
            n->refpoints = pointarray;
            cq_select_partition(n);

            for (int i = 1; i < maxcolors; i++)
            {
                bool needsplit;
                double bestpriority = 0;
                cq_node *bestnode = nullptr;

                needsplit = false;
                for (int j = 0; j < numnodes; j++)
                {
                    n = &nodes[j];
                    if (!n->isleafnode || !n->needsplit)
                    {
                        continue;
                    }
                    if (needsplit == false || n->splitpriority > bestpriority + 0.1)
                    {
                        needsplit = true;
                        bestpriority = n->splitpriority;
                        bestnode = n;
                    }
                }
                if (!needsplit)
                {
                    break;
                }

                bestnode->isleafnode = false;
                for (int k = 0; k < 2; k++)
                {
                    n = &nodes[numnodes];
                    numnodes++;
                    if (numnodes > maxnodes)
                    {
                        err::fatal("cq_create_palette: internal error");
                    }

                    bestnode->childrennode[k] = n;
                    n->isleafnode = true;
                    n->parentnode = bestnode;
                    n->numpoints = 0;
                    n->refpoints = nullptr;
                }

                // partition the points using the best splitter
                {
                    const int splitaxis = bestnode->bestsplitter.axis;
                    const int splitdist = bestnode->bestsplitter.dist;

                    unsigned char (*left)[cq_dim];
                    unsigned char (*right)[cq_dim];
                    left = &bestnode->refpoints[0];
                    right = &bestnode->refpoints[bestnode->numpoints - 1];
                    while (1)
                    {
                        while ((*left)[splitaxis] < splitdist)
                        {
                            left++;
                        }
                        while ((*right)[splitaxis] >= splitdist)
                        {
                            right--;
                        }
                        if (left >= right)
                        {
                            break;
                        }
                        unsigned char tmp[cq_dim];
                        cq_vector_copy(*left, tmp);
                        cq_vector_copy(*right, *left);
                        cq_vector_copy(tmp, *right);
                    }
                    if (right != left - 1)
                    {
                        err::fatal("cq_create_palette: internal error");
                    }

                    bestnode->childrennode[0]->numpoints = (int)(left - bestnode->refpoints);
                    bestnode->childrennode[0]->refpoints = bestnode->refpoints;
                    bestnode->childrennode[1]->numpoints = (int)(&bestnode->refpoints[bestnode->numpoints] - left);
                    bestnode->childrennode[1]->refpoints = left;
                    if (bestnode->childrennode[0]->numpoints <= 0 ||
                        bestnode->childrennode[0]->numpoints != bestnode->bestsplitter.numpoints[0])
                    {
                        err::fatal("cq_create_palette: internal error");
                    }
                    if (bestnode->childrennode[1]->numpoints <= 0 ||
                        bestnode->childrennode[1]->numpoints != bestnode->bestsplitter.numpoints[1])
                    {
                        err::fatal("cq_create_palette: internal error");
                    }
                }

                cq_select_partition(bestnode->childrennode[0]);
                cq_select_partition(bestnode->childrennode[1]);
            }

            for (int i = 0; i < numnodes; i++)
            {
                n = &nodes[i];
                s = &searchtree_out[i];
                s->isleafnode = n->isleafnode;
                if (!n->isleafnode)
                {
                    s->planeaxis = n->bestsplitter.axis;
                    s->planedist = n->bestsplitter.dist;
                    s->childrennode[0] = &searchtree_out[n->childrennode[0] - nodes];
                    s->childrennode[1] = &searchtree_out[n->childrennode[1] - nodes];
                }
            }

            numcolors_out = 0;
            n = &nodes[0];
            while (1)
            {
                while (!n->isleafnode)
                {
                    n = n->childrennode[0];
                }
                s = &searchtree_out[n - nodes];
                s->result = numcolors_out;
                for (int k = 0; k < cq_dim; k++)
                {
                    int val = (int)floor(n->centerofpoints[k] + 0.5 + 0.00001);
                    val = val < 255 ? val : 255;
                    val = 0 > val ? 0 : val;
                    colors_out[numcolors_out][k] = (unsigned char)val;
                }
                numcolors_out++;
                while (n->parentnode)
                {
                    if (n == n->parentnode->childrennode[0])
                    {
                        break;
                    }
                    n = n->parentnode;
                }
                if (!n->parentnode)
                {
                    break;
                }
                n = n->parentnode->childrennode[1];
            }

            if (2 * numcolors_out - 1 != numnodes)
            {
                err::fatal("cq_create_palette: internal error");
            }

            free(pointarray);
            free(nodes);
        }

        void cq_map_point_r(int *bestdist, int *best,
                            cq_searchnode *node, const unsigned char (*colors)[cq_dim],
                            const unsigned char point[cq_dim], int searchradius)
        {
            while (!node->isleafnode)
            {
                int dist = point[node->planeaxis] - node->planedist;
                if (dist <= -searchradius)
                {
                    node = node->childrennode[0];
                }
                else if (dist >= searchradius - 1)
                {
                    node = node->childrennode[1];
                }
                else
                {
                    cq_map_point_r(bestdist, best, node->childrennode[0], colors, point, searchradius);
                    cq_map_point_r(bestdist, best, node->childrennode[1], colors, point, searchradius);
                    return;
                }
            }
            int dist = 0;
            for (int k = 0; k < cq_dim; k++)
            {
                dist += (colors[node->result][k] - point[k]) * (colors[node->result][k] - point[k]);
            }
            if (dist <= *bestdist)
            {
                if (dist < *bestdist || node->result < *best)
                {
                    *bestdist = dist;
                    *best = node->result;
                }
            }
        }

        int cq_map_point(const unsigned char point[cq_dim], const unsigned char (*colors)[cq_dim],
                         int numcolors, cq_searchnode *searchtree)
        {
            if (numcolors <= 0)
            {
                err::fatal("cq_map_point: internal error");
            }

            cq_searchnode *node;
            int bestdist;
            int best;
            int searchradius;

            for (node = searchtree; !node->isleafnode;)
            {
                node = node->childrennode[point[node->planeaxis] >= node->planedist];
            }
            best = node->result;
            bestdist = 0;
            for (int k = 0; k < cq_dim; k++)
            {
                bestdist += (colors[best][k] - point[k]) * (colors[best][k] - point[k]);
            }

            searchradius = (int)ceil(sqrt((double)bestdist) + 0.1);
            cq_map_point_r(&bestdist, &best, searchtree, colors, point, searchradius);
            return best;
        }

        // ===== new texture collection =====

        // should be smaller than 62 * 62 and smaller than the engine texture limit
        constexpr int radtextures_max = 2048;

        int newtextures_current_miptex_index(const rad_state &state, int newcount)
        {
            binary::reader textures(state.map->textures);
            std::int32_t count;
            err::require(textures.i32(count), "invalid texture directory");
            return count + newcount;
        }

        void newtextures_write(rad_state &state, std::vector<std::vector<byte>> &newtextures)
        {
            if (newtextures.empty())
            {
                return;
            }

            format::map_data &map = *state.map;
            int newcount = (int)newtextures.size();
            binary::reader original_textures(map.textures);
            std::int32_t nummiptex;
            err::require(original_textures.i32(nummiptex),
                         "newtextures_write: invalid texture directory");

            // grow the offset table by newcount entries, shifting the pixel data up
            size_t dataaddr = 4 + (size_t)nummiptex * 4;
            size_t datasize = map.textures.size() - dataaddr;
            size_t newdataaddr = 4 + (size_t)(nummiptex + newcount) * 4;
            err::require((int)(map.textures.size() + (newdataaddr - dataaddr)) <= state.options.max_map_miptex,
                         "newtextures_write: exceeded MAX_MAP_MIPTEX");
            map.textures.resize(map.textures.size() + (newdataaddr - dataaddr));
            memmove(map.textures.data() + newdataaddr, map.textures.data() + dataaddr, datasize);
            binary::reader shifted_textures(map.textures);
            binary::writer texture_output(map.textures);
            for (int i = 0; i < nummiptex; i++)
            {
                std::int32_t ofs;
                err::require(shifted_textures.i32_at(4 + (size_t)i * 4, ofs),
                             "newtextures_write: invalid texture offset");
                if (ofs < 0) // bad texture
                {
                    continue;
                }
                ofs += (int)(newdataaddr - dataaddr);
                err::require(texture_output.patch_i32(4 + (size_t)i * 4, ofs),
                             "newtextures_write: could not patch texture offset");
            }

            err::require(nummiptex + newcount < limits::max_map_textures,
                         "newtextures_write: exceeded MAX_MAP_TEXTURES");
            for (int i = 0; i < newcount; i++)
            {
                err::require((int)(map.textures.size() + newtextures[(size_t)i].size()) <= state.options.max_map_miptex,
                             "newtextures_write: exceeded MAX_MAP_MIPTEX");
                int ofs = (int)map.textures.size();
                err::require(texture_output.patch_i32(4 + (size_t)(nummiptex + i) * 4, ofs),
                             "newtextures_write: could not append texture offset");
                map.textures.insert(map.textures.end(), newtextures[(size_t)i].begin(), newtextures[(size_t)i].end());
            }
            nummiptex += newcount;
            err::require(texture_output.patch_i32(0, nummiptex),
                         "newtextures_write: could not patch texture count");

            newtextures.clear();
        }

        unsigned int hash_bytes(int size, void *data)
        {
            unsigned int hash = 0;
            for (int i = 0; i < size; i++)
            {
                hash = 31 * hash + ((unsigned char *)data)[i];
            }
            return hash;
        }

        void get_light_int(const rad_state &state, const format::dface_t *face, const int texsize[2],
                           int ix, int iy, vec3v &light)
        {
            ix = ix < texsize[0] ? ix : texsize[0];
            ix = 0 > ix ? 0 : ix;
            iy = iy < texsize[1] ? iy : texsize[1];
            iy = 0 > iy ? 0 : iy;
            math::clear(light);
            if (face->lightofs < 0)
            {
                return;
            }
            for (int k = 0; k < maxlightmaps && face->styles[k] != 255; k++)
            {
                const byte *samples = &state.map->lighting[(size_t)(face->lightofs + k * (texsize[0] + 1) * (texsize[1] + 1) * 3)];
                if (face->styles[k] == 0)
                {
                    const byte *sample = &samples[(iy * (texsize[0] + 1) + ix) * 3];
                    for (int c = 0; c < 3; c++)
                        light[c] = sample[c] + light[c];
                }
            }
        }

        void get_light(const rad_state &state, const format::dface_t *face, const int texsize[2],
                       double x, double y, vec3v &light)
        {
            int ix, iy;
            double dx, dy;
            ix = (int)floor(x);
            iy = (int)floor(y);
            dx = x - ix;
            dx = dx < 1 ? dx : 1;
            dx = 0 > dx ? 0 : dx;
            dy = y - iy;
            dy = dy < 1 ? dy : 1;
            dy = 0 > dy ? 0 : dy;

            // bilinear interpolation
            vec3v light00, light10, light01, light11;
            get_light_int(state, face, texsize, ix, iy, light00);
            get_light_int(state, face, texsize, ix + 1, iy, light10);
            get_light_int(state, face, texsize, ix, iy + 1, light01);
            get_light_int(state, face, texsize, ix + 1, iy + 1, light11);
            vec3v light0, light1;
            math::scale(light00, 1 - dy, light0);
            math::multiply_add(light0, dy, light01, light0);
            math::scale(light10, 1 - dy, light1);
            math::multiply_add(light1, dy, light11, light1);
            math::scale(light0, 1 - dx, light);
            math::multiply_add(light, dx, light1, light);
        }

        bool get_valid_texture_name(const rad_state &state, int miptex, char name[16])
        {
            const format::map_data &map = *state.map;
            binary::reader textures(map.textures);
            std::int32_t numtextures = 0;
            textures.i32(numtextures);
            std::int32_t offset;
            int size;
            const format::miptex_t *mt;

            if (miptex < 0 || miptex >= numtextures)
            {
                return false;
            }
            if (!textures.i32_at(4 + (size_t)miptex * 4, offset))
                return false;
            size = (int)map.textures.size() - offset;
            if (offset < 0 || offset < 4 + numtextures * 4 || size < (int)sizeof(format::miptex_t))
            {
                return false;
            }

            mt = (const format::miptex_t *)&map.textures[(size_t)offset];
            str::copy(name, 16, mt->name);

            if (strcmp(name, mt->name))
            {
                return false;
            }

            if (strlen(name) >= 5 && !str::istarts_with(&name[1], "_rad"))
            {
                return false;
            }

            return true;
        }

        // the texinfo index encoded in a "?_radnnnnn" texture name, or -1 if
        // the texture was not created by hlrad
        int parse_implicit_texinfo_from_texture(const rad_state &state, int miptex)
        {
            const format::map_data &map = *state.map;
            binary::reader textures(map.textures);
            std::int32_t numtextures = 0;
            textures.i32(numtextures);
            if (miptex < 0 || miptex >= numtextures)
            {
                return -1;
            }
            std::int32_t offset;
            if (!textures.i32_at(4 + (size_t)miptex * 4, offset))
                return -1;
            int size = (int)map.textures.size() - offset;
            if (offset < 0 || offset < 4 + numtextures * 4 || size < (int)sizeof(format::miptex_t))
            {
                return -1;
            }

            char name[16];
            str::copy(name, sizeof(name), (const char *)(map.textures.data() + offset));
            if (!(strlen(name) >= 6 && str::istarts_with(&name[1], "_rad")
                  && '0' <= name[5] && name[5] <= '9'))
            {
                return -1;
            }

            int texinfo = atoi(&name[5]);
            if (texinfo < 0 || texinfo >= (int)map.texinfo.size())
            {
                return -1;
            }
            return texinfo;
        }
    }

    // removes all "?_rad*" textures created by a previous hlrad run; does
    // nothing when the map has none
    void delete_embedded_lightmaps(rad_state &state)
    {
        format::map_data &map = *state.map;
        int countrestoredfaces = 0;
        int countremovedtexinfos = 0;
        int countremovedtextures = 0;
        int i;
        binary::reader textures(map.textures);
        std::int32_t numtextures = 0;
        textures.i32(numtextures);

        // step 1: parse the original texinfo stored in each "?_rad*" texture
        // and restore the faces whose lightmaps had been embedded
        for (i = 0; i < (int)map.faces.size(); i++)
        {
            format::dface_t *f = &map.faces[(size_t)i];
            int texinfo;

            texinfo = format::parse_texinfo_for_face(map, f);
            if (texinfo != f->texinfo)
            {
                f->texinfo = (short)texinfo;
                countrestoredfaces++;
            }
        }

        // step 2: remove redundant texinfo
        {
            std::vector<bool> texinfoused(map.texinfo.size(), false);

            for (i = 0; i < (int)map.faces.size(); i++)
            {
                format::dface_t *f = &map.faces[(size_t)i];

                if (f->texinfo < 0 || f->texinfo >= (int)map.texinfo.size())
                {
                    continue;
                }
                texinfoused[(size_t)f->texinfo] = true;
            }
            for (i = (int)map.texinfo.size() - 1; i > -1; i--)
            {
                format::texinfo_t *info = &map.texinfo[(size_t)i];

                if (texinfoused[(size_t)i])
                {
                    break; // still used by a face; must not remove this texinfo
                }
                if (info->miptex < 0 || info->miptex >= numtextures)
                {
                    break; // invalid; must not remove this texinfo
                }
                if (parse_implicit_texinfo_from_texture(state, info->miptex) == -1)
                {
                    break; // not added by hlrad; must not remove this texinfo
                }
                countremovedtexinfos++;
            }
            map.texinfo.resize((size_t)(i + 1)); // shrink the texinfo lump
        }

        // step 3: remove redundant textures
        {
            int numremaining; // number of remaining textures
            std::vector<bool> textureused((size_t)numtextures, false);

            for (i = 0; i < (int)map.texinfo.size(); i++)
            {
                format::texinfo_t *info = &map.texinfo[(size_t)i];

                if (info->miptex < 0 || info->miptex >= numtextures)
                {
                    continue;
                }
                textureused[(size_t)info->miptex] = true;
            }
            for (i = numtextures - 1; i > -1; i--)
            {
                if (textureused[(size_t)i] || parse_implicit_texinfo_from_texture(state, i) == -1)
                {
                    break; // must not remove this texture
                }
                countremovedtextures++;
            }
            numremaining = i + 1;

            if (numremaining < numtextures)
            {
                size_t dataaddr = 4 + (size_t)numtextures * 4;
                std::int32_t remainofs;
                err::require(textures.i32_at(4 + (size_t)numremaining * 4, remainofs),
                             "DeleteEmbeddedLightmaps: invalid texture offset");
                size_t datasize = (size_t)remainofs - dataaddr;
                size_t newdataaddr = 4 + (size_t)numremaining * 4;
                memmove(map.textures.data() + newdataaddr, map.textures.data() + dataaddr, datasize);
                map.textures.resize(newdataaddr + datasize);
                binary::reader remaining_textures(map.textures);
                binary::writer texture_output(map.textures);
                err::require(texture_output.patch_i32(0, numremaining),
                             "DeleteEmbeddedLightmaps: could not patch texture count");
                for (i = 0; i < numremaining; i++)
                {
                    std::int32_t ofs;
                    err::require(remaining_textures.i32_at(4 + (size_t)i * 4, ofs),
                                 "DeleteEmbeddedLightmaps: invalid texture offset");
                    if (ofs < 0) // bad texture
                    {
                        continue;
                    }
                    ofs -= (int)(dataaddr - newdataaddr);
                    err::require(texture_output.patch_i32(4 + (size_t)i * 4, ofs),
                                 "DeleteEmbeddedLightmaps: could not patch texture offset");
                }

                numtextures = numremaining;
            }
        }

        if (countrestoredfaces > 0 || countremovedtexinfos > 0 || countremovedtextures > 0)
        {
            logging::info("DeleteEmbeddedLightmaps: restored %d faces, removed %d texinfos and %d textures.\n",
                          countrestoredfaces, countremovedtexinfos, countremovedtextures);
        }
    }

    // checks for zhlt_embedlightmap and updates the faces, texinfo, texture
    // data and lighting data accordingly
    void embed_lightmap_in_textures(rad_state &state)
    {
        format::map_data &map = *state.map;

        if (map.lighting.empty())
        {
            // hlrad hasn't run
            return;
        }
        if (map.textures.empty())
        {
            // texdata hasn't been initialized
            return;
        }
        if (state.options.notextures)
        {
            // hlrad didn't load the wad files
            return;
        }

        int i, j, k;
        int miplevel;
        int count = 0;
        int count_bytes = 0;
        bool logged = false;
        std::vector<std::vector<byte>> newtextures;

        for (i = 0; i < (int)map.faces.size(); i++)
        {
            format::dface_t *f = &map.faces[(size_t)i];

            if (f->lightofs == -1) // some faces don't have a lightmap
            {
                continue;
            }
            if (f->texinfo < 0 || f->texinfo >= (int)map.texinfo.size())
            {
                continue;
            }

            format::entity *ent = state.face_entity[(size_t)i];
            int originaltexinfonum = f->texinfo;
            const format::texinfo_t *originaltexinfo = &map.texinfo[(size_t)originaltexinfonum];
            char texname[16];
            if (!get_valid_texture_name(state, originaltexinfo->miptex, texname))
            {
                continue;
            }
            const rad_texture *tex = &state.textures[(size_t)originaltexinfo->miptex];

            if (ent == &state.entities[0]) // world
            {
                continue;
            }
            if (!strncmp(texname, "sky", 3)
                || originaltexinfo->flags & tex_special) // skip special surfaces
            {
                continue;
            }
            if (!int_for_key(*ent, "zhlt_embedlightmap"))
            {
                continue;
            }

            if (!logged)
            {
                logging::info("\n");
                logging::info("Embed Lightmap : ");
                logged = true;
            }

            bool poweroftwo = true;
            vec_t denominator = 188.0;
            vec_t gamma = 1.05f;
            int resolution = 1;
            if (int_for_key(*ent, "zhlt_embedlightmapresolution"))
            {
                resolution = int_for_key(*ent, "zhlt_embedlightmapresolution");
                if (resolution <= 0 || resolution > texture_step || ((resolution - 1) & resolution) != 0)
                {
                    err::fatal("resolution cannot be %d; valid values are 1, 2, 4 ... %d.", resolution, (int)texture_step);
                }
            }

            // calculate the texture size and allocate memory for all miplevels

            int texturesize[2];
            float (*texture)[5]; // red, green, blue, alpha; the last one is the number of samples
            byte (*texturemips[format::mip_levels])[4]; // red, green, blue and alpha channel
            int s, t;
            int texmins[2];
            int texmaxs[2];
            int texsize[2]; // texturesize = (texsize + 1) * texture_step
            int side[2];

            format::get_face_extents(map, i, texmins, texmaxs);
            texsize[0] = texmaxs[0] - texmins[0];
            texsize[1] = texmaxs[1] - texmins[1];
            if (texsize[0] < 0 || texsize[1] < 0 || texsize[0] > limits::max_surface_extent || texsize[1] > limits::max_surface_extent)
            {
                logging::warn("skipped a face with bad surface extents @ (%4.3f %4.3f %4.3f)",
                              state.face_centroids[(size_t)i][0], state.face_centroids[(size_t)i][1], state.face_centroids[(size_t)i][2]);
                continue;
            }

            for (k = 0; k < 2; k++)
            {
                texturesize[k] = (texsize[k] + 1) * texture_step;
                if (texturesize[k] < texsize[k] * texture_step + resolution * 4)
                {
                    texturesize[k] = texsize[k] * texture_step + resolution * 4; // prevent edge bleeding
                }
                texturesize[k] = (texturesize[k] + resolution - 1) / resolution;
                texturesize[k] += 15 - (texturesize[k] + 15) % 16; // must be multiples of 16
                if (poweroftwo)
                {
                    for (j = 0; j <= 30; j++)
                    {
                        if ((1 << j) >= texturesize[k])
                        {
                            texturesize[k] = (1 << j);
                            break;
                        }
                    }
                }
                side[k] = (texturesize[k] * resolution - texsize[k] * texture_step) / 2;
            }
            texture = (float (*)[5])malloc((size_t)(texturesize[0] * texturesize[1]) * sizeof(float[5]));
            err::require(texture != nullptr, "embed_lightmap_in_textures: out of memory");
            for (miplevel = 0; miplevel < format::mip_levels; miplevel++)
            {
                texturemips[miplevel] = (byte (*)[4])malloc((size_t)((texturesize[0] >> miplevel) * (texturesize[1] >> miplevel)) * sizeof(byte[4]));
                err::require(texturemips[miplevel] != nullptr, "embed_lightmap_in_textures: out of memory");
            }

            // calculate the texture

            for (t = 0; t < texturesize[1]; t++)
            {
                for (s = 0; s < texturesize[0]; s++)
                {
                    float (*dest)[5] = &texture[t * texturesize[0] + s];
                    (*dest)[0] = 0;
                    (*dest)[1] = 0;
                    (*dest)[2] = 0;
                    (*dest)[3] = 0;
                    (*dest)[4] = 0;
                }
            }
            for (t = -side[1]; t < texsize[1] * texture_step + side[1]; t++)
            {
                for (s = -side[0]; s < texsize[0] * texture_step + side[0]; s++)
                {
                    double s_vec, t_vec;
                    double src_s, src_t;
                    int src_is, src_it;
                    byte src_index;
                    byte src_color[3];
                    double dest_s, dest_t;
                    int dest_is, dest_it;
                    float (*dest)[5];
                    double light_s, light_t;
                    vec3v light;

                    s_vec = s + texmins[0] * texture_step + 0.5;
                    t_vec = t + texmins[1] * texture_step + 0.5;

                    if (resolution == 1)
                    {
                        dest_s = s_vec;
                        dest_t = t_vec;
                    }
                    else
                    {
                        // the final blurred texture is shifted by half a pixel so
                        // that lightmap samples align with the center of pixels
                        dest_s = s_vec / resolution + 0.5;
                        dest_t = t_vec / resolution + 0.5;
                    }
                    dest_s = dest_s - texturesize[0] * floor(dest_s / texturesize[0]);
                    dest_t = dest_t - texturesize[1] * floor(dest_t / texturesize[1]);
                    dest_is = (int)floor(dest_s); // dest_s % texturesize[0]
                    dest_it = (int)floor(dest_t); // dest_t % texturesize[1]
                    dest_is = dest_is < texturesize[0] - 1 ? dest_is : texturesize[0] - 1;
                    dest_is = 0 > dest_is ? 0 : dest_is;
                    dest_it = dest_it < texturesize[1] - 1 ? dest_it : texturesize[1] - 1;
                    dest_it = 0 > dest_it ? 0 : dest_it;
                    dest = &texture[dest_it * texturesize[0] + dest_is];

                    src_s = s_vec;
                    src_t = t_vec;
                    src_s = src_s - tex->width * floor(src_s / tex->width);
                    src_t = src_t - tex->height * floor(src_t / tex->height);
                    src_is = (int)floor(src_s); // src_s % tex->width
                    src_it = (int)floor(src_t); // src_t % tex->height
                    src_is = src_is < tex->width - 1 ? src_is : tex->width - 1;
                    src_is = 0 > src_is ? 0 : src_is;
                    src_it = src_it < tex->height - 1 ? src_it : tex->height - 1;
                    src_it = 0 > src_it ? 0 : src_it;
                    src_index = tex->canvas[(size_t)(src_it * tex->width + src_is)];
                    src_color[0] = tex->palette[src_index][0];
                    src_color[1] = tex->palette[src_index][1];
                    src_color[2] = tex->palette[src_index][2];

                    // get light from the center of the destination pixel
                    light_s = (s_vec + resolution * (dest_is + 0.5 - dest_s)) / texture_step - texmins[0];
                    light_t = (t_vec + resolution * (dest_it + 0.5 - dest_t)) / texture_step - texmins[1];
                    get_light(state, f, texsize, light_s, light_t, light);

                    (*dest)[4] += 1;
                    if (!(texname[0] == '{' && src_index == 255))
                    {
                        for (k = 0; k < 3; k++)
                        {
                            float v = src_color[k] * (float)pow((double)(light[k] / denominator), gamma);
                            float vc = v < 255 ? v : 255;
                            vc = 0 > vc ? 0 : vc;
                            (*dest)[k] += 255 * vc;
                        }
                        (*dest)[3] += 255;
                    }
                }
            }
            for (t = 0; t < texturesize[1]; t++)
            {
                for (s = 0; s < texturesize[0]; s++)
                {
                    float (*src)[5] = &texture[t * texturesize[0] + s];
                    byte (*dest)[4] = &texturemips[0][t * texturesize[0] + s];

                    if ((*src)[4] == 0) // no samples (outside face range?)
                    {
                        (*dest)[0] = 0;
                        (*dest)[1] = 0;
                        (*dest)[2] = 0;
                        (*dest)[3] = 255;
                    }
                    else
                    {
                        if ((*src)[3] / (*src)[4] <= 0.4 * 255) // transparent
                        {
                            (*dest)[0] = 0;
                            (*dest)[1] = 0;
                            (*dest)[2] = 0;
                            (*dest)[3] = 0;
                        }
                        else // normal
                        {
                            for (j = 0; j < 3; j++)
                            {
                                int val = (int)floor((*src)[j] / (*src)[3] + 0.5);
                                val = val < 255 ? val : 255;
                                val = 0 > val ? 0 : val;
                                (*dest)[j] = (byte)val;
                            }
                            (*dest)[3] = 255;
                        }
                    }
                }
            }

            for (miplevel = 1; miplevel < format::mip_levels; miplevel++)
            {
                for (t = 0; t < (texturesize[1] >> miplevel); t++)
                {
                    for (s = 0; s < (texturesize[0] >> miplevel); s++)
                    {
                        byte (*src[4])[4];
                        byte (*dest)[4];
                        double average[4];

                        dest = &texturemips[miplevel][t * (texturesize[0] >> miplevel) + s];
                        src[0] = &texturemips[miplevel - 1][(2 * t) * (texturesize[0] >> (miplevel - 1)) + (2 * s)];
                        src[1] = &texturemips[miplevel - 1][(2 * t) * (texturesize[0] >> (miplevel - 1)) + (2 * s + 1)];
                        src[2] = &texturemips[miplevel - 1][(2 * t + 1) * (texturesize[0] >> (miplevel - 1)) + (2 * s)];
                        src[3] = &texturemips[miplevel - 1][(2 * t + 1) * (texturesize[0] >> (miplevel - 1)) + (2 * s + 1)];

                        average[0] = average[1] = average[2] = 0;
                        average[3] = 0;
                        for (k = 0; k < 4; k++)
                        {
                            for (j = 0; j < 3; j++)
                            {
                                average[j] += (*src[k])[3] * (*src[k])[j];
                            }
                            average[3] += (*src[k])[3];
                        }

                        if (average[3] / 4 <= 0.4 * 255)
                        {
                            (*dest)[0] = 0;
                            (*dest)[1] = 0;
                            (*dest)[2] = 0;
                            (*dest)[3] = 0;
                        }
                        else
                        {
                            for (j = 0; j < 3; j++)
                            {
                                int val = (int)floor(average[j] / average[3] + 0.5);
                                val = val < 255 ? val : 255;
                                val = 0 > val ? 0 : val;
                                (*dest)[j] = (byte)val;
                            }
                            (*dest)[3] = 255;
                        }
                    }
                }
            }

            // create its palette

            byte palette[256][3];
            cq_searchnode *palettetree = cq_alloc_search_tree(256);
            int paletteoffset;
            int palettenumcolors;

            {
                int palettemaxcolors;
                int numsamplepoints;
                unsigned char (*samplepoints)[3];

                if (texname[0] == '{')
                {
                    paletteoffset = 0;
                    palettemaxcolors = 255;
                    // the transparency color
                    palette[255][0] = tex->palette[255][0];
                    palette[255][1] = tex->palette[255][1];
                    palette[255][2] = tex->palette[255][2];
                }
                else
                {
                    paletteoffset = 0;
                    palettemaxcolors = 256;
                }

                samplepoints = (unsigned char (*)[3])malloc((size_t)(texturesize[0] * texturesize[1]) * sizeof(unsigned char[3]));
                err::require(samplepoints != nullptr, "embed_lightmap_in_textures: out of memory");
                numsamplepoints = 0;
                for (t = 0; t < texturesize[1]; t++)
                {
                    for (s = 0; s < texturesize[0]; s++)
                    {
                        byte (*src)[4] = &texturemips[0][t * texturesize[0] + s];
                        if ((*src)[3] > 0)
                        {
                            samplepoints[numsamplepoints][0] = (*src)[0];
                            samplepoints[numsamplepoints][1] = (*src)[1];
                            samplepoints[numsamplepoints][2] = (*src)[2];
                            numsamplepoints++;
                        }
                    }
                }

                cq_create_palette(numsamplepoints, samplepoints, palettemaxcolors, &palette[paletteoffset], palettenumcolors, palettetree);
                for (j = palettenumcolors; j < palettemaxcolors; j++)
                {
                    palette[paletteoffset + j][0] = 0;
                    palette[paletteoffset + j][1] = 0;
                    palette[paletteoffset + j][2] = 0;
                }

                free(samplepoints);
            }

            // emit a texinfo

            err::require((int)map.texinfo.size() < limits::max_map_texinfo,
                         "embed_lightmap_in_textures: exceeded MAX_MAP_TEXINFO");
            f->texinfo = (short)map.texinfo.size();
            map.texinfo.push_back(map.texinfo[(size_t)originaltexinfonum]);
            format::texinfo_t *info = &map.texinfo.back();

            if (resolution != 1)
            {
                // apply a scale and a shift over the original vectors
                for (k = 0; k < 2; k++)
                {
                    for (j = 0; j < 3; j++)
                    {
                        info->vecs[k][j] = (float)(info->vecs[k][j] * (1.0 / resolution));
                    }
                    info->vecs[k][3] = (float)(info->vecs[k][3] / resolution + 0.5);
                }
            }
            info->miptex = newtextures_current_miptex_index(state, (int)newtextures.size());

            // emit a texture

            int miptexsize;

            miptexsize = (int)sizeof(format::miptex_t);
            for (miplevel = 0; miplevel < format::mip_levels; miplevel++)
            {
                miptexsize += (texturesize[0] >> miplevel) * (texturesize[1] >> miplevel);
            }
            miptexsize += 2 + 256 * 3 + 2;
            format::miptex_t *miptex = (format::miptex_t *)malloc((size_t)miptexsize);
            err::require(miptex != nullptr, "embed_lightmap_in_textures: out of memory");

            memset(miptex, 0, sizeof(format::miptex_t));
            miptex->width = texturesize[0];
            miptex->height = texturesize[1];
            byte *p = (byte *)miptex + sizeof(format::miptex_t);
            for (miplevel = 0; miplevel < format::mip_levels; miplevel++)
            {
                miptex->offsets[miplevel] = (unsigned)(p - (byte *)miptex);
                for (int t2 = 0; t2 < (texturesize[1] >> miplevel); t2++)
                {
                    for (int s2 = 0; s2 < (texturesize[0] >> miplevel); s2++)
                    {
                        byte (*src)[4] = &texturemips[miplevel][t2 * (texturesize[0] >> miplevel) + s2];
                        if ((*src)[3] > 0)
                        {
                            if (palettenumcolors)
                            {
                                unsigned char point[3];
                                point[0] = (*src)[0];
                                point[1] = (*src)[1];
                                point[2] = (*src)[2];
                                *p = (byte)(paletteoffset + cq_map_point(point, &palette[paletteoffset], palettenumcolors, palettetree));
                            }
                            else // this should never happen
                            {
                                *p = (byte)(paletteoffset + 0);
                            }
                        }
                        else
                        {
                            *p = 255;
                        }
                        p++;
                    }
                }
            }
            {
                short palettecount = 256;
                std::memcpy(p, &palettecount, 2);
            }
            p += 2;
            memcpy(p, palette, 256 * 3);
            p += 256 * 3;
            {
                short zero = 0;
                std::memcpy(p, &zero, 2);
            }
            p += 2;
            if (p != (byte *)miptex + miptexsize)
            {
                err::fatal("embed_lightmap_in_textures: internal error");
            }

            if (texname[0] == '{')
            {
                strcpy(miptex->name, "{_rad");
            }
            else
            {
                strcpy(miptex->name, "__rad");
            }
            if (originaltexinfonum < 0 || originaltexinfonum > 99999)
            {
                err::fatal("embed_lightmap_in_textures: internal error: texinfo out of range");
            }
            miptex->name[5] = (char)('0' + (originaltexinfonum / 10000) % 10); // store the original texinfo
            miptex->name[6] = (char)('0' + (originaltexinfonum / 1000) % 10);
            miptex->name[7] = (char)('0' + (originaltexinfonum / 100) % 10);
            miptex->name[8] = (char)('0' + (originaltexinfonum / 10) % 10);
            miptex->name[9] = (char)('0' + (originaltexinfonum) % 10);
            char table[62];
            for (int x = 0; x < 62; x++)
            {
                table[x] = (char)(x >= 36 ? 'a' + (x - 36) : x >= 10 ? 'A' + (x - 10) : '0' + x); // ascii order
            }
            miptex->name[10] = '\0';
            miptex->name[11] = '\0';
            miptex->name[12] = '\0';
            miptex->name[13] = '\0';
            miptex->name[14] = '\0';
            miptex->name[15] = '\0';
            unsigned int hash = hash_bytes(miptexsize, miptex);
            miptex->name[10] = table[(hash / 62 / 62) % 52 + 10];
            miptex->name[11] = table[(hash / 62) % 62];
            miptex->name[12] = table[(hash) % 62];
            miptex->name[13] = table[(count / 62) % 62];
            miptex->name[14] = table[(count) % 62];
            miptex->name[15] = '\0';
            if ((int)newtextures.size() >= radtextures_max)
            {
                err::fatal("the number of textures created by hlrad has exceeded its internal limit(%d).", (int)radtextures_max);
            }
            newtextures.emplace_back((byte *)miptex, (byte *)miptex + miptexsize);
            count++;
            count_bytes += miptexsize;

            free(miptex);

            cq_free_search_tree(palettetree);

            free(texture);
            for (miplevel = 0; miplevel < format::mip_levels; miplevel++)
            {
                free(texturemips[miplevel]);
            }
        }
        newtextures_write(state, newtextures); // update texdata now

        if (logged)
        {
            logging::info("added %d texinfos and textures (%d bytes)\n", count, count_bytes);
        }
    }
}
