#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include "../common/types.h"
#include "format/bsp/data.h"
#include "../math/vector.h"
#include "vis.h"

#if defined(_M_X64) || defined(_M_AMD64) || defined(__SSE2__) \
    || (defined(_M_IX86_FP) && _M_IX86_FP >= 2)
#define HLTOOLS_VIS_SSE2 1
#endif

namespace vis
{
    constexpr int max_portals = 32768;
    constexpr int max_points_on_winding = 128;
    constexpr int max_points_on_fixed_winding = 32;
    constexpr int max_portals_on_leaf = 256;
    constexpr int max_room_neighbor = 16;

    enum class portal_status
    {
        none,
        working,
        done,
    };

    struct alignas(16) winding
    {
#if HLTOOLS_VIS_SSE2
        alignas(16) float small_points[3][4] = {};
#endif
        int numpoints = 0;
        bool original = false;
        math::vec3v points[max_points_on_winding] = {};

        void pack_small_points()
        {
#if HLTOOLS_VIS_SSE2
            if (numpoints > 4)
                return;
            for (int i = 0; i < numpoints; i++)
            {
                small_points[0][i] = points[i].x;
                small_points[1][i] = points[i].y;
                small_points[2][i] = points[i].z;
            }
#endif
        }
    };

    struct plane
    {
        math::vec3v normal;
        float dist = 0.0f;
    };

    struct portal
    {
        plane plane_;
        int leaf = 0;
        winding *winding_ = nullptr;
        portal_status status = portal_status::none;
        byte *visbits = nullptr;
        byte *mightsee = nullptr;
        unsigned nummightsee = 0;
        int numcansee = 0;
        std::uint32_t zone = 0;
    };

    struct leaf
    {
        unsigned numportals = 0;
        portal *portals[max_portals_on_leaf] = {};
    };

    struct leaf_info
    {
        bool isoverviewpoint = false;
        bool isskyboxpoint = false;
        std::vector<int> additional_leaves;
        int neighbor = 0;
    };

    // an info_overview_point entity: its leaf sees everything (or, reversed,
    // everything sees its leaf, for skybox points)
    struct overview_point
    {
        int visleafnum = 0;
        int reverse = 0;
    };

    // an info_portal entity: its leaf gains visibility of a target info_leaf,
    // spreading to portal neighbors up to `neighbor` hops
    struct room_portal
    {
        int visleafnum = 0;
        int target_visleafnum = 0;
        int neighbor = 0;
    };

    struct pstack
    {
        byte mightsee[32760 / 8] = {};
        pstack *head = nullptr;
        leaf *leaf_ = nullptr;
        portal *portal_ = nullptr;
        winding *source = nullptr;
        winding *pass = nullptr;
        winding windings[3];
        char freewindings[3] = {};
        const plane *portalplane = nullptr;
        int clip_plane_count = -1;
        plane *clip_plane = nullptr;
        std::vector<plane> clip_plane_buf;
    };

    struct thread_data
    {
        byte *leafvis = nullptr;
        portal *base = nullptr;
        pstack pstack_head;
    };

    struct vis_state
    {
        format::map_data *map = nullptr;
        vis_options options;

        int numportals = 0;
        unsigned portalleafs = 0;
        std::vector<overview_point> overview_points;
        std::vector<room_portal> rooms;
        std::vector<portal> portals;
        std::vector<leaf> leafs;
        std::vector<leaf_info> leafinfos;
        std::vector<int> leafstarts;
        std::vector<int> leafcounts;
        int leafcount_all = 0;

        std::vector<winding> windings;
        std::vector<byte> uncompressed;
        std::vector<byte> visibility;
        byte *vismap_p = nullptr;
        unsigned bitbytes = 0;
        unsigned bitlongs = 0;
        int originalvismapsize = 0;
        int totalvis = 0;
    };

    int compress_vis(const byte *src, unsigned src_length, byte *dest, unsigned dest_length);
    void base_portal_vis(vis_state &state, int index);
    void portal_flow(vis_state &state, portal *p);
    void max_dist_vis(vis_state &state, int index);
}
