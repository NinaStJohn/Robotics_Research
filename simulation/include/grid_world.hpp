#pragma once
#include <vector>
#include <utility>
#include <unordered_map>
#include <unordered_set>
#include <cstdint>
#include <string>
#include "types.hpp"

// Occupancy of a cell:
//   Free    — open, traversable
//   Static  — known map structure (walls). Fixed at construction time and
//             never discovered/reveal-able at runtime: ts.cpp gives Static
//             cells no product state at all, so build-time Static layout is
//             not currently reversible (mirrors the old set_blocked
//             limitation, intentionally, for this object type). Runtime
//             NBA-driven discovery of Static structure is future work.
//   Dynamic — runtime-discoverable obstacles. Always get a product state
//             (never skipped by ts.cpp), so blocking/unblocking is always a
//             pure edge-cost change, handled incrementally.
enum class Occupancy : unsigned char { Free = 0, Static = 1, Dynamic = 2 };

class GridWorld {
    public:
        GridWorld(int w, int h);

        int width() const;
        int height() const;

        // occupancy grid
        bool in_bounds(Pos pos) const;

        bool is_blocked(Pos pos) const;    // occupancy != Free (out-of-bounds counts as blocked)
        bool is_static(Pos pos) const;      // occupancy == Static
        bool is_dynamic(Pos pos) const;     // occupancy == Dynamic

        // Refuses to overwrite the other occupied type on the same cell
        // (set_static won't clobber a Dynamic cell and vice versa).
        void set_static(Pos pos, bool value = true);
        void set_dynamic(Pos pos, bool value = true);

        // Unconditionally forces a cell back to Free, regardless of its
        // current type — unlike set_static(false)/set_dynamic(false), which
        // only clear their own type. Used by save/load (§ grid_vis.cpp) to
        // reset a cell before applying a loaded layout.
        void set_free(Pos pos);

        // Discovery bookkeeping (belief maps only — bookkeeping/vis, doesn't
        // affect planning). "Has this cell ever been sensed."
        bool is_discovered(Pos pos) const;
        void mark_discovered(Pos pos);

        // neighbors
        std::vector<Pos> neighbors4(Pos pos) const;

        // Label grid
        void define_label(const std::string& name);
        void set_label(const Pos& pnt, const std::string& name, bool value);
        bool has_label(const Pos& pnt, const std::string& name) const;
        const std::unordered_map<std::string,int>& label_map() const;
        std::vector<std::string> label_names() const;
        // for vis get layer
        const std::vector<std::uint64_t>& labels() const;

        // may need for label, per-cell mask
        std::uint64_t label_mask(int x, int y) const;

    private:
        int w_, h_;
        std::vector<std::vector<Occupancy>>     occ_;
        std::vector<std::vector<unsigned char>> discovered_;

        // Label map
        std::unordered_map<std::string, int> label_bit_;         // name -> bit index
        std::vector<std::uint64_t> labels_;                      // size w*h, each cell mask

};
