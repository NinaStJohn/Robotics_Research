/*
    Take in grid world and vizulize it

    think i am going to use this has insiration
    https://github.com/anson10/Path-Finding-Visualiser
*/
#include <iostream>
#include <vector>
#include <algorithm>
#include "raylib.h"

#include "grid_vis.hpp"
#include "grid_world.hpp"
#include "ts.hpp"
#include "types.hpp"
#include "dstar.hpp"
#include "debug_log.hpp"


// private functions
bool path_is_blocked(const std::vector<Pos>& path, const GridWorld& world);

/*
Takes in run from LTL, vizluize run with path
 - highlights path in different color
 - highlights start
 - highlights goal
*/
static std::vector<Pos>
flatten_path(const std::vector<std::vector<Pos>>& segments)
{
    std::vector<Pos> out;
    for (const std::vector<Pos>& seg : segments) {
        for (const Pos& pos : seg) {
            if (!out.empty() && pos.x == out.back().x && pos.y == out.back().y)
                continue;
            out.push_back(pos);
        }
    }
    return out;
}

// Builds path and path_ids in parallel from a LassoResult,
// deduplicating consecutive identical positions.
static void build_flat_path(
    const LassoResult& lasso,
    std::vector<Pos>& out_path,
    std::vector<unsigned>& out_ids
){
    out_path.clear();
    out_ids.clear();
    for (size_t i = 0; i < lasso.prefix.size(); i++) {
        const Pos& p = lasso.prefix[i];
        if (!out_path.empty() && out_path.back().x == p.x && out_path.back().y == p.y)
            continue;
        out_path.push_back(p);
        out_ids.push_back(lasso.prefix_ids[i]);
    }
    for (size_t i = 0; i < lasso.cycle.size(); i++) {
        const Pos& p = lasso.cycle[i];
        if (!out_path.empty() && out_path.back().x == p.x && out_path.back().y == p.y)
            continue;
        out_path.push_back(p);
        out_ids.push_back(lasso.cycle_ids[i]);
    }
}

// Length of the prefix AFTER the same dedup build_flat_path applies, so it can
// be used as an index into the deduped flat path (raw prefix.size() cannot).
static int dedup_prefix_len(const LassoResult& lasso)
{
    std::vector<Pos> pp; std::vector<unsigned> pi;
    LassoResult prefix_only{lasso.prefix, {}, lasso.prefix_ids, {}};
    build_flat_path(prefix_only, pp, pi);
    return (int)pp.size();
}

// Index at which the repeating loop restarts when the robot wraps around.
// The prefix ends at the accepting anchor and the cycle begins at that SAME
// anchor; build_flat_path dedups the shared cell into the prefix, so the loop
// must restart at the anchor itself (= deduped prefix length - 1), not at the
// first post-anchor cell. Using the raw length skips the anchor on every wrap.
static int cycle_restart_index(const LassoResult& lasso)
{
    int n = dedup_prefix_len(lasso);
    return n > 0 ? n - 1 : 0;
}

void dynamic_visulizer(
    GridWorld& world,
    const LassoResult& lasso,
    WPA& wpa,
    DStarPlanner& planner
){
    const int w = world.width();
    const int h = world.height();

    const int cellSize = 80;
    const int margin   = 20;

    const int screenW = margin * 2 + w * cellSize;
    const int screenH = margin * 2 + h * cellSize;

    InitWindow(screenW, screenH, "Path Trace");
    SetTargetFPS(60);

    std::vector<Pos>      path;
    std::vector<unsigned> path_ids;
    build_flat_path(lasso, path, path_ids);

    if (path.empty()) {
        std::cerr << "dynamic_visualizer: empty path, nothing to show\n";
        return;
    }

    int  step_index   = 0;
    int  cycle_start  = cycle_restart_index(lasso);
    bool replanned    = false;
    int  replan_count = 0;          // advances the overlay color on each replan
    float elapsed     = 0.0f;
    const float step_interval = 0.5f;

    // Distinct overlay colors cycled per replan, so overlapping replanned
    // paths stay tellable apart. Red is reserved for a currently-blocked path
    // and blue for the original (un-replanned) path; these are the rest.
    const Color replan_palette[] = {
        Color{255, 200,  80, 128},  // orange
        Color{200, 120, 255, 128},  // purple
        Color{ 80, 220, 200, 128},  // teal
        Color{255, 140, 200, 128},  // pink
        Color{180, 230,  90, 128},  // lime
        Color{120, 200, 255, 128},  // sky
    };
    const int replan_palette_n = (int)(sizeof(replan_palette) / sizeof(replan_palette[0]));

    while (!WindowShouldClose()) {

        // check for click updates - adding/removing obstacles
        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            int mouse_x = GetMouseX();
            int mouse_y = GetMouseY();
            int gx = (mouse_x - margin) / cellSize;
            int gy = (mouse_y - margin) / cellSize;
            if (world.in_bounds({gx, gy})) {
                world.set_blocked({gx, gy}, !world.is_blocked({gx, gy}));
                std::vector<unsigned> changed = detect_changed_states(wpa, {{gx, gy}});

                bool blocked = path_is_blocked(path, world);
                std::cout << "[DBG click] path blocked: " << (blocked ? "YES — replan needed" : "NO") << "\n";

                for (unsigned s : changed) {
                    std::cout << "  [DBG links] state " << s
                              << " (nba=" << wpa.nba_state_of(s) << ") neighbors:\n";
                    for (const WPA::Neighbor& nb : wpa.neighbors_ext(s)) {
                        std::cout << "    -> state " << nb.dst
                                  << " pos=(" << wpa.pos_of(nb.dst).x << "," << wpa.pos_of(nb.dst).y << ")"
                                  << " cost=" << nb.cost << "\n";
                    }
                }
                std::cout << "\n";

                bool is_now_blocked = world.is_blocked({gx, gy});
                unsigned current_state = path_ids[step_index];

                dbg("events.log")
                    << "[click] cell=(" << gx << "," << gy << ")"
                    << " now_blocked=" << is_now_blocked
                    << " path_blocked=" << blocked
                    << " robot step_index=" << step_index
                    << " current_state=" << current_state
                    << " pos=(" << wpa.pos_of(current_state).x << "," << wpa.pos_of(current_state).y << ")"
                    << " nba=" << wpa.nba_state_of(current_state)
                    << " #changed=" << changed.size() << "\n" << std::flush;

                LassoResult new_lasso = dstar_replan(wpa, planner, current_state, changed, is_now_blocked, planner.mode);

                if (!new_lasso.prefix.empty() || !new_lasso.cycle.empty()) {
                    build_flat_path(new_lasso, path, path_ids);
                    cycle_start = cycle_restart_index(new_lasso);
                    // The replanned path starts AT the robot's current cell
                    // (dstar_replan plans from current_state), so the robot is
                    // at index 0. Keeping the old step_index would teleport it.
                    step_index  = 0;
                    replanned   = true;
                    replan_count++;
                    dbg("events.log") << "  [click] replan OK: new prefix="
                        << new_lasso.prefix.size() << " cycle=" << new_lasso.cycle.size() << "\n" << std::flush;
                } else {
                    dbg("events.log") << "  [click] replan FAILED: empty lasso (no path)\n" << std::flush;
                }
            }
        }

        // draw everything from scratch
        BeginDrawing();
        ClearBackground(RAYWHITE);

        // Draw occupancy + grid
        for (int y = 0; y < h; ++y) {
            for (int x = 0; x < w; ++x) {
                const int px = margin + x * cellSize;
                const int py = margin + y * cellSize;

                if (world.is_blocked({x, y}))
                    DrawRectangle(px, py, cellSize, cellSize, DARKGRAY);
                DrawRectangleLines(px, py, cellSize, cellSize, LIGHTGRAY);
            }
        }

        bool needs_replan = path_is_blocked(path, world);

        // Draw path overlay
        for (const Pos& p : path) {
            if (!world.in_bounds({p.x, p.y})) continue;
            const int px = margin + p.x * cellSize;
            const int py = margin + p.y * cellSize;

            if (needs_replan)
                DrawRectangle(px, py, cellSize, cellSize, Color{252, 182, 182, 128}); // red — blocked
            else if (replanned)
                DrawRectangle(px, py, cellSize, cellSize,
                              replan_palette[(replan_count - 1) % replan_palette_n]); // cycles per replan
            else
                DrawRectangle(px, py, cellSize, cellSize, Color{197, 247, 250, 128}); // blue — original
        }

        // Highlight labeled cells (a, b, etc.)
        {
            const Color label_colors[] = {
                Color{0,   200,   0, 160},   // first label  — green
                Color{220,   0,   0, 160},   // second label — red
                Color{0,    80, 220, 160},   // third label  — blue
                Color{200, 120,   0, 160},   // fourth label — orange
            };
            const std::vector<std::string> names = world.label_names();
            for (int li = 0; li < (int)names.size(); ++li) {
                const Color col = label_colors[li % 4];
                for (int y = 0; y < h; ++y) {
                    for (int x = 0; x < w; ++x) {
                        if (!world.has_label({x, y}, names[li])) continue;
                        const int px = margin + x * cellSize;
                        const int py = margin + y * cellSize;
                        DrawRectangle(px, py, cellSize, cellSize, col);
                        DrawText(names[li].c_str(),
                                 px + cellSize/2 - 5,
                                 py + cellSize/2 - 8,
                                 16, WHITE);
                    }
                }
            }
        }

        // advance robot
        if (!needs_replan) {
            elapsed += GetFrameTime();
        }
        if (elapsed >= step_interval) {
            step_index++;
            if (step_index >= (int)path.size()) step_index = cycle_start;
            elapsed = 0.0f;

            unsigned sid = path_ids[step_index];
            dbg("events.log")
                << "[step] step_index=" << step_index
                << " state=" << sid
                << " pos=(" << wpa.pos_of(sid).x << "," << wpa.pos_of(sid).y << ")"
                << " nba=" << wpa.nba_state_of(sid) << "\n" << std::flush;
        }

        // draw robot
        const Pos& curr = path[step_index];
        DrawCircle(margin + curr.x * cellSize + cellSize/2,
                   margin + curr.y * cellSize + cellSize/2,
                   cellSize/3, Color{247, 235, 106, 255});

        EndDrawing();
    }
    CloseWindow();
}

// check to see if path blocked
bool path_is_blocked(
    const std::vector<Pos>& path, 
    const GridWorld& world) {
    for (const Pos& p : path) {
        if (world.is_blocked(p)) return true;
    }
    return false;
}


void static_visualizer(
    const GridWorld& world,
    const std::vector<std::vector<Pos>>& path_segments
){
    const int w = world.width();
    const int h = world.height();

    const int cellSize = 80;
    const int margin   = 20;

    const int screenW = margin * 2 + w * cellSize;
    const int screenH = margin * 2 + h * cellSize;

    InitWindow(screenW, screenH, "Path Trace");
    SetTargetFPS(60);

    const auto path = flatten_path(path_segments);

    while (!WindowShouldClose()) {
        BeginDrawing();
        ClearBackground(RAYWHITE);

        // Draw occupancy + grid
        for (int y = 0; y < h; ++y) {
            for (int x = 0; x < w; ++x) {
                const int px = margin + x * cellSize;
                const int py = margin + y * cellSize;

                if (world.is_blocked({x,y})) {
                    DrawRectangle(px, py, cellSize, cellSize, DARKGRAY);
                }
                DrawRectangleLines(px, py, cellSize, cellSize, LIGHTGRAY);
            }
        }

        // Draw path overlay
        for (const auto& p : path) {
            if (!world.in_bounds({p.x, p.y})) continue;

            const int px = margin + p.x * cellSize;
            const int py = margin + p.y * cellSize;
            DrawRectangle(px, py, cellSize, cellSize, Color{0, 121, 241, 140});
        }

        // Highlight start/end
        if (!path.empty()) {
            const Pos s = path.front();
            const Pos g = path.back();

            DrawRectangle(margin + s.x * cellSize, margin + s.y * cellSize,
                          cellSize, cellSize, Color{0, 200, 0, 170});   // start

            DrawRectangle(margin + g.x * cellSize, margin + g.y * cellSize,
                          cellSize, cellSize, Color{220, 0, 0, 170});   // goal
        }

        EndDrawing();
    }

    CloseWindow();
}

