/*
    Take in grid world and vizulize it

    think i am going to use this has insiration
    https://github.com/anson10/Path-Finding-Visualiser
*/
#include <iostream>
#include <vector>
#include <algorithm>
#include <chrono>
#include <sstream>
#include <iomanip>
#include "raylib.h"

#include "grid_vis.hpp"
#include "grid_world.hpp"
#include "ts.hpp"
#include "types.hpp"
#include "dstar.hpp"
#include "sensing.hpp"
#include "debug_log.hpp"


// private functions
bool path_is_blocked(const std::vector<Pos>& path, const GridWorld& belief);

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

// Metrics shown in the bottom strip — refreshed whenever the path changes
// (initial plan or any replan) and once per frame for the live g(current).
struct Metrics {
    int    prefix_len   = 0;
    int    cycle_len    = 0;
    int    replan_count = 0;
    double last_replan_ms = 0.0;
    double g_current    = 0.0;
};

// Runs sense()+reveal() from the robot's current cell, and if belief actually
// changed, builds the StateChange batch and replans. Shared by the initial
// pre-loop sense and every subsequent tick so both go through identical
// logic. Returns true if a replan happened (caller should adopt new_lasso).
// `out_replan_ms` is only written when a replan actually ran (times just the
// dstar_replan() call, not sense/reveal).
static bool sense_reveal_and_replan(
    const GridWorld& world,
    GridWorld& robot_map,
    const SensorConfig& sensor_cfg,
    WPA& wpa,
    DStarPlanner& planner,
    unsigned current_state,
    LassoResult& out_new_lasso,
    double& out_replan_ms
){
    std::vector<Pos> visible = sense(sensor_cfg, wpa.pos_of(current_state), world);
    std::vector<Pos> diff    = reveal(robot_map, world, visible);

    if (diff.empty()) return false;

    std::vector<unsigned> touched = detect_changed_states(wpa, diff);
    std::vector<StateChange> changes = build_state_changes(wpa, robot_map, touched);

    dbg("events.log")
        << "[sense] center=(" << wpa.pos_of(current_state).x << "," << wpa.pos_of(current_state).y << ")"
        << " visible=" << visible.size()
        << " diff=" << diff.size()
        << " state_changes=" << changes.size() << "\n" << std::flush;

    if (changes.empty()) return false;   // belief moved but no state actually changed cost

    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    out_new_lasso = dstar_replan(wpa, planner, current_state, changes, planner.mode);
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    out_replan_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    dbg("events.log")
        << "  [sense] replan " << (out_new_lasso.prefix.empty() && out_new_lasso.cycle.empty() ? "FAILED (empty lasso)" : "OK")
        << " new prefix=" << out_new_lasso.prefix.size()
        << " cycle=" << out_new_lasso.cycle.size()
        << " took=" << out_replan_ms << "ms\n" << std::flush;

    return true;
}

// ------------------------------------------------------------------
// Debug overlay helper (functional debug aid, not "pretty" — kept
// intentionally minimal; the real visual pass is future work, see TODO.md).
// ------------------------------------------------------------------

// Outline of the sensed region around `center`, shaped per cfg.metric.
static void draw_sensor_outline(
    const SensorConfig& cfg, Pos center,
    int originX, int originY, int cellSize, Color col
){
    int cx = originX + center.x * cellSize + cellSize / 2;
    int cy = originY + center.y * cellSize + cellSize / 2;

    switch (cfg.metric) {
        case SensorMetric::Euclidean: {
            DrawCircleLines(cx, cy, (float)(cfg.radius * cellSize), col);
            break;
        }
        case SensorMetric::Chebyshev: {
            int half = (int)(cfg.radius * cellSize) + cellSize / 2;
            DrawRectangleLines(cx - half, cy - half, half * 2, half * 2, col);
            break;
        }
        case SensorMetric::Manhattan:
        default: {
            int r = (int)(cfg.radius * cellSize) + cellSize / 2;
            Vector2 top{ (float)cx, (float)(cy - r) };
            Vector2 right{ (float)(cx + r), (float)cy };
            Vector2 bottom{ (float)cx, (float)(cy + r) };
            Vector2 left{ (float)(cx - r), (float)cy };
            DrawLineV(top, right, col);
            DrawLineV(right, bottom, col);
            DrawLineV(bottom, left, col);
            DrawLineV(left, top, col);
            break;
        }
    }
}

// Draws one pane's grid + occupancy (Static and Dynamic both render as
// DARKGRAY — this pass is about the dual-map/interaction capability, not
// distinguishing object types visually; see TODO.md #7 for the deferred
// visual redesign).
static void draw_pane_occupancy(
    const GridWorld& g, int originX, int originY, int cellSize
){
    for (int y = 0; y < g.height(); ++y) {
        for (int x = 0; x < g.width(); ++x) {
            const int px = originX + x * cellSize;
            const int py = originY + y * cellSize;
            if (g.is_blocked({x, y}))
                DrawRectangle(px, py, cellSize, cellSize, DARKGRAY);
            DrawRectangleLines(px, py, cellSize, cellSize, LIGHTGRAY);
        }
    }
}

static void draw_pane_labels(
    const GridWorld& g, int originX, int originY, int cellSize
){
    const Color label_colors[] = {
        Color{0,   200,   0, 160},   // first label  — green
        Color{220,   0,   0, 160},   // second label — red
        Color{0,    80, 220, 160},   // third label  — blue
        Color{200, 120,   0, 160},   // fourth label — orange
    };
    const std::vector<std::string> names = g.label_names();
    for (int li = 0; li < (int)names.size(); ++li) {
        const Color col = label_colors[li % 4];
        for (int y = 0; y < g.height(); ++y) {
            for (int x = 0; x < g.width(); ++x) {
                if (!g.has_label({x, y}, names[li])) continue;
                const int px = originX + x * cellSize;
                const int py = originY + y * cellSize;
                DrawRectangle(px, py, cellSize, cellSize, col);
                DrawText(names[li].c_str(),
                         px + cellSize/2 - 5,
                         py + cellSize/2 - 8,
                         16, WHITE);
            }
        }
    }
}

static void draw_robot_marker(Pos p, int originX, int originY, int cellSize)
{
    DrawCircle(originX + p.x * cellSize + cellSize/2,
               originY + p.y * cellSize + cellSize/2,
               cellSize/3, Color{247, 235, 106, 255});
}

static void draw_metrics_strip(
    const Metrics& m, int x, int y, int width, int height
){
    DrawRectangle(x, y, width, height, Color{30, 30, 30, 255});

    std::ostringstream os;
    os << std::fixed << std::setprecision(3);
    os << "prefix: " << m.prefix_len
       << "   cycle: " << m.cycle_len
       << "   total path: " << (m.prefix_len + m.cycle_len)
       << "   replans: " << m.replan_count
       << "   last replan: " << m.last_replan_ms << " ms"
       << "   g(current): " << m.g_current;

    DrawText(os.str().c_str(), x + 10, y + height/2 - 8, 18, RAYWHITE);
}

void dynamic_visulizer(
    GridWorld& world,
    GridWorld& robot_map,
    const LassoResult& lasso,
    WPA& wpa,
    DStarPlanner& planner,
    const SensorConfig& sensor_cfg
){
    const int w = world.width();
    const int h = world.height();

    const int cellSize      = 80;
    const int margin        = 20;
    const int paneGap       = 50;
    const int paneLabelH    = 28;   // space above each pane for its title
    const int metricsHeight = 40;

    const int worldOriginX = margin;
    const int robotOriginX = worldOriginX + w * cellSize + paneGap;
    const int paneOriginY  = margin + paneLabelH;

    const int screenW = margin * 2 + w * cellSize * 2 + paneGap;
    const int screenH = paneOriginY + h * cellSize + margin + metricsHeight;

    InitWindow(screenW, screenH, "World / Robot belief");
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
    bool debug_overlay = true;      // toggle with 'D'

    Metrics metrics;
    metrics.prefix_len = dedup_prefix_len(lasso);
    metrics.cycle_len  = (int)path.size() - metrics.prefix_len;

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

    // Initial sense from the robot's starting cell — seed_from_static only
    // gives the robot Static structure, so without this it starts blind to
    // any Dynamic cell already within radius at t=0.
    {
        LassoResult new_lasso;
        double replan_ms = 0.0;
        if (sense_reveal_and_replan(world, robot_map, sensor_cfg, wpa, planner,
                                     path_ids[step_index], new_lasso, replan_ms)) {
            if (!new_lasso.prefix.empty() || !new_lasso.cycle.empty()) {
                build_flat_path(new_lasso, path, path_ids);
                cycle_start = cycle_restart_index(new_lasso);
                step_index  = 0;
                replanned   = true;
                replan_count++;
                metrics.prefix_len     = dedup_prefix_len(new_lasso);
                metrics.cycle_len      = (int)path.size() - metrics.prefix_len;
                metrics.replan_count   = replan_count;
                metrics.last_replan_ms = replan_ms;
            }
        }
    }

    while (!WindowShouldClose()) {

        if (IsKeyPressed(KEY_D)) debug_overlay = !debug_overlay;

        // check for click updates — only inside the WORLD pane, writes ground
        // truth only. The robot only finds out once sense()/reveal() (below)
        // brings it within radius; clicking far from the robot will look
        // like nothing happened in the robot pane until then — that's
        // intended (partial observability), not a bug. The world pane shows
        // the click immediately, since it's ground truth.
        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            int mouse_x = GetMouseX();
            int mouse_y = GetMouseY();
            if (mouse_x >= worldOriginX && mouse_x < worldOriginX + w * cellSize &&
                mouse_y >= paneOriginY  && mouse_y < paneOriginY + h * cellSize) {
                int gx = (mouse_x - worldOriginX) / cellSize;
                int gy = (mouse_y - paneOriginY) / cellSize;
                if (world.in_bounds({gx, gy})) {
                    world.set_dynamic({gx, gy}, !world.is_dynamic({gx, gy}));

                    dbg("events.log")
                        << "[click] cell=(" << gx << "," << gy << ")"
                        << " now_dynamic=" << world.is_dynamic({gx, gy})
                        << " (ground truth only — robot not yet informed)\n" << std::flush;
                }
            }
        }

        // draw everything from scratch
        BeginDrawing();
        ClearBackground(RAYWHITE);

        DrawText("World (ground truth)", worldOriginX, margin, 20, BLACK);
        DrawText("Robot belief",         robotOriginX, margin, 20, BLACK);

        // Left pane: ground truth. Right pane: robot's belief map.
        draw_pane_occupancy(world,     worldOriginX, paneOriginY, cellSize);
        draw_pane_occupancy(robot_map, robotOriginX, paneOriginY, cellSize);

        bool needs_replan = path_is_blocked(path, robot_map);

        // Path/lasso overlay — robot pane only, since that's what the robot
        // is actually following (planned off belief, not ground truth).
        for (const Pos& p : path) {
            if (!world.in_bounds({p.x, p.y})) continue;
            const int px = robotOriginX + p.x * cellSize;
            const int py = paneOriginY  + p.y * cellSize;

            if (needs_replan)
                DrawRectangle(px, py, cellSize, cellSize, Color{252, 182, 182, 128}); // red — blocked
            else if (replanned)
                DrawRectangle(px, py, cellSize, cellSize,
                              replan_palette[(replan_count - 1) % replan_palette_n]); // cycles per replan
            else
                DrawRectangle(px, py, cellSize, cellSize, Color{197, 247, 250, 128}); // blue — original
        }

        // Labels are world truth, shown on both panes (robot_map was seeded
        // with the same labels in seed_from_static).
        draw_pane_labels(world,     worldOriginX, paneOriginY, cellSize);
        draw_pane_labels(robot_map, robotOriginX, paneOriginY, cellSize);

        if (debug_overlay) {
            // draw_sensor_outline(sensor_cfg, path[step_index], worldOriginX, paneOriginY, cellSize, Color{0, 120, 255, 200});   // commented out: robot/radius not wanted on the world pane (for now)
            draw_sensor_outline(sensor_cfg, path[step_index], robotOriginX, paneOriginY, cellSize, Color{0, 120, 255, 200});
        }

        // advance robot
        if (!needs_replan) {
            elapsed += GetFrameTime();
        }
        bool stepped = false;
        if (elapsed >= step_interval) {
            step_index++;
            if (step_index >= (int)path.size()) step_index = cycle_start;
            elapsed = 0.0f;
            stepped = true;

            unsigned sid = path_ids[step_index];
            dbg("events.log")
                << "[step] step_index=" << step_index
                << " state=" << sid
                << " pos=(" << wpa.pos_of(sid).x << "," << wpa.pos_of(sid).y << ")"
                << " nba=" << wpa.nba_state_of(sid) << "\n" << std::flush;
        }

        // Sense from the robot's (possibly new) cell each tick — this is the
        // SensingCadence::EveryTick behavior; OnMove/OnDemand would gate this
        // differently once implemented (see sensing.hpp).
        if (stepped && sensor_cfg.cadence == SensingCadence::EveryTick) {
            LassoResult new_lasso;
            double replan_ms = 0.0;
            if (sense_reveal_and_replan(world, robot_map, sensor_cfg, wpa, planner,
                                         path_ids[step_index], new_lasso, replan_ms)) {
                if (!new_lasso.prefix.empty() || !new_lasso.cycle.empty()) {
                    build_flat_path(new_lasso, path, path_ids);
                    cycle_start = cycle_restart_index(new_lasso);
                    // The replanned path starts AT the robot's current cell
                    // (dstar_replan plans from current_state), so the robot is
                    // at index 0. Keeping the old step_index would teleport it.
                    step_index  = 0;
                    replanned   = true;
                    replan_count++;
                    metrics.prefix_len     = dedup_prefix_len(new_lasso);
                    metrics.cycle_len      = (int)path.size() - metrics.prefix_len;
                    metrics.replan_count   = replan_count;
                    metrics.last_replan_ms = replan_ms;
                }
            }
        }

        // draw robot — robot pane only (for now); world pane intentionally
        // left without the robot/radius overlay.
        // draw_robot_marker(path[step_index], worldOriginX, paneOriginY, cellSize);
        draw_robot_marker(path[step_index], robotOriginX, paneOriginY, cellSize);

        metrics.g_current = get_g(planner.prefix.g, path_ids[step_index]);
        draw_metrics_strip(metrics, 0, paneOriginY + h * cellSize + margin, screenW, metricsHeight);

        EndDrawing();
    }
    CloseWindow();
}

// check to see if path blocked, per the robot's own belief
bool path_is_blocked(
    const std::vector<Pos>& path,
    const GridWorld& belief) {
    for (const Pos& p : path) {
        if (belief.is_blocked(p)) return true;
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
