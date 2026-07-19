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
#include <fstream>
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
// Theme — one place to tweak the look. Kept simple (flat colors, no
// textures/fonts) but with enough visual hierarchy (cards, headers, rounded
// cells, line-based paths instead of stacked translucent rectangles) that
// panes read as a scene instead of a flat stack of overlays.
// ------------------------------------------------------------------
namespace theme {
    const Color background     = Color{234, 237, 241, 255};
    const Color panel_bg       = Color{252, 252, 253, 255};
    const Color panel_border   = Color{217, 221, 227, 255};
    const Color grid_line      = Color{223, 226, 231, 255};
    const Color cell_free      = Color{255, 255, 255, 255};
    const Color cell_static    = Color{57,  62,  70,  255};   // wall — charcoal
    const Color cell_dynamic   = Color{196, 88,  61,  255};   // obstacle — rust
    const Color header_world   = Color{71,  85,  105, 255};   // slate
    const Color header_robot   = Color{12,  133, 122, 255};   // teal
    const Color path_original  = Color{37,  99,  180, 255};   // blue   — prefix, no replan yet
    const Color path_cycle     = Color{124, 58,  237, 255};   // purple — loop, no replan yet
    const Color path_blocked   = Color{217, 45,  32,  255};   // red    — currently blocked (both)
    const Color robot_fill     = Color{250, 204, 21,  255};   // amber
    const Color robot_outline  = Color{161, 98,  7,   255};
    const Color sensor_outline = Color{59,  130, 246, 150};
    const Color metrics_bg     = Color{26,  28,  32,  255};
    const Color metrics_accent = header_robot;
    const Color metrics_text   = Color{221, 225, 230, 255};   // light text — for the dark metrics strip
    const Color status_text    = Color{40,  46,  54,  255};   // dark text — for the light background

    const Color path_palette[] = {
        Color{219, 140, 15,  255},  // amber
        Color{147, 51,  190, 255},  // purple
        Color{13,  148, 136, 255},  // teal
        Color{219, 39,  119, 255},  // pink
        Color{101, 163, 13,  255},  // lime
        Color{29,  120, 200, 255},  // sky
    };
    const int path_palette_n = (int)(sizeof(path_palette) / sizeof(path_palette[0]));

    const Color label_colors[] = {
        Color{34,  197, 94,  215},  // green
        Color{239, 68,  68,  215},  // red
        Color{59,  130, 246, 215},  // blue
        Color{249, 115, 22,  215},  // orange
    };
}

// ------------------------------------------------------------------
// Drawing helpers
// ------------------------------------------------------------------

// Card-style panel: soft drop shadow, background, border, and a colored
// header bar with a centered title.
static void draw_panel(
    int x, int y, int width, int height, int headerH,
    const char* title, Color headerColor
){
    DrawRectangle(x + 4, y + 4, width, height, Fade(BLACK, 0.08f));
    DrawRectangle(x, y, width, height, theme::panel_bg);
    DrawRectangleLinesEx(Rectangle{(float)x, (float)y, (float)width, (float)height}, 1.5f, theme::panel_border);

    DrawRectangle(x, y, width, headerH, headerColor);
    int textW = MeasureText(title, 18);
    DrawText(title, x + (width - textW) / 2, y + headerH / 2 - 9, 18, RAYWHITE);
}

// Rounded, inset cells instead of edge-to-edge flat rectangles — gives each
// cell a little breathing room so the grid reads as a scene, not a filled
// bitmap. Static/Dynamic occupancy get distinct colors (wall vs obstacle).
static void draw_pane_occupancy(
    const GridWorld& g, int originX, int originY, int cellSize
){
    const int inset = 3;
    for (int y = 0; y < g.height(); ++y) {
        for (int x = 0; x < g.width(); ++x) {
            Rectangle r{
                (float)(originX + x * cellSize + inset),
                (float)(originY + y * cellSize + inset),
                (float)(cellSize - inset * 2),
                (float)(cellSize - inset * 2)
            };
            Color fill = g.is_static({x, y})  ? theme::cell_static
                       : g.is_dynamic({x, y}) ? theme::cell_dynamic
                       :                        theme::cell_free;
            DrawRectangleRounded(r, 0.22f, 8, fill);
            DrawRectangleRoundedLines(r, 0.22f, 8, theme::grid_line);
        }
    }
}

// Labels as colored circular badges (rather than a full-cell fill) so they
// stay legible without blotting out the occupancy/path underneath.
static void draw_pane_labels(
    const GridWorld& g, int originX, int originY, int cellSize
){
    const std::vector<std::string> names = g.label_names();
    for (int li = 0; li < (int)names.size(); ++li) {
        Color col = theme::label_colors[li % 4];
        for (int y = 0; y < g.height(); ++y) {
            for (int x = 0; x < g.width(); ++x) {
                if (!g.has_label({x, y}, names[li])) continue;
                int cx = originX + x * cellSize + cellSize / 2;
                int cy = originY + y * cellSize + cellSize / 2;
                float r = cellSize * 0.32f;
                DrawCircle(cx, cy, r, col);
                DrawCircleLines(cx, cy, r, Fade(BLACK, 0.25f));
                int tw = MeasureText(names[li].c_str(), 18);
                DrawText(names[li].c_str(), cx - tw / 2, cy - 9, 18, WHITE);
            }
        }
    }
}

// Path/lasso as a connected line through cell centers (with small node dots
// and the loop-closing segment back to the cycle anchor) instead of filling
// every path cell with a translucent rectangle — avoids the previous "muddy
// soup" where path + label + occupancy overlays all stacked on each other.
// prefix_color/cycle_color are distinct so the one-time run-up and the
// repeating loop read as different things at a glance; a segment/node
// belongs to the cycle once its start index is >= cycle_start (the shared
// anchor cell itself renders in cycle_color).
static void draw_path_overlay(
    const std::vector<Pos>& path, int cycle_start,
    int originX, int originY, int cellSize,
    Color prefix_color, Color cycle_color
){
    if (path.size() < 2) return;

    auto center = [&](const Pos& p) -> Vector2 {
        return Vector2{
            (float)(originX + p.x * cellSize + cellSize / 2),
            (float)(originY + p.y * cellSize + cellSize / 2)
        };
    };

    for (size_t i = 1; i < path.size(); ++i) {
        Color seg_color = ((int)(i - 1) >= cycle_start) ? cycle_color : prefix_color;
        DrawLineEx(center(path[i - 1]), center(path[i]), 4.0f, seg_color);
    }

    if (cycle_start >= 0 && cycle_start < (int)path.size())
        DrawLineEx(center(path.back()), center(path[cycle_start]), 4.0f, Fade(cycle_color, 0.6f));

    for (size_t i = 0; i < path.size(); ++i) {
        Color node_color = ((int)i >= cycle_start) ? cycle_color : prefix_color;
        Vector2 c = center(path[i]);
        DrawCircle((int)c.x, (int)c.y, 5.0f, node_color);
    }
}

// "(x,y) -> (x,y) -> ..." for path[from..to] inclusive.
static std::string format_positions(const std::vector<Pos>& path, int from, int to)
{
    std::ostringstream os;
    for (int i = from; i <= to; ++i) {
        if (i > from) os << " -> ";
        os << "(" << path[i].x << "," << path[i].y << ")";
    }
    return os.str();
}

static void draw_robot_marker(Pos p, int originX, int originY, int cellSize)
{
    int cx = originX + p.x * cellSize + cellSize / 2;
    int cy = originY + p.y * cellSize + cellSize / 2;
    DrawCircle(cx, cy, cellSize * 0.42f, Fade(theme::robot_fill, 0.25f));   // soft glow
    DrawCircle(cx, cy, cellSize * 0.26f, theme::robot_fill);
    DrawCircleLines(cx, cy, cellSize * 0.26f, theme::robot_outline);
}

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
            DrawRectangleLinesEx(Rectangle{(float)(cx - half), (float)(cy - half), (float)(half * 2), (float)(half * 2)}, 2.0f, col);
            break;
        }
        case SensorMetric::Manhattan:
        default: {
            int r = (int)(cfg.radius * cellSize) + cellSize / 2;
            Vector2 top{ (float)cx, (float)(cy - r) };
            Vector2 right{ (float)(cx + r), (float)cy };
            Vector2 bottom{ (float)cx, (float)(cy + r) };
            Vector2 left{ (float)(cx - r), (float)cy };
            DrawLineEx(top, right, 2.0f, col);
            DrawLineEx(right, bottom, 2.0f, col);
            DrawLineEx(bottom, left, 2.0f, col);
            DrawLineEx(left, top, 2.0f, col);
            break;
        }
    }
}

static void draw_metrics_strip(
    const Metrics& m, int x, int y, int width, int height,
    const std::string& prefix_text, const std::string& cycle_text,
    Color prefix_color, Color cycle_color
){
    DrawRectangle(x, y, width, height, theme::metrics_bg);
    DrawRectangle(x, y, width, 3, theme::metrics_accent);

    std::ostringstream os;
    os << std::fixed << std::setprecision(2);
    os << "PREFIX " << m.prefix_len
       << "     CYCLE " << m.cycle_len
       << "     TOTAL " << (m.prefix_len + m.cycle_len)
       << "     REPLANS " << m.replan_count
       << "     LAST REPLAN " << m.last_replan_ms << " ms"
       << "     g(current) " << m.g_current;

    DrawText(os.str().c_str(), x + 16, y + 12, 18, theme::metrics_text);

    std::string prefix_line = "PREFIX PATH: " + prefix_text;
    std::string cycle_line  = "LOOP PATH: "   + cycle_text;
    DrawText(prefix_line.c_str(), x + 16, y + 40, 15, prefix_color);
    DrawText(cycle_line.c_str(),  x + 16, y + 62, 15, cycle_color);
}

// ------------------------------------------------------------------
// Testing/UX helpers: status bar + legend, world layout save/load,
// metrics-to-CSV. All keyboard-toggle based (no raygui dependency).
// ------------------------------------------------------------------

static const char* suffix_mode_name(SuffixMode m)
{
    return m == SuffixMode::OPTION1_ASTAR ? "OPTION1_ASTAR" : "OPTION2_INCREMENTAL";
}

static const char* replan_mode_name(ReplanMode m)
{
    return m == ReplanMode::DSTAR_INCREMENTAL ? "DSTAR_INCREMENTAL" : "FULL_RECOMPUTE";
}

// One-line control readout, so a screen recording is self-documenting
// without narration.
static void draw_status_line(
    int x, int y, bool paused, SuffixMode suffix_mode, ReplanMode replan_mode,
    bool placing_static, bool debug_overlay, float step_interval, double sensor_radius
){
    std::ostringstream os;
    os << std::fixed << std::setprecision(2);
    os << "[SPACE] " << (paused ? "PAUSED" : "RUNNING")
       << "  [RIGHT] step-once(paused)"
       << "  [M] suffix=" << suffix_mode_name(suffix_mode)
       << "  [N] replan=" << replan_mode_name(replan_mode)
       << "  [T] placing=" << (placing_static ? "STATIC" : "DYNAMIC")
       << "  [D] debug=" << (debug_overlay ? "ON" : "OFF")
       << "  [+/-] speed=" << step_interval << "s"
       << "  [[/]] radius=" << sensor_radius
       << "  [F2] shot  [F5] save  [F9] load";

    DrawText(os.str().c_str(), x, y, 15, theme::status_text);
}

// Static/Dynamic + per-label color swatches, so a shared screenshot doesn't
// need the `theme` source to be legible.
static void draw_legend(int x, int y, const GridWorld& world)
{
    int cx = x;
    auto swatch = [&](Color col, const char* label) {
        DrawRectangle(cx, y, 14, 14, col);
        DrawRectangleLines(cx, y, 14, 14, Fade(BLACK, 0.35f));
        DrawText(label, cx + 20, y - 1, 14, theme::status_text);
        cx += 20 + MeasureText(label, 14) + 18;
    };
    swatch(theme::cell_static, "Static");
    swatch(theme::cell_dynamic, "Dynamic");
    const std::vector<std::string> names = world.label_names();
    for (int li = 0; li < (int)names.size(); ++li)
        swatch(theme::label_colors[li % 4], names[li].c_str());
}

// Plain-text world layout, human-readable/hand-editable:
//   WIDTH w / HEIGHT h / LABEL name x y / STATIC x y / DYNAMIC x y
static void save_world_layout(const GridWorld& world, const std::string& path)
{
    std::ofstream os(path, std::ios::out | std::ios::trunc);
    if (!os) { std::cerr << "save_world_layout: could not open " << path << "\n"; return; }

    os << "WIDTH "  << world.width()  << "\n";
    os << "HEIGHT " << world.height() << "\n";

    for (const std::string& name : world.label_names())
        for (int y = 0; y < world.height(); ++y)
            for (int x = 0; x < world.width(); ++x)
                if (world.has_label({x, y}, name))
                    os << "LABEL " << name << " " << x << " " << y << "\n";

    for (int y = 0; y < world.height(); ++y) {
        for (int x = 0; x < world.width(); ++x) {
            if (world.is_static({x, y}))  os << "STATIC "  << x << " " << y << "\n";
            if (world.is_dynamic({x, y})) os << "DYNAMIC " << x << " " << y << "\n";
        }
    }
}

// Only touches `world` (ground truth) — the robot only ever learns about it
// via reveal(), same as any other ground-truth edit. Requires matching
// WIDTH/HEIGHT (this session's product/WPA is already built for a fixed
// size); mismatched files are rejected rather than resized.
static bool load_world_layout(GridWorld& world, const std::string& path)
{
    std::ifstream is(path);
    if (!is) { std::cerr << "load_world_layout: could not open " << path << "\n"; return false; }

    std::string tag;
    int w = 0, h = 0;
    is >> tag >> w >> tag >> h;
    if (w != world.width() || h != world.height()) {
        std::cerr << "load_world_layout: " << w << "x" << h << " in file vs "
                   << world.width() << "x" << world.height() << " in sim — skipping\n";
        return false;
    }

    const std::vector<std::string> names = world.label_names();
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            world.set_free({x, y});
            for (const std::string& name : names)
                world.set_label({x, y}, name, false);
        }
    }

    while (is >> tag) {
        if (tag == "LABEL") {
            std::string name; int x, y;
            is >> name >> x >> y;
            world.set_label({x, y}, name, true);
        } else if (tag == "STATIC") {
            int x, y; is >> x >> y;
            world.set_static({x, y}, true);
        } else if (tag == "DYNAMIC") {
            int x, y; is >> x >> y;
            world.set_dynamic({x, y}, true);
        }
    }
    return true;
}

static void log_metrics_csv(const Metrics& m, int step_index, Pos pos)
{
    std::ofstream os("output/metrics.csv", std::ios::out | std::ios::app);
    os << step_index << "," << pos.x << "," << pos.y << ","
       << m.prefix_len << "," << m.cycle_len << "," << (m.prefix_len + m.cycle_len) << ","
       << m.replan_count << "," << std::fixed << std::setprecision(4) << m.last_replan_ms
       << "," << m.g_current << "\n";
}

void dynamic_visulizer(
    GridWorld& world,
    GridWorld& robot_map,
    const LassoResult& lasso,
    WPA& wpa,
    DStarPlanner& planner,
    SensorConfig& sensor_cfg
){
    const int w = world.width();
    const int h = world.height();

    const int cellSize      = 80;
    const int outerMargin   = 24;
    const int panelPad      = 10;
    const int headerH       = 34;
    const int panelGap      = 40;
    const int statusBarH    = 56;   // control readout line + legend line
    const int metricsGap    = 16;
    const int metricsHeight = 90;   // numeric line + prefix line + loop line

    const int panelWidth  = w * cellSize + panelPad * 2;
    const int panelHeight = headerH + h * cellSize + panelPad * 2;

    const int worldPanelX = outerMargin;
    const int panelY      = outerMargin + statusBarH;
    const int robotPanelX = worldPanelX + panelWidth + panelGap;

    const int worldOriginX = worldPanelX + panelPad;
    const int robotOriginX = robotPanelX + panelPad;
    const int paneOriginY  = panelY + headerH + panelPad;

    const int screenW = outerMargin * 2 + panelWidth * 2 + panelGap;
    const int screenH = panelY + panelHeight + metricsGap + metricsHeight + outerMargin;

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
    float step_interval = 0.5f;     // adjustable with '+'/'-'
    const float step_interval_min = 0.05f;
    const float step_interval_max = 2.0f;
    float sense_elapsed = 0.0f;
    const float sense_interval = 0.25f;   // independent of step_interval — see sense_due below
    bool debug_overlay = true;      // toggle with 'D'

    bool paused         = false;    // toggle with SPACE — freezes movement+sensing, not drawing
    bool placing_static = false;    // toggle with 'T' — click places Static instead of Dynamic
    int  shot_count     = 0;        // screenshot filename counter, F2

    Metrics metrics;
    metrics.prefix_len = dedup_prefix_len(lasso);
    metrics.cycle_len  = (int)path.size() - metrics.prefix_len;

    // Truncate + header once per run; log_metrics_csv() appends after this.
    {
        std::ofstream os("output/metrics.csv", std::ios::out | std::ios::trunc);
        os << "step_index,pos_x,pos_y,prefix_len,cycle_len,total_len,replan_count,last_replan_ms,g_current\n";
    }

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

    bool single_step_forced = false;

    while (!WindowShouldClose()) {

        if (IsKeyPressed(KEY_D)) debug_overlay = !debug_overlay;
        if (IsKeyPressed(KEY_SPACE)) paused = !paused;
        if (IsKeyPressed(KEY_T)) placing_static = !placing_static;

        if (paused && IsKeyPressed(KEY_RIGHT)) {
            // Force this frame's movement+sense block to run exactly once,
            // even though paused — the shared !needs_replan/elapsed and
            // sense_elapsed thresholds below are what actually gate it, so
            // pre-loading them past threshold is enough; no separate code
            // path needed.
            single_step_forced = true;
            elapsed       = step_interval;
            sense_elapsed = sense_interval;
        }

        if (IsKeyPressed(KEY_EQUAL)) step_interval = std::max(step_interval_min, step_interval - 0.05f);
        if (IsKeyPressed(KEY_MINUS)) step_interval = std::min(step_interval_max, step_interval + 0.05f);
        if (IsKeyPressed(KEY_RIGHT_BRACKET)) sensor_cfg.radius = std::min(10.0, sensor_cfg.radius + 0.5);
        if (IsKeyPressed(KEY_LEFT_BRACKET))  sensor_cfg.radius = std::max(0.5,  sensor_cfg.radius - 0.5);

        if (IsKeyPressed(KEY_M)) {
            // Rebuilds cycle tables from the WPA's CURRENT edge weights, so
            // everything discovered so far is preserved — only the suffix
            // strategy changes (same idea as dstar_replan's FULL_RECOMPUTE
            // branch, parameterized on suffix_mode instead of mode).
            SuffixMode new_suffix = (planner.suffix_mode == SuffixMode::OPTION1_ASTAR)
                ? SuffixMode::OPTION2_INCREMENTAL : SuffixMode::OPTION1_ASTAR;
            planner = make_planner(wpa, planner.mode, new_suffix);
            unsigned current_state = path_ids[step_index];
            LassoResult rebuilt = dstar_plan(wpa, planner, current_state);
            if (!rebuilt.prefix.empty() || !rebuilt.cycle.empty()) {
                build_flat_path(rebuilt, path, path_ids);
                cycle_start = cycle_restart_index(rebuilt);
                step_index  = 0;
                metrics.prefix_len = dedup_prefix_len(rebuilt);
                metrics.cycle_len  = (int)path.size() - metrics.prefix_len;
            }
            dbg("events.log") << "[mode] suffix_mode -> " << suffix_mode_name(planner.suffix_mode) << "\n" << std::flush;
        }
        if (IsKeyPressed(KEY_N)) {
            planner.mode = (planner.mode == ReplanMode::DSTAR_INCREMENTAL)
                ? ReplanMode::FULL_RECOMPUTE : ReplanMode::DSTAR_INCREMENTAL;
            dbg("events.log") << "[mode] replan_mode -> " << replan_mode_name(planner.mode) << "\n" << std::flush;
        }

        if (IsKeyPressed(KEY_F2)) {
            std::ostringstream fn;
            fn << "output/screenshot_" << std::setw(3) << std::setfill('0') << shot_count++ << ".png";
            TakeScreenshot(fn.str().c_str());
            dbg("events.log") << "[screenshot] saved " << fn.str() << "\n" << std::flush;
        }
        if (IsKeyPressed(KEY_F5)) {
            save_world_layout(world, "output/world_layout.txt");
            dbg("events.log") << "[layout] saved output/world_layout.txt\n" << std::flush;
        }
        if (IsKeyPressed(KEY_F9)) {
            if (load_world_layout(world, "output/world_layout.txt"))
                dbg("events.log") << "[layout] loaded output/world_layout.txt\n" << std::flush;
        }

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
                    if (placing_static) {
                        world.set_static({gx, gy}, !world.is_static({gx, gy}));
                        dbg("events.log")
                            << "[click] cell=(" << gx << "," << gy << ")"
                            << " now_static=" << world.is_static({gx, gy})
                            << " (ground truth only — placed after build won't remove a graph node)\n" << std::flush;
                    } else {
                        world.set_dynamic({gx, gy}, !world.is_dynamic({gx, gy}));
                        dbg("events.log")
                            << "[click] cell=(" << gx << "," << gy << ")"
                            << " now_dynamic=" << world.is_dynamic({gx, gy})
                            << " (ground truth only — robot not yet informed)\n" << std::flush;
                    }
                }
            }
        }

        // draw everything from scratch
        BeginDrawing();
        ClearBackground(theme::background);

        draw_status_line(outerMargin, outerMargin, paused, planner.suffix_mode, planner.mode,
                          placing_static, debug_overlay, step_interval, sensor_cfg.radius);
        draw_legend(outerMargin, outerMargin + 26, world);

        draw_panel(worldPanelX, panelY, panelWidth, panelHeight, headerH, "WORLD (ground truth)", theme::header_world);
        draw_panel(robotPanelX, panelY, panelWidth, panelHeight, headerH, "ROBOT BELIEF",          theme::header_robot);

        // Left pane: ground truth. Right pane: robot's belief map.
        draw_pane_occupancy(world,     worldOriginX, paneOriginY, cellSize);
        draw_pane_occupancy(robot_map, robotOriginX, paneOriginY, cellSize);

        bool needs_replan = path_is_blocked(path, robot_map);

        // Path/lasso overlay — robot pane only, since that's what the robot
        // is actually following (planned off belief, not ground truth).
        // Prefix and loop get distinct colors so the one-time run-up and the
        // repeating cycle read as different things at a glance; consecutive
        // replans still cycle through the palette (using adjacent entries
        // for prefix/cycle so replans stay distinguishable from each other
        // too), collapsing to a single alert red for both while blocked.
        Color prefix_color, cycle_color;
        if (needs_replan) {
            prefix_color = theme::path_blocked;
            cycle_color  = theme::path_blocked;
        } else if (replanned) {
            int i = (replan_count - 1) % theme::path_palette_n;
            int j = (i + 1) % theme::path_palette_n;
            prefix_color = theme::path_palette[i];
            cycle_color  = theme::path_palette[j];
        } else {
            prefix_color = theme::path_original;
            cycle_color  = theme::path_cycle;
        }
        draw_path_overlay(path, cycle_start, robotOriginX, paneOriginY, cellSize, prefix_color, cycle_color);

        // Labels are world truth, shown on both panes (robot_map was seeded
        // with the same labels in seed_from_static).
        draw_pane_labels(world,     worldOriginX, paneOriginY, cellSize);
        draw_pane_labels(robot_map, robotOriginX, paneOriginY, cellSize);

        if (debug_overlay) {
            // draw_sensor_outline(sensor_cfg, path[step_index], worldOriginX, paneOriginY, cellSize, theme::sensor_outline);   // commented out: robot/radius not wanted on the world pane (for now)
            draw_sensor_outline(sensor_cfg, path[step_index], robotOriginX, paneOriginY, cellSize, theme::sensor_outline);
        }

        // advance robot + sense — frozen while paused, except for a single
        // forced tick from KEY_RIGHT (see above, which pre-loads elapsed/
        // sense_elapsed past their thresholds so the exact same logic below
        // fires once instead of needing a separate code path).
        if (!paused || single_step_forced) {
            single_step_forced = false;

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

                log_metrics_csv(metrics, step_index, path[step_index]);
            }

            // Sense on its own timer, decoupled from movement — NOT gated on
            // `stepped`. Movement itself is gated on `!needs_replan` (see
            // above), so if sensing only ran when the robot moved, a robot
            // stalled by a blocked path could never sense again to notice
            // the block clearing later: stalled -> never steps -> never
            // senses -> stuck forever, even after you reopen the cell.
            // EveryTick means every simulation tick, independent of whether
            // the robot actually advanced a cell.
            sense_elapsed += GetFrameTime();
            bool sense_due = false;
            switch (sensor_cfg.cadence) {
                case SensingCadence::EveryTick: sense_due = sense_elapsed >= sense_interval; break;
                case SensingCadence::OnMove:    sense_due = stepped;                        break;
                case SensingCadence::OnDemand:  sense_due = false;                          break;
            }
            if (sense_due) {
                sense_elapsed = 0.0f;
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
        }

        // draw robot — robot pane only (for now); world pane intentionally
        // left without the robot/radius overlay.
        // draw_robot_marker(path[step_index], worldOriginX, paneOriginY, cellSize);
        draw_robot_marker(path[step_index], robotOriginX, paneOriginY, cellSize);

        metrics.g_current = get_g(planner.prefix.g, path_ids[step_index]);
        std::string prefix_text = format_positions(path, 0, cycle_start);
        std::string cycle_text  = format_positions(path, cycle_start, (int)path.size() - 1);
        draw_metrics_strip(metrics, 0, panelY + panelHeight + metricsGap, screenW, metricsHeight,
                            prefix_text, cycle_text, prefix_color, cycle_color);

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
