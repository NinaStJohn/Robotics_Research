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


// private functions

// helper
static int idx(int x, int y, int w) { return y * w + x; }

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
    for (const auto& seg : segments) {
        out.insert(out.end(), seg.begin(), seg.end());
    }
    return out;
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

void vis_multi_run();


