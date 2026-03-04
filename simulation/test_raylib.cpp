#include "raylib.h"
#include <vector>

// Simple position struct for testing
struct Pos {
    int x;
    int y;
};

int main() {
    const int gridWidth = 5;
    const int gridHeight = 5;
    const int cellSize = 100;

    const int screenWidth  = gridWidth  * cellSize;
    const int screenHeight = gridHeight * cellSize;

    InitWindow(screenWidth, screenHeight, "Raylib Grid Test");
    SetTargetFPS(60);

    // Fake path for testing
    std::vector<Pos> path = {
        {0,0}, {1,0}, {2,0}, {2,1}, {2,2}, {3,2}, {4,2}
    };

    while (!WindowShouldClose()) {

        BeginDrawing();
        ClearBackground(RAYWHITE);

        // Draw grid
        for (int y = 0; y < gridHeight; ++y) {
            for (int x = 0; x < gridWidth; ++x) {
                DrawRectangleLines(
                    x * cellSize,
                    y * cellSize,
                    cellSize,
                    cellSize,
                    LIGHTGRAY
                );
            }
        }

        // Draw path
        for (const auto& p : path) {
            DrawRectangle(
                p.x * cellSize,
                p.y * cellSize,
                cellSize,
                cellSize,
                Color{0, 121, 241, 150}  // semi-transparent blue
            );
        }

        EndDrawing();
    }

    CloseWindow();
    return 0;
}

/*
g++ test_raylib.cpp $(pkg-config --cflags --libs raylib) -o test
./test
*/