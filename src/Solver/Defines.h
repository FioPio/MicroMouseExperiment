#pragma once

#define MAZE_WIDTH  16
#define MAZE_HEIGHT 16

#define GOAL_X_0    ((MAZE_WIDTH / 2) - 1)
#define GOAL_X_1     (MAZE_WIDTH / 2)
#define GOAL_Y_0    ((MAZE_HEIGHT / 2) - 1)
#define GOAL_Y_1     (MAZE_HEIGHT / 2)

#define STARTING_X 0
#define STARTING_Y 0


/// DIRECTIONS
enum Direction {
    UP    = 0,
    RIGHT = 1,
    DOWN  = 2,
    LEFT  = 3,
    NONE  = 4
};

#define ALL_DIRECTIONS {UP,RIGHT,DOWN, LEFT}
/// Visualization parameters
#define CELL_SIDE            30 // pixels
#define WINDOW_MARGIN        60 // pixels
#define CHANCE_OF_OPEN_WALL  20 // pixels


// Visualization options
//#define SHOW_MAZE_CREATION