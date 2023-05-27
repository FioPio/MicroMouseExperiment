#pragma once

#include <vector>

#include "Defines.h"

// Implementation of the Floodfill method

struct CellDataSolver {

    int theoretical_cost = -1;
    int real_cost = -1;

    bool walls[4] = {false,  // TOP
                     false,  // RIGHT
                     false,  // BOTTOM
                     false}; // LEFT
};

struct Point {

    int x;
    int y;
};

class Solver {

    CellDataSolver solver_map[MAZE_HEIGHT][MAZE_WIDTH];

    void addWall(int x, int y,  Direction wall_to_add);

    std::vector<Direction> getNonVisitedNeighbors(int x, int y);

    CellDataSolver neighbor(int x, int y, Direction direction);

    void cleanObsoleteData();

    std::queue<Direction> getReversePath(cv::Point starting_point, 
                                          cv::Point end_point);

    std::queue<Direction> reverseDirections(std::vector<Direction> directions);                                     

    Direction getMaxDescendantCostNeigbour(int x, int y);

public:

    Solver();

    void addMeasurement(int x, int y, Direction direction, int distance);

    void drawMaze( int wait_ms = 0, int x = -1, int y = -1, std::queue<Direction> path = {});

    std::queue<Direction> getPath(int x, int y);
    
};



Direction rotateDirection(Direction input_direction, int rotation);
void getNeighbor(int& x, int& y, Direction direction);
int getRandomNumber(int min, int max);