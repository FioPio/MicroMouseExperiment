#pragma once

#include <vector>

#include "Defines.h"

// Implementation of the Floodfill method

struct CellDataSolver {

    int cost = -1;

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

    std::vector<Direction> getAccessibleNeighbors(int x, int y);

    CellDataSolver neighbor(int x, int y, Direction direction);

    std::queue<Direction> getReversePath(cv::Point starting_point, 
                                          cv::Point end_point);

    std::queue<Direction> getDirectPath(cv::Point starting_point);                                  

    std::queue<Direction> reverseDirections(std::vector<Direction> directions);                                     

    Direction getMaxDescendantCostNeigbour(int x, int y);

    int x_goal = -1;
    int y_goal = -1;

    void generateFirstApproachCosts();

    Direction exploreNode(int x_node, 
                          int y_node, 
                          std::queue<cv::Point>& queue_to_explore);

public:

    Solver();

    void addMeasurement(int x, int y, Direction direction, int distance);

    void drawMaze( int wait_ms = 0, int x = -1, int y = -1, std::queue<Direction> path = {});

    std::queue<Direction> getPath(int x, int y);

    void setGoal(int x, int y);
    
};



Direction rotateDirection(Direction input_direction, int rotation);
void getNeighbor(int& x, int& y, Direction direction);
int getRandomNumber(int min, int max);