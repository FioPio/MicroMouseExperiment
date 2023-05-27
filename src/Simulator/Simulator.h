#pragma once

#include <vector>
#include <opencv2/opencv.hpp>

#include "../Solver/Solver.h"


struct CellData {

    bool accessible = false;
    bool walls[4] = {true,  // TOP
                     true,  // RIGHT
                     true,  // BOTTOM
                     true}; // LEFT
};

namespace Simulator {

    class BasicMazeInfo {

        CellData maze[MAZE_HEIGHT][MAZE_WIDTH];

        std::string frame_name = "";

    public:
        BasicMazeInfo () {};

        cv::Mat drawMaze( int wait_ms = 0, int x = -1, int y = -1);

        void initMazeGird();

        void removeWall(int x, int y,  Direction wall_to_remove);

        std::vector<Direction> getNonVisitedNeighbors(int x, int y);

        void markAsAccessible(int x, int y);

        void setFrameName(const std::string& name);
        std::string getFrameName();

        int getWallDistance(int x, int y, Direction direction);

        bool hasWall(int x, int y, Direction direction);
    };

    class Maze : public BasicMazeInfo {

        void generateRandomMaze();

    public:

        Maze();
    };


    class MicroMouse : public BasicMazeInfo {

        int x = 0;
        int y = 0;
        
        Direction orientation = RIGHT;

        Maze* observable_maze;

        Solver maze_solver;

    public:

        MicroMouse(Maze* created_maze);

        void run();
        void drawMaze( int wait_ms = 0, int x = -1, int y = -1);

        void getSensorReadings();
    };

};
