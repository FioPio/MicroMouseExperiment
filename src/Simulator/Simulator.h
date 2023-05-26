#include <vector>
#include <random>
#include <opencv2/opencv.hpp>

#define CELL_SIDE            30 // pixels
#define WINDOW_MARGIN        60 // pixels
#define CHANCE_OF_OPEN_WALL  20 // pixels


// Visualization options
//#define SHOW_MAZE_CREATION

enum Direction {
    UP    = 0,
    RIGHT = 1,
    DOWN  = 2,
    LEFT  = 3,
    NONE  = 4
};

struct SensorReadings {

    int front;
    int left;
    int right;
};

struct CellData {

    bool accessible = false;
    bool walls[4] = {true,  // TOP
                     true,  // RIGHT
                     true,  // BOTTOM
                     true}; // LEFT
};

namespace Simulator {

    class BasicMazeInfo {

        int num_rows;
        int num_cols;

        CellData** maze = nullptr;

        std::string frame_name = "";

    public:
        BasicMazeInfo () {};

        ~BasicMazeInfo() {

            if (maze != nullptr) {

                for (int num_row = 0; num_row < num_rows; num_row++) {

                    delete[] maze[num_row];
                }

                delete[] maze;
            }
        }

        cv::Mat drawMaze( int wait_ms = 0, int x = -1, int y = -1);

        void initMazeGird();

        void removeWall(int x, int y,  Direction wall_to_remove);

        std::vector<Direction> getNonVisitedNeighbors(int x, int y);

        void getNeighbor(int& x, int& y, Direction direction);

        int getMazeWidth();
        int getMazeHeight();

        void setMazeWidth(int width);
        void setMazeHeight(int height);

        void markAsAccessible(int x, int y);

        void setFrameName(const std::string& name);
        std::string getFrameName();

        int getWallDistance(int x, int y, Direction direction);
    };

    class Maze : public BasicMazeInfo {

        void generateRandomMaze();

    public:

        Maze( int _num_rows = 16, int _num_cols = 16);
    };


    class MicroMouse : public BasicMazeInfo {

        int x = 0;
        int y = 0;
        
        Direction orientation = RIGHT;

        Maze* observable_maze;

    public:

        MicroMouse(Maze* created_maze);
        void run();
        void drawMaze( int wait_ms = 0, int x = -1, int y = -1);

        void getSensorReadings();
    };

};

int getRandomNumber(int min, int max) {

    std::random_device rand_dev;
    std::mt19937 generator(rand_dev());
    std::uniform_int_distribution<int> distr(min, max);

    return distr(generator);
}

Direction rotateDirection(Direction input_direction, int rotation) {

    return (Direction)((((int) input_direction) + rotation) % 4 );
}