#include <vector>
#include <random>

#define CELL_SIDE      30 // pixels
#define WINDOW_MARGIN  60 // pixels

enum WallIndex {
    TOP    = 0,
    RIGHT  = 1,
    BOTTOM = 2,
    LEFT   = 3,
    NONE = 4
};

struct CellData {

    bool visited = false;
    bool walls[4] = {true,  // TOP
                     true,  // RIGHT
                     true,  // BOTTOM
                     true}; // LEFT
};

namespace Simulator {

    class Maze {

        // The wall thickness is the measurement unit
        int num_rows;
        int num_cols;

        CellData** maze = nullptr;

        void generateRandomMaze();

        void initMazeGird();

        std::vector<WallIndex> getNonVisitedNeighbors(int x, int y);

        void getNeighbor(int& x, int& y, WallIndex direction);

        void removeWall(int x, int y,  WallIndex wall_to_remove);

    public:

        Maze( int num_rows = 16, int num_cols = 16): 
            num_rows(num_rows), num_cols(num_cols) {

            generateRandomMaze();
        }

        ~Maze() {

            if (maze != nullptr) {

                for (int num_row = 0; num_row < num_rows; num_row++) {

                    delete[] maze[num_row];
                }

                delete[] maze;
            }
        }


        void drawMaze( int wait_ms = 0, int x = -1, int y = -1);
    };

};

int getRandomNumber(int min, int max) {

    std::random_device rand_dev;
    std::mt19937 generator(rand_dev());
    std::uniform_int_distribution<int> distr(min, max);

    return distr(generator);
}