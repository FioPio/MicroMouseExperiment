#include <iostream>
#include <opencv2/opencv.hpp>
#include <stack>

#include "Simulator.h"


void Simulator::Maze::initMazeGird() {

    // Allocate the maze full of walls
    maze = new CellData*[num_rows];

    for (int num_row = 0; num_row < num_rows; num_row++) {

        maze[num_row] = new CellData [num_cols];

        for (int num_col = 0; num_col < num_cols; num_col++) {
            
            maze[num_row][num_col].visited = false;
            for (int num_wall = 0; num_wall < 4; num_wall++) {

                maze[num_row][num_col].walls[num_wall] = true;
            }
        }
    }
}

std::vector<WallIndex> Simulator::Maze::getNonVisitedNeighbors(int x, int y) {

    std::vector<WallIndex> non_visited;

    // Top
    if (y > 0 && !maze[y-1][x].visited){

        non_visited.push_back(TOP);
    }

    // Right
    if (x < num_cols -1 && !maze[y][x+1].visited){

        non_visited.push_back(RIGHT);
    }

    // Bottom
    if (y < num_rows - 1 && !maze[y+1][x].visited){

        non_visited.push_back(BOTTOM);
    }

    // Left
    if (x > 0 && !maze[y][x-1].visited){

        non_visited.push_back(LEFT);
    }

    return non_visited;
}


void Simulator::Maze::getNeighbor(int& x, int& y, WallIndex direction) {

    switch(direction) {

        case TOP:
            y--;
            break;
        
        case RIGHT:
            x++;
            break;
        
        case BOTTOM:
            y++;
            break;
        
        case LEFT:
            x--;
            break;
    }
}


void Simulator::Maze::removeWall(int x, int y,  WallIndex wall_to_remove) {

    // Remove the wall
    maze[y][x].walls[wall_to_remove] = false;

    // But also the same wall in the neighbor
    getNeighbor(x, y, wall_to_remove);

    // It is the facing wall for the neigbor
    wall_to_remove = (WallIndex) ((((int) wall_to_remove) + 2) % 4);

    maze[y][x].walls[wall_to_remove] = false;
}


void Simulator::Maze::generateRandomMaze() {

    std::cout << "[INFO] Creating random maze\n";

    initMazeGird();
    
    // Using the algorithm Recursive backtracker to
    // create it

    // The initial cell is (0, 0)
    int current_x = 0;
    int current_y = 0;

    std::stack<cv::Point> alternative_cells;

    while(current_x >= 0) {

        // Set it as visited
        maze[current_y][current_x].visited = true;

        std::vector<WallIndex> possible_directions = 
                getNonVisitedNeighbors(current_x, current_y);

        if(possible_directions.size() > 0) {

            // Adds it to the stack
            alternative_cells.push(cv::Point(current_x, 
                                             current_y));

            int random_index =
            getRandomNumber(0, possible_directions.size() - 1);

            WallIndex random_direction = 
            possible_directions[random_index];
            
            removeWall(current_x, current_y, random_direction);

            getNeighbor(current_x, current_y, random_direction);

            // If reached the goal
            if((current_x == 7 || current_x == 8) &&
               (current_y == 7 || current_y == 8)) {
                
                removeWall(7,7,RIGHT);
                removeWall(8,7,BOTTOM);
                removeWall(8,8,LEFT);
                removeWall(7,8,TOP);

                maze[7][7].visited = true;
                maze[7][8].visited = true;
                maze[8][8].visited = true;
                maze[8][7].visited = true;

                if(alternative_cells.size() > 0){

                    // Take one from the stack
                    current_x = alternative_cells.top().x;
                    current_y = alternative_cells.top().y;

                    alternative_cells.pop();
                }
                else {

                    current_x = -1;
                }
            }
        }
        // If none was found but there are 
        // alternatives in the stack
        else if(alternative_cells.size() > 0){

            // If it is not next to the center, 
            // the start or the walls there
            // is a random chance to remove one wall
            int chance = getRandomNumber(0,100);
            if ((current_x > 1 && current_y > 1) && // Not in the start
                (current_x < num_cols -1 && 
                 current_y < num_rows -1) && // Or a corner
                ((current_x < 6 || current_x > 9) ||
                (current_y < 6 || current_y > 9)) &&
                chance < 15) {

                // Remove random wall
                WallIndex random_direction = (WallIndex)
                getRandomNumber(0, 3);

                removeWall(current_x, current_y, random_direction);
            }
            else {
                // Take one from the stack
                current_x = alternative_cells.top().x;
                current_y = alternative_cells.top().y;

            }

            alternative_cells.pop();
        }
        else {

            current_x = -1;
        }

        drawMaze(150, current_x, current_y);
    }
}


void Simulator::Maze::drawMaze( int wait_ms, int x, int y) {

    int maze_width = num_cols * CELL_SIDE; // in pixels
    int image_width = (/*margins*/ 2 * WINDOW_MARGIN) + 
                      maze_width;

    int maze_height = num_rows * CELL_SIDE; // in pixels
    int image_height = (/*margins*/ 2 * WINDOW_MARGIN) + 
                      maze_height;
    // White canvas
    cv::Mat canvas(image_width, image_height, CV_8UC3, cv::Scalar(255, 255, 255));

    for (int num_row = 0; num_row < num_rows; num_row++) {

        for (int num_col = 0; num_col < num_cols; num_col++) {
            
            int x0 = WINDOW_MARGIN + (num_col * CELL_SIDE);
            int x1 = WINDOW_MARGIN + ((num_col + 1) * CELL_SIDE);

            int y0 = WINDOW_MARGIN + (num_row * CELL_SIDE);
            int y1 = WINDOW_MARGIN + ((num_row + 1) * CELL_SIDE);

            if(!maze[num_row][num_col].visited) {

                cv::Rect cell_rect(x0,
                                y0,
                                CELL_SIDE,
                                CELL_SIDE);

                // Draws the cell background as black
                cv::rectangle(canvas, cell_rect, cv::Scalar(0,0,0), cv::FILLED);
            }

            // If there is a wall on top
            if (maze[num_row][num_col].walls[TOP]) { 
                
                cv::line(canvas, 
                         cv::Point(x0, y0),
                         cv::Point(x1, y0),
                         cv::Scalar(127,127,127),
                         2);
            }

            // If there is a wall on the right
            if (maze[num_row][num_col].walls[RIGHT]) { 
                
                cv::line(canvas, 
                         cv::Point(x1, y0),
                         cv::Point(x1, y1),
                         cv::Scalar(127,127,127),
                         2);
            }

            // If there is a wall on the bottom
            if (maze[num_row][num_col].walls[BOTTOM]) { 
                
                cv::line(canvas, 
                         cv::Point(x0, y1),
                         cv::Point(x1, y1),
                         cv::Scalar(127,127,127),
                         2);
            }

            // If there is a wall on the left
            if (maze[num_row][num_col].walls[LEFT]) { 
                
                cv::line(canvas, 
                         cv::Point(x0, y0),
                         cv::Point(x0, y1),
                         cv::Scalar(127,127,127),
                         2);
            }
        }
    }

    // Paint start and goal
    cv::Rect start_cell(WINDOW_MARGIN + 1,
                        WINDOW_MARGIN + 1,
                        CELL_SIDE - 2,
                        CELL_SIDE - 2);

    cv::Rect goal_area(WINDOW_MARGIN + 1 + (7 * CELL_SIDE),
                       WINDOW_MARGIN + 1 + (7 * CELL_SIDE),
                       (2 * CELL_SIDE) - 2,
                       (2 * CELL_SIDE) - 2);

    // Draws the cell background as black
    cv::rectangle(canvas, start_cell, cv::Scalar(0,0,255), cv::FILLED);

    cv::rectangle(canvas, goal_area, cv::Scalar(0,255,0), cv::FILLED);

    // Draw current cell
    if (x != -1  && y != -1) {

        cv::Rect current_cell(WINDOW_MARGIN + 1 + (CELL_SIDE * x),
                              WINDOW_MARGIN + 1 + (CELL_SIDE * y),
                              CELL_SIDE - 2,
                              CELL_SIDE - 2);


        // Draws the cell background as black
        cv::rectangle(canvas, current_cell, cv::Scalar(5,94,255), cv::FILLED);
    }

    cv::imshow("Maze", canvas);
    cv::waitKey(wait_ms);
}