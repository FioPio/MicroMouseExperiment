#include <iostream>
#include <stack>

#include "Simulator.h"


void Simulator::BasicMazeInfo::initMazeGird() {

    for (int num_row = 0; num_row < MAZE_HEIGHT; num_row++) {

        for (int num_col = 0; num_col < MAZE_WIDTH; num_col++) {
            
            maze[num_row][num_col].accessible = false;
            for (int num_wall = 0; num_wall < 4; num_wall++) {

                maze[num_row][num_col].walls[num_wall] = true;
            }
        }
    }
}


bool Simulator::BasicMazeInfo::hasWall(int x, int y, Direction direction) {

    return maze[y][x].walls[direction];
}


std::vector<Direction> Simulator::BasicMazeInfo::getNonVisitedNeighbors(int x, int y) {

    std::vector<Direction> non_visited;

    // Top
    if (y > 0 && !maze[y-1][x].accessible){

        non_visited.push_back(UP);
    }

    // Right
    if (x < MAZE_WIDTH -1 && !maze[y][x+1].accessible){

        non_visited.push_back(RIGHT);
    }

    // Bottom
    if (y < MAZE_HEIGHT - 1 && !maze[y+1][x].accessible){

        non_visited.push_back(DOWN);
    }

    // Left
    if (x > 0 && !maze[y][x-1].accessible){

        non_visited.push_back(LEFT);
    }

    return non_visited;
}


void Simulator::BasicMazeInfo::removeWall(int x, int y,  Direction wall_to_remove) {

    // Remove the wall
    maze[y][x].walls[wall_to_remove] = false;

    // But also the same wall in the neighbor
    getNeighbor(x, y, wall_to_remove);

    // It is the facing wall for the neighbor
    Direction neighbor_wall_to_remove = rotateDirection(wall_to_remove, 2);

    maze[y][x].walls[neighbor_wall_to_remove] = false;
}


void Simulator::BasicMazeInfo::markAsAccessible(int x, int y) {

    maze[y][x].accessible = true;
}


void Simulator::BasicMazeInfo::setFrameName(const std::string& name) {

    frame_name = name;
}


std::string Simulator::BasicMazeInfo::getFrameName() {

    return frame_name;
}

Simulator::Maze::Maze() {

    generateRandomMaze();

    setFrameName("Original Maze");
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
        markAsAccessible(current_x, current_y);

        std::vector<Direction> possible_directions = 
                getNonVisitedNeighbors(current_x, current_y);

        if(possible_directions.size() > 0) {

            // Adds it to the stack
            alternative_cells.push(cv::Point(current_x, 
                                             current_y));

            int random_index =
            getRandomNumber(0, possible_directions.size() - 1);

            Direction random_direction = 
            possible_directions[random_index];
            
            removeWall(current_x, current_y, random_direction);

            getNeighbor(current_x, current_y, random_direction);

            // If reached the goal
            if((current_x == GOAL_X_0 || current_x == GOAL_X_1) &&
               (current_y == GOAL_Y_0 || current_y == GOAL_Y_1)) {
                
                removeWall(GOAL_X_0,GOAL_Y_0, RIGHT);
                removeWall(GOAL_X_1,GOAL_Y_0, DOWN);
                removeWall(GOAL_X_1,GOAL_Y_1, LEFT);
                removeWall(GOAL_X_0,GOAL_Y_1, UP);

                markAsAccessible(GOAL_X_0,GOAL_Y_0);
                markAsAccessible(GOAL_X_1,GOAL_Y_0);
                markAsAccessible(GOAL_X_1,GOAL_Y_1);
                markAsAccessible(GOAL_X_0,GOAL_Y_1);

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
                (current_x < MAZE_WIDTH -1 && 
                 current_y < MAZE_HEIGHT -1) && // Or a corner
                ((current_x < GOAL_X_0 - 1 || current_x > GOAL_X_1 + 1 ) ||
                 (current_y < GOAL_Y_0 - 1 || current_y > GOAL_Y_1 + 1)) &&
                chance < CHANCE_OF_OPEN_WALL) {

                // Remove random wall
                Direction random_direction = (Direction)
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
        #ifdef SHOW_MAZE_CREATION
            drawMaze(150, current_x, current_y);
        #endif
    }
}


cv::Mat Simulator::BasicMazeInfo::drawMaze( int wait_ms, int x, int y) {

    int maze_width = MAZE_WIDTH * CELL_SIDE; // in pixels
    int image_width = (/*margins*/ 2 * WINDOW_MARGIN) + 
                      maze_width;

    int maze_height = MAZE_HEIGHT * CELL_SIDE; // in pixels
    int image_height = (/*margins*/ 2 * WINDOW_MARGIN) + 
                      maze_height;
    // White canvas
    cv::Mat canvas(image_width, image_height, CV_8UC3, cv::Scalar(255, 255, 255));

    for (int num_row = 0; num_row < MAZE_HEIGHT; num_row++) {

        for (int num_col = 0; num_col < MAZE_WIDTH; num_col++) {
            
            int x0 = WINDOW_MARGIN + (num_col * CELL_SIDE);
            int x1 = WINDOW_MARGIN + ((num_col + 1) * CELL_SIDE);

            int y0 = WINDOW_MARGIN + (num_row * CELL_SIDE);
            int y1 = WINDOW_MARGIN + ((num_row + 1) * CELL_SIDE);

            if(!maze[num_row][num_col].accessible) {

                cv::Rect cell_rect(x0,
                                y0,
                                CELL_SIDE,
                                CELL_SIDE);

                // Draws the cell background as black
                cv::rectangle(canvas, cell_rect, cv::Scalar(0,0,0), cv::FILLED);
            }

            // If there is a wall on top
            if (maze[num_row][num_col].walls[UP]) { 
                
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
            if (maze[num_row][num_col].walls[DOWN]) { 
                
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

    cv::Rect goal_area(WINDOW_MARGIN + 1 + (GOAL_X_0 * CELL_SIDE),
                       WINDOW_MARGIN + 1 + (GOAL_Y_0 * CELL_SIDE),
                       (2 * CELL_SIDE) - 2,
                       (2 * CELL_SIDE) - 2);

    // Paints the starting cell background as red
    cv::rectangle(canvas, start_cell, cv::Scalar(0,0,255), cv::FILLED);

    // Paints the goal area background as green
    cv::rectangle(canvas, goal_area, cv::Scalar(0,255,0), cv::FILLED);

    // Draw current cell
    if (x != -1  && y != -1) {

        cv::Rect current_cell(WINDOW_MARGIN + 1 + (CELL_SIDE * x),
                              WINDOW_MARGIN + 1 + (CELL_SIDE * y),
                              CELL_SIDE - 2,
                              CELL_SIDE - 2);


        // Draws the cell background as orangeish
        cv::rectangle(canvas, current_cell, cv::Scalar(5,94,255), cv::FILLED);
    }

    cv::imshow(frame_name, canvas);
    cv::waitKey(wait_ms);

    return canvas;
}

int Simulator::BasicMazeInfo::getWallDistance(int x, int y, Direction direction) {

    int distance = 0;

    while (!maze[y][x].walls[direction]) {

        distance++;
        getNeighbor(x, y, direction);
    }

    return distance;
}


// MicroMouse data
Simulator::MicroMouse::MicroMouse(Maze* created_maze) {

    observable_maze = created_maze;
    
    // setUp start position
    initMazeGird();

    // Set it as visited
    markAsAccessible(0,0);

    setFrameName("Mouse");
}


void Simulator::MicroMouse::drawMaze( int wait_ms, int x, int y) {

    cv::Mat canvas = BasicMazeInfo::drawMaze(1);

    cv::Rect current_cell(WINDOW_MARGIN + 8 + (CELL_SIDE * x),
                          WINDOW_MARGIN + 8 + (CELL_SIDE * y),
                          CELL_SIDE - 16,
                          CELL_SIDE - 16);


    // Draws the cell background as orange
    cv::rectangle(canvas, current_cell, cv::Scalar(255,94,0), cv::FILLED);

    int x_head = current_cell.x + (current_cell.width / 2);
    int y_head = current_cell.y + (current_cell.height / 2);

    switch (orientation)
    {
        case UP:
            y_head -= 7;
            break;

        case RIGHT:
            x_head += 7;
            break;

        case DOWN:
            y_head += 7;
            break;

        case LEFT:
            x_head -= 7;
            break;
    }

    cv::Point robot_head(x_head, y_head);
    cv::circle(canvas, robot_head, 5, cv::Scalar(127, 0, 127), cv::FILLED);

    cv::imshow(getFrameName(), canvas);
    cv::waitKey(wait_ms);
}


void Simulator::MicroMouse::getSensorReadings() {

    // Gets the free distance for the left, 
    // front and right sensors keeping in mind
    // the robot orientation
    for (int rotation:{ /*left*/-1, 
                        /*frontal*/0, 
                        /*right*/1}) {

        Direction rotated_direction = 
            rotateDirection(orientation, rotation);

        // Get distances
        int sensor_distance = 
            observable_maze->getWallDistance(x, 
                                             y, 
                                             rotated_direction);
        // Read and update the world map
        // Update the map
        int updater_x = x;
        int updater_y = y;

        // Add the information to the solver!!
        maze_solver.addMeasurement(updater_x,
                                   updater_y,
                                   rotated_direction,
                                   sensor_distance);

        // Add the data to the image
        for (int num_cell = 0; num_cell < sensor_distance; num_cell++) {

            if (updater_x > -1 && updater_y > -1) {

                removeWall(updater_x, updater_y, rotated_direction);
                getNeighbor(updater_x, updater_y, rotated_direction);

                markAsAccessible(updater_x, updater_y);
            }
        }
    }
}


void Simulator::MicroMouse::run() {

    getSensorReadings();
    drawMaze(1,x,y);
    maze_solver.drawMaze(0);

    std::queue<Direction> path = maze_solver.getPath(x, y);
    
    //for (int num_step = 0; num_step < 50; num_step++) {
    while(path.size() > 0) {

        drawMaze(1,x,y);
        getSensorReadings();
        drawMaze(1,x,y);
        maze_solver.drawMaze(10, x, y, path);

         if(path.size() > 0) {

            Direction next_step = path.front();
            path.pop();

            drawMaze(150,x,y);

            if (hasWall(x, y, next_step)) {

                std::queue<Direction> new_path = maze_solver.getPath(x, y);

                std::swap( path, new_path );
            }
            else {

                orientation = next_step;
                getNeighbor(x, y, next_step);
            }
        }
    }

    maze_solver.drawMaze(10, x, y);
    drawMaze(0,x,y);

}