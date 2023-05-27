#include <random>
#include <opencv2/opencv.hpp>

#include "Solver.h"


Solver::Solver() {
    
    // Set the map to empty
    for (int num_row = 0; num_row < MAZE_HEIGHT; num_row++) {

        for (int num_col = 0; num_col < MAZE_WIDTH; num_col++) {

            solver_map[num_row][num_col].real_cost = -1;
            solver_map[num_row][num_col].theoretical_cost = -1;

            for (int num_wall = 0; num_wall < 4; num_wall++){

                solver_map[num_row][num_col].walls[num_wall] = false;
            }

            // Define the margins
            if(num_row == 0) {

                solver_map[num_row][num_col].walls[UP] = true;
            }
            if(num_col == 0) {

                solver_map[num_row][num_col].walls[LEFT] = true;
            }
            if(num_row == MAZE_HEIGHT -1) {

                solver_map[num_row][num_col].walls[DOWN] = true;
            }
            if(num_col == MAZE_WIDTH -1) {

                solver_map[num_row][num_col].walls[RIGHT] = true;
            }
        }
    }

    solver_map[STARTING_Y][STARTING_X].theoretical_cost = 0;
}


void Solver::addMeasurement(int x, int y, Direction direction, int distance) {

    solver_map[y][x].real_cost = solver_map[y][x].theoretical_cost;
    
    for (int num_distance = 0; num_distance < distance; num_distance ++) {

        getNeighbor(x,y, direction);
    }

    addWall(x, y, direction);
}


void Solver::addWall(int x, int y,  Direction wall_to_add) {

    // Remove the wall
    solver_map[y][x].walls[wall_to_add] = true;

    // But also the same wall in the neighbor
    getNeighbor(x, y, wall_to_add);

    // It is the facing wall for the neigbor
    wall_to_add = rotateDirection(wall_to_add, 2);

    solver_map[y][x].walls[wall_to_add] = true;
}


Direction rotateDirection(Direction input_direction, int rotation) {

    return (Direction)((((int) input_direction) + rotation) % 4 );
}


int getRandomNumber(int min, int max) {

    std::random_device rand_dev;
    std::mt19937 generator(rand_dev());
    std::uniform_int_distribution<int> distr(min, max);

    return distr(generator);
}


void getNeighbor(int& x, int& y, Direction direction) {

    switch(direction) {

        case UP:
            y--;
            break;
        
        case RIGHT:
            x++;
            break;
        
        case DOWN:
            y++;
            break;
        
        case LEFT:
            x--;
            break;
    }
}



void Solver::drawMaze( int wait_ms, int x, int y) {

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

            if(solver_map[num_row][num_col].walls[UP]   &&
               solver_map[num_row][num_col].walls[RIGHT]&&
               solver_map[num_row][num_col].walls[DOWN ]&&
               solver_map[num_row][num_col].walls[LEFT]) {

                cv::Rect cell_rect(x0,
                                   y0,
                                   CELL_SIDE,
                                   CELL_SIDE);

                // Draws the cell background as black
                cv::rectangle(canvas, cell_rect, cv::Scalar(0,0,0), cv::FILLED);
            }

            // If there is a wall on top
            if (solver_map[num_row][num_col].walls[UP]) { 
                
                cv::line(canvas, 
                         cv::Point(x0, y0),
                         cv::Point(x1, y0),
                         cv::Scalar(127,127,127),
                         2);
            }

            // If there is a wall on the right
            if (solver_map[num_row][num_col].walls[RIGHT]) { 
                
                cv::line(canvas, 
                         cv::Point(x1, y0),
                         cv::Point(x1, y1),
                         cv::Scalar(127,127,127),
                         2);
            }

            // If there is a wall on the bottom
            if (solver_map[num_row][num_col].walls[DOWN]) { 
                
                cv::line(canvas, 
                         cv::Point(x0, y1),
                         cv::Point(x1, y1),
                         cv::Scalar(127,127,127),
                         2);
            }

            // If there is a wall on the left
            if (solver_map[num_row][num_col].walls[LEFT]) { 
                
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

    cv::imshow("Solver", canvas);
    cv::waitKey(wait_ms);
}


std::queue<Direction> Solver::getPath(int x, int y) {

    cleanObsoleteData();
    
    std::queue<cv::Point> queue_to_explore;

    // Adds the first point (current point)
    queue_to_explore.push(cv::Point(x, y));

    bool goal_reached = false;

    while(queue_to_explore.size() > 0 && !goal_reached) {

        // Consume this element
        int current_x = queue_to_explore.front().x;
        int current_y = queue_to_explore.front().y;
        queue_to_explore.pop();

        int current_cost = 
            solver_map[current_y][current_x].theoretical_cost;

        std::vector<Direction> possible_directions = 
            getNonVisitedNeighbors(current_x, current_y);

        if (possible_directions.size() > 0) {

            // Add them to the queue and write the score down
            for (Direction direction:possible_directions) {
                int neighbor_x = current_x;
                int neighbor_y = current_y;

                getNeighbor(neighbor_x, neighbor_y, direction);

                solver_map[neighbor_y][neighbor_x].theoretical_cost = 
                    current_cost + 1;

                queue_to_explore.push(cv::Point(neighbor_x, neighbor_y));

                if (neighbor_x >= GOAL_X_0 &&
                    neighbor_x <= GOAL_X_1 &&
                    neighbor_y >= GOAL_Y_0 &&
                    neighbor_y <= GOAL_Y_1) {

                    goal_reached = true;
                    break;
                }
            }
        }
    }

    std::queue<Direction> path;

    if (goal_reached) {

        path = getReversePath(queue_to_explore.back(), cv::Point(x,y));
    }

    return path;
}


std::vector<Direction> Solver::getNonVisitedNeighbors(int x, int y) {

    std::vector<Direction> non_visited;

    for (Direction direction:{UP,RIGHT, DOWN, LEFT}) {

        // Add non explored neighbours
        if(!solver_map[y][x].walls[direction] &&
            neighbor(x,y, direction).theoretical_cost == -1) {
                
            non_visited.push_back(direction);
        }
    }

    return non_visited;
}


std::queue<Direction> Solver::getReversePath(cv::Point starting_point, 
                                              cv::Point end_point) {

    std::vector<Direction> reverse_path;

    int current_x = starting_point.x;
    int current_y = starting_point.y;

    while(current_x != end_point.x || current_y != end_point.y) {

        Direction lower_decrease = 
            getMaxDescendantCostNeigbour(current_x, current_y);
        getNeighbor(current_x, current_y, lower_decrease);

        if (lower_decrease == NONE) {

            break;
        }
        else {

            reverse_path.push_back(lower_decrease);
        }
    }

    return reverseDirections(reverse_path);
}


std::queue<Direction> Solver::reverseDirections(std::vector<Direction> directions) {
    
    std::queue<Direction> reversed_directions;

    for (int num_direction = directions.size() -1; num_direction >= 0; num_direction--) {

        Direction input_direction = directions[num_direction];

        Direction output_direction = rotateDirection(input_direction, 2);

        reversed_directions.push(output_direction);
    }

    return reversed_directions;
}


Direction Solver::getMaxDescendantCostNeigbour(int x, int y){

    Direction lower_descend = NONE;

    int min_score = solver_map[y][x].theoretical_cost;

    for (Direction direction:{UP,RIGHT,DOWN, LEFT}) {

        if (!solver_map[y][x].walls[direction]) {

            int neighbor_cost = 
                neighbor(x, y, direction).theoretical_cost;

            if (neighbor_cost != -1 && neighbor_cost < min_score) {

                min_score = neighbor_cost;
                lower_descend = direction;
            }
        }
    }

    return lower_descend;
}


CellDataSolver Solver::neighbor(int x, int y, Direction direction) {

    getNeighbor(x, y, direction);

    return solver_map[y][x];
}


void Solver::cleanObsoleteData() {

    for (int num_row = 0; num_row < MAZE_HEIGHT; num_row++) {

        for (int num_col = 0; num_col < MAZE_WIDTH; num_col++) {

            if (solver_map[num_row][num_col].real_cost == -1) {

                solver_map[num_row][num_col].theoretical_cost = -1;
            }
        }
    }
}