#include <random>
#include <opencv2/opencv.hpp>

#include "Solver.h"


Solver::Solver() {
    
    // Set the map to empty
    for (int num_row = 0; num_row < MAZE_HEIGHT; num_row++) {

        for (int num_col = 0; num_col < MAZE_WIDTH; num_col++) {

            solver_map[num_row][num_col].cost = -1;

            for (int wall:ALL_DIRECTIONS){

                solver_map[num_row][num_col].walls[wall] = false;
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
}


void Solver::addMeasurement(int x, int y, Direction direction, int distance) {
 
    int x_to_set_wall = x;
    int y_to_set_wall = y;

    for (int num_distance = 0; num_distance < distance; num_distance ++) {

        getNeighbor(x_to_set_wall,
                    y_to_set_wall, 
                    direction);
    }

    addWall(x_to_set_wall,
            y_to_set_wall, 
            direction);
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


std::vector<Direction> Solver::getAccessibleNeighbors(int x, int y) {

    std::vector<Direction> accessible_neighbours;

    for (Direction direction:ALL_DIRECTIONS) {

        if (!solver_map[y][x].walls[direction]) {

            accessible_neighbours.push_back(direction);
        }
    }

    return accessible_neighbours;
}


Direction rotateDirection(Direction input_direction, int rotation) {

    int rotated_direction = ((int) input_direction) + rotation;

    while (rotated_direction < 0 ) {

        rotated_direction += 4;
    }

    return (Direction)( rotated_direction % 4 );
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


void Solver::drawMaze( int wait_ms, int x, int y, std::queue<Direction> path) {

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

    int path_x = x;
    int path_y = y;

    while(path.size() > 0) {
        
        Direction next_step = path.front();
        path.pop();

        getNeighbor(path_x, path_y, next_step);

        cv::Rect current_cell(WINDOW_MARGIN + 1 + (CELL_SIDE * path_x),
                              WINDOW_MARGIN + 1 + (CELL_SIDE * path_y),
                              CELL_SIDE - 2,
                              CELL_SIDE - 2);


        // Draws the cell background as blue
        cv::rectangle(canvas, current_cell, cv::Scalar(255, 255, 0), cv::FILLED);
    }

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


void Solver::setGoal(int x, int y) {

    x_goal = x;
    y_goal = y;

    generateFirstApproachCosts();
}


void Solver::generateFirstApproachCosts() {

    if (x_goal >= 0 && x_goal < MAZE_WIDTH &&
        y_goal >= 0 && y_goal < MAZE_HEIGHT) {

        for (int x_cell = 0; x_cell < MAZE_WIDTH; x_cell++) {

            for (int y_cell = 0; y_cell < MAZE_HEIGHT; y_cell++) {

                int x_difference = abs(x_goal - x_cell);
                int y_difference = abs(y_goal - y_cell);

                int approximate_cost = x_difference + y_difference;

                solver_map[y_cell][x_cell].cost = approximate_cost;
            } 
        } 
    }
}


std::queue<Direction> Solver::getPath(int x_start, int y_start) {

    //generateFirstApproachCosts();
    
    std::queue<cv::Point> queue_to_explore;

    bool goal_reached = false;

    // Consume this element
    int current_x = x_start;
    int current_y = y_start;

    while(!goal_reached) {

        Direction decreasing_direction = 
            exploreNode(current_x, current_y, queue_to_explore); 

        if (decreasing_direction != NONE) {

            getNeighbor(current_x, current_y, decreasing_direction);
        }

        while(queue_to_explore.size() > 0)  {

            int queue_x = queue_to_explore.front().x;
            int queue_y = queue_to_explore.front().y;
            queue_to_explore.pop();

            exploreNode(queue_x, queue_y, queue_to_explore);
        }

        if ( current_x == x_goal && current_y == y_goal) {

            goal_reached = true;
        }
    }

    std::queue<Direction> path;

    if (goal_reached) {

        path = getDirectPath(cv::Point(x_start, y_start));
    }

    return path;
}



Direction Solver::exploreNode(int x_node, 
                              int y_node, 
                              std::queue<cv::Point>& queue_to_explore) {

    Direction decreasing_direction = NONE;

    int node_cost = 
                solver_map[y_node][x_node].cost;

    std::vector<Direction> possible_neighbors = 
        getAccessibleNeighbors(x_node, y_node);
    
    Direction minim_neighbor_direction = 
        getMaxDescendantCostNeigbour(x_node, y_node);
    
    int minimal_neighbor_cost = 
            neighbor(x_node, 
                     y_node,
                     minim_neighbor_direction).cost;

    
    if (node_cost < minimal_neighbor_cost) {

        solver_map[y_node][x_node].cost = 
            minimal_neighbor_cost+ 1;

        for(Direction possible_neighbor:possible_neighbors) {

            int neighbor_x = x_node;
            int neighbor_y = y_node;
            
            getNeighbor(neighbor_x, neighbor_y, possible_neighbor);

            queue_to_explore.push(cv::Point(neighbor_x, neighbor_y));
        }
    }
    else {

        decreasing_direction = minim_neighbor_direction;
    }

    return decreasing_direction;
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


std::queue<Direction> Solver::getDirectPath(cv::Point starting_point) {

    std::queue<Direction> path;

    int current_x = starting_point.x;
    int current_y = starting_point.y;

    while(current_x != x_goal || current_y != y_goal) {

        Direction lower_decrease = 
            getMaxDescendantCostNeigbour(current_x, current_y);
        getNeighbor(current_x, current_y, lower_decrease);

        if (lower_decrease == NONE) {

            break;
        }
        else {

            path.push(lower_decrease);
        }
    }

    return path;
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

    // Big number
    int min_score = 10000;

    for (Direction direction:{UP,RIGHT,DOWN, LEFT}) {

        if (!solver_map[y][x].walls[direction]) {

            int neighbor_cost = 
                neighbor(x, y, direction).cost;

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
