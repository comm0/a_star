#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>

// Define a structure to represent a cell in the grid
struct cell_t {
    int x, y;        // Coordinates of the cell
    double f, g, h;  // f = g + h for A* algorithm

    cell_t(int x, int y) : x(x), y(y), f(0), g(0), h(0) {}

    // Function to calculate the heuristic value (Manhattan distance) to the goal
    double heuristic(const cell_t& goal) const {
        return std::abs(x - goal.x) + std::abs(y - goal.y);
    }
};

// Custom comparator for priority queue based on f value
struct compare_cells {
    bool operator()(const cell_t& a, const cell_t& b) {
        return a.f > b.f;
    }
};

// A* search function
std::vector<cell_t> a_star(const std::vector<std::vector<int>>& grid, cell_t start, cell_t goal) {
    int rows = grid.size();
    int cols = grid[0].size();

    std::vector<std::vector<bool>> visited(rows, std::vector<bool>(cols, false));
    std::vector<std::vector<cell_t>> came_from(rows, std::vector<cell_t>(cols, cell_t(-1, -1)));
    std::priority_queue<cell_t, std::vector<cell_t>, compare_cells> open_list;

    open_list.push(start);

    while (!open_list.empty()) {
        cell_t current = open_list.top();
        open_list.pop();

        if (current.x == goal.x && current.y == goal.y) {
            // Goal reached, construct and return the path
            std::vector<cell_t> path;
            while (current.x != -1 && current.y != -1) {
                path.push_back(current);
                current = came_from[current.x][current.y];
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        visited[current.x][current.y] = true;

        // Define possible moves (right, left, down, up)
        int dx[4] = {1, -1, 0, 0};
        int dy[4] = {0, 0, 1, -1};

        for (int i = 0; i < 4; ++i) {
            int new_x = current.x + dx[i];
            int new_y = current.y + dy[i];

            // Check if the new coordinates are valid and not visited or blocked
            if (new_x >= 0 && new_x < rows && new_y >= 0 && new_y < cols && !visited[new_x][new_y] && grid[new_x][new_y] == 0) {
                cell_t neighbor(new_x, new_y);
                double tentative_g = current.g + 1.0; // Assuming the cost of movement is 1

                if (tentative_g < neighbor.g || !visited[new_x][new_y]) {
                    neighbor.g = tentative_g;
                    neighbor.h = neighbor.heuristic(goal);
                    neighbor.f = neighbor.g + neighbor.h;
                    came_from[new_x][new_y] = current;
                    open_list.push(neighbor);
                }
            }
        }
    }

    // If the search fails to find a path, return an empty vector
    return std::vector<cell_t>();
}

int max_x = 0;
int max_y = 0; // todo move it somewhere else someday
std::vector<cell_t> convert_path_to_coordinates(const std::string& path) {
    std::vector<cell_t> coordinates;
    int x = 0, y = 0;
    int min_x = 0, min_y = 0;

    // Initial processing to determine minimums
    for (char dir : path) {
        switch(dir) {
            case 'N': y += 1; break;
            case 'E': x += 1; break;
            case 'S': y -= 1; break;
            case 'W': x -= 1; break;
            default: break; // Handle unexpected characters by ignoring them
        }

        // Update minimums
        if (x < min_x) min_x = x;
        if (y < min_y) min_y = y;
    }

    // Apply one extra column and row for border
    min_x -= 1; 
    min_y -= 1;



    // Reset for the actual coordinate calculation
    x = 0, y = 0;
    coordinates.push_back({x - min_x, y - min_y});  // Apply offset

    for (char dir : path) {
        switch(dir) {
            case 'N': y += 1; break;
            case 'E': x += 1; break;
            case 'S': y -= 1; break;
            case 'W': x -= 1; break;
            default: break; // Handle unexpected characters by ignoring them
        }
        auto x_val = x - min_x; // Apply offsets
        auto y_val = y - min_y;
        coordinates.push_back({x_val, y_val}); 

        if(x_val > max_x)
        {
            max_x = x_val;
        }
        if(y_val > max_y)
        {
            max_y = y_val;
        }
    }

    return coordinates;
}

bool contains(const std::vector<cell_t>& obstacles, int x, int y) {
    return std::find_if(obstacles.begin(), obstacles.end(), [x, y](const cell_t& cell) {
        return cell.x == x && cell.y == y;
    }) != obstacles.end();
}

// A* search function
std::vector<cell_t> a_star(cell_t start, cell_t goal, const std::vector<cell_t>& obstacles, int rows, int cols) {
    std::vector<std::vector<bool>> visited(rows, std::vector<bool>(cols, false));
    std::vector<std::vector<cell_t>> came_from(rows, std::vector<cell_t>(cols, cell_t(-1, -1)));
    std::priority_queue<cell_t, std::vector<cell_t>, compare_cells> open_list;

    open_list.push(start);

    while (!open_list.empty()) {
        cell_t current = open_list.top();
        open_list.pop();

        if (current.x == goal.x && current.y == goal.y) {
            // Goal reached, construct and return the path
            std::vector<cell_t> path;
            while (current.x != -1 && current.y != -1) {
                path.push_back(current);
                current = came_from[current.x][current.y];
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        visited[current.x][current.y] = true;

        // Define possible moves (right, left, down, up)
        int dx[4] = {1, -1, 0, 0};
        int dy[4] = {0, 0, 1, -1};

        for (int i = 0; i < 4; ++i) {
            int new_x = current.x + dx[i];
            int new_y = current.y + dy[i];

            // Check if the new coordinates are valid and not visited or blocked
            if (new_x >= 0 && new_x < rows && new_y >= 0 && new_y < cols && !visited[new_x][new_y] && !contains(obstacles, new_x, new_y)) {
                cell_t neighbor(new_x, new_y);
                double tentative_g = current.g + 1.0; // Assuming the cost of movement is 1

                if (tentative_g < neighbor.g || !visited[new_x][new_y]) {
                    neighbor.g = tentative_g;
                    neighbor.h = neighbor.heuristic(goal);
                    neighbor.f = neighbor.g + neighbor.h;
                    came_from[new_x][new_y] = current;
                    open_list.push(neighbor);
                }
            }
        }
    }

    // If the search fails to find a path, return an empty vector
    return std::vector<cell_t>();
}

std::string convert_path_to_coordinates(const std::vector<cell_t>& path) {
    std::string directions;

    for (size_t i = 1; i < path.size(); ++i) {
        const cell_t& curr = path[i];
        const cell_t& prev = path[i - 1];

        // Determine the direction based on the change in x and y coordinates
        if (curr.x > prev.x) {
            directions += 'E';
        } else if (curr.x < prev.x) {
            directions += 'W';
        } else if (curr.y > prev.y) {
            directions += 'N';
        } else if (curr.y < prev.y) {
            directions += 'S';
        }
    }

    return directions;
}

std::string convert_coordinates_to_path(const std::vector<cell_t>& path) {
    std::string directions;

    for (size_t i = 1; i < path.size(); ++i) {
        const cell_t& curr = path[i];
        const cell_t& prev = path[i - 1];

        // Determine the direction based on the change in x and y coordinates
        if (curr.x > prev.x) {
            directions += 'E';
        } else if (curr.x < prev.x) {
            directions += 'W';
        } else if (curr.y > prev.y) {
            directions += 'N';
        } else if (curr.y < prev.y) {
            directions += 'S';
        }
    }

    return directions;
}

int main() {
    std::string path_start = "NWWWWNEEENN";
    std::cout << "Init path: " << path_start << "\n\n";
    auto obstacles = convert_path_to_coordinates(path_start);
    for (const auto& cell : obstacles)
        {
            std::cout << "(" << cell.x << ", " << cell.y << ")" << "\n";
        }
    std::cout << "max x:" << max_x << ", max y: " << max_y << "\n";
    
    std::vector<std::vector<int>> grid = {
        {0, 1, 0, 0, 0},
        {0, 0, 0, 1, 0},
        {0, 1, 0, 1, 0},
        {0, 0, 0, 0, 0},
        {0, 0, 0, 1, 0}
    };

    cell_t start(0, 0);
    cell_t goal(4, 4);

    //std::vector<cell_t> path = a_star(grid, start, goal);
    if (obstacles.size() <= 2)
    {
        std::cout << "Not enough nodes\n";
        return 0;
    }
    cell_t start_2 = obstacles.back();
    obstacles.pop_back();
    cell_t goal_2 = obstacles.front();
    // Remove the first element by creating a new vector without the first element
    obstacles = std::vector<cell_t>(obstacles.begin() + 1, obstacles.end()); // do not remove?

    std::vector<cell_t> path = a_star(start_2, goal_2, obstacles, max_x + 2, max_y + 2);
    
    if (!path.empty()) {
        std::cout << "Path found:" << "\n";
        for (const auto& cell_t : path) {
            std::cout << "(" << cell_t.x << ", " << cell_t.y << ")" << "\n";
        }
        std::cout << "End path: " << convert_coordinates_to_path(path) << "\n\n";
    } else {
        std::cout << "No path found!" << "\n";
    }

    return 0;
}
