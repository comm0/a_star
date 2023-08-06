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

int main() {
    std::vector<std::vector<int>> grid = {
        {0, 1, 0, 0, 0},
        {0, 0, 0, 1, 0},
        {0, 1, 0, 1, 0},
        {0, 0, 0, 0, 0},
        {0, 0, 0, 1, 0}
    };

    cell_t start(0, 0);
    cell_t goal(4, 4);

    std::vector<cell_t> path = a_star(grid, start, goal);

    if (!path.empty()) {
        std::cout << "Path found:" << std::endl;
        for (const auto& cell_t : path) {
            std::cout << "(" << cell_t.x << ", " << cell_t.y << ")" << std::endl;
        }
    } else {
        std::cout << "No path found!" << std::endl;
    }

    return 0;
}
