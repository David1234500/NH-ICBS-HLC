#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <algorithm>
#include <functional>
#include <cassert>

struct Position {
    int x, y;

    bool operator==(const Position& other) const {
        return x == other.x && y == other.y;
    }
};

// Custom hash function for Position
namespace std {
    template <>
    struct hash<Position> {
        size_t operator()(const Position& p) const {
            return hash<int>()(p.x) ^ hash<int>()(p.y);
        }
    };
}

struct Agent {
    int id;
    Position start;
    Position goal;
};

struct Node {
    Position pos;
    int g;
    int h;
    Position parent;
};

// Custom hash function for Node
namespace std {
    template <>
    struct hash<Node> {
        size_t operator()(const Node& n) const {
            return hash<Position>()(n.pos);
        }
    };
}

bool operator==(const Node& lhs, const Node& rhs) {
    return lhs.pos == rhs.pos;
}

struct NodeCompare {
    bool operator()(const Node& lhs, const Node& rhs) const {
        return lhs.g + lhs.h > rhs.g + rhs.h;
    }
};

using Grid = std::vector<std::vector<bool>>;
using Path = std::vector<Position>;

Grid createGrid(int width, int height, const std::vector<Position>& obstacles) {
    Grid grid(height, std::vector<bool>(width, false));

    for (const auto& obstacle : obstacles) {
        grid[obstacle.y][obstacle.x] = true;
    }

    return grid;
}

bool isPositionValid(const Grid& grid, const Position& pos) {
    return pos.y >= 0 && pos.y < static_cast<int>(grid.size()) &&
           pos.x >= 0 && pos.x < static_cast<int>(grid[0].size()) &&
           !grid[pos.y][pos.x];
}

std::vector<Position> getNeighbors(const Grid& grid, const Position& pos) {
    std::vector<Position> neighbors;

    const std::vector<Position> potentialNeighbors = {
        {pos.x - 1, pos.y}, {pos.x + 1, pos.y},
        {pos.x, pos.y - 1}, {pos.x, pos.y + 1}
    };

    for (const auto& neighbor : potentialNeighbors) {
        if (isPositionValid(grid, neighbor)) {
            neighbors.push_back(neighbor);
        }
    }

    return neighbors;
}

int manhattanDistance(const Position& a, const Position& b) {
    return std::abs(a.x - b.x) + std::abs(a.y - b.y);
}

Path reconstructPath(const std::unordered_map<Position, Position>& cameFrom, const Position& start, const Position& goal) {
    Path path;
    Position current = goal;

    while (current != start) {
        path.push_back(current);
        current = cameFrom.at(current);
    }

    path.push_back(start);
    std::reverse(path.begin(), path.end());

    return path;
}

Path findPathAStar(const Grid& grid, const Position& start, const Position& goal) {
    std::priority_queue<Node, std::vector<Node>, NodeCompare> openList;
    std::unordered_map<    Position, Node> allNodes;
    std::unordered_map<Position, Position> cameFrom;
    Node startNode{start, 0, manhattanDistance(start, goal), {-1, -1}};
    allNodes[start] = startNode;
    openList.push(startNode);

    while (!openList.empty()) {
        Node current = openList.top();
        openList.pop();

        if (current.pos == goal) {
            return reconstructPath(cameFrom, start, goal);
        }

        for (const Position& neighbor : getNeighbors(grid, current.pos)) {
            int tentative_g = current.g + 1;

            auto it = allNodes.find(neighbor);
            if (it == allNodes.end() || tentative_g < it->second.g) {
                cameFrom[neighbor] = current.pos;
                Node neighborNode{neighbor, tentative_g, manhattanDistance(neighbor, goal), current.pos};
                allNodes[neighbor] = neighborNode;
                openList.push(neighborNode);
            }
        }
    }

    // No path found
    return Path{};
}

bool checkCollision(const Path& path1, const Path& path2) {
    size_t maxLength = std::max(path1.size(), path2.size());

    for (size_t t = 0; t < maxLength; ++t) {
        Position pos1 = (t < path1.size()) ? path1[t] : path1.back();
        Position pos2 = (t < path2.size()) ? path2[t] : path2.back();

        if (pos1 == pos2) {
            return true;
        }
    }

    return false;
}

std::vector<Path> findMultiAgentPaths(const Grid& grid, const std::vector<Agent>& agents) {
    std::vector<Path> paths(agents.size());

    for (const Agent& agent : agents) {
        Path path = findPathAStar(grid, agent.start, agent.goal);
        assert(!path.empty());
        paths[agent.id] = path;
    }

    // Resolve collisions with ID
    bool hasCollisions = true;
    while (hasCollisions) {
        hasCollisions = false;

        for (size_t i = 0; i < agents.size(); ++i) {
            for (size_t j = i + 1; j < agents.size(); ++j) {
                if (checkCollision(paths[i], paths[j])) {
                    hasCollisions = true;
                    // Replan the path for the agent with a higher ID
                    size_t replanAgentIdx = std::max(agents[i].id, agents[j].id);
                    const Agent& replanAgent = agents[replanAgentIdx];
                    paths[replanAgent.id] = findPathAStar(grid, replanAgent.start, replanAgent.goal);
                    assert(!paths[replanAgent.id].empty());
                }
            }
        }
    }

    return paths;
}


// Check if there is a conflict between two paths at a specific time step
bool hasConflict(const Path& path1, const Path& path2, int time) {
    const Position& pos1 = (time < static_cast<int>(path1.size())) ? path1[time] : path1.back();
    const Position& pos2 = (time < static_cast<int>(path2.size())) ? path2[time] : path2.back();

    return pos1 == pos2;
}

// Check if a set of paths are conflict-free
bool isConflictFree(const std::vector<Path>& paths) {
    for (size_t i = 0; i < paths.size(); ++i) {
        for (size_t j = i + 1; j < paths.size(); ++j) {
            for (size_t t = 0; t < std::max(paths[i].size(), paths[j].size()); ++t) {
                if (hasConflict(paths[i], paths[j], t)) {
                    return false;
                }
            }
        }
    }
    return true;
}

// Find a set of conflict-free paths for agents with a given cost limit
std::vector<Path> findPathsICTS(const Grid& grid, const std::vector<Agent>& agents, const std::vector<int>& costs) {
    std::vector<Path> paths(agents.size());
    for (size_t i = 0; i < agents.size(); ++i) {
        paths[i] = findPathAStar(grid, agents[i].start, agents[i].goal, costs[i]);
    }
    return paths;
}

// ICTS main function
std::vector<Path> findMultiAgentPathsICTS(const Grid& grid, const std::vector<Agent>& agents) {
    std::vector<int> costs(agents.size(), 0);
    std::vector<Path> paths = findPathsICTS(grid, agents, costs);

    while (!isConflictFree(paths)) {
        // Increase the cost limit for one of the agents
        bool foundBetterCost = false;
        for (size_t i = 0; i < agents.size(); ++i) {
            if (!foundBetterCost) {
                costs[i]++;
                std::vector<Path> newPaths = findPathsICTS(grid, agents, costs);
                if (isConflictFree(newPaths)) {
                    paths = newPaths;
                    foundBetterCost = true;
                } else {
                    costs[i]--;
                }
            }
        }

        if (!foundBetterCost) {
            // If no better cost was found, increment the cost limit for all agents
            for (size_t i = 0; i < agents.size(); ++i) {
                costs[i]++;
            }
            paths = findPathsICTS(grid, agents, costs);
        }
    }

    return paths;
}

// Update main() function

int main() {
    Grid grid = createGrid(5, 5, {{1, 1}, {1, 2}, {1, 3}});

    std::vector<Agent> agents = {
        {0, {0, 0}, {4, 4}},
        {1, {0, 4}, {4, 0}},
    };

    std::vector<Path> paths = findMultiAgentPathsICTS(grid, agents);

    for (size_t i = 0; i < paths.size(); ++i) {
        std::cout << "Path for agent " << agents[i].id << ": ";
        for (const Position& pos : paths[i]) {
            std::cout << "(" << pos.x << ", " << pos.y << ") ";
        }
        std::cout << std::endl;
    }

    return 0;
}
