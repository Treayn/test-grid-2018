#include <algorithm>
#include <cmath>
#include "Pathfinder.h"
#include "Node.cpp"

// Const static.
const std::array<coordinate_t, 4> Pathfinder::directions = {{
    std::make_pair( 0,  1),
    std::make_pair( 1,  0),
    std::make_pair( 0, -1),
    std::make_pair(-1,  0)
}};

// Pass in heuristic functions here.
Pathfinder::Pathfinder(const std::vector<cell_query_t>&& heuristic_funcs, const Point& start, const Point& end) :
    heuristics(std::move(heuristic_funcs)),
    unvisited(
        [](const Node& lhs, const Node& rhs) -> bool { return lhs.getHeuristicCost() < rhs.getHeuristicCost(); }
    ),
    visited(
        1000,
        [](const Node& node) -> std::size_t { return ((int16_t) node.first << 8) | node.second },
        [](const Node& lhs, const Node& rhs) -> bool { return ((lhs.getX() == rhs.getX()) && (lhs.getY() == rhs.getY())); }
    ),
    last_heading(0.0)
{
    // Push goal onto priority queue with huge value.
    unvisited.push(new Node((int8_t) end.x, (int8_t) end.y, std::numeric_limits<float>::max()));

    // Push starting point onto priority queue with 0 cost.
    unvisited.push(new Node((int8_t) start.x, (int8_t) start.y, 0.0);
    
    // Create goal reference for algorithm termination.
    goal = unvisited.top();
}

float Pathfinder::calculateCosts(Node * node) {
    return std::accumulate(
        heuristics.begin(),
        heuristics.end(),
        node->getHeuristicCost(),
        [](float cost_sum, cell_query_t cost_func) {
            return cost_sum + cost_func(node->getX(), node->getY());
        }
    );
}

void Pathfinder::filter(Node * base_node, Node * ancestor_node) {
    if(ancestor_node) {                 // Check for nullptr/last point.
        if(ancestor_node->getParent()) {
            // Check for significant heading changes along the path.
            if(abs(node->getHeading() - ancestor_node->getHeading() > 20.0) {
                Point point;
                point.x = ancestor_node->getX();
                point.y = ancestor_node->getY();
                point.theta = base_node->getHeading();

                path.push_back(point);  // Push point associated with heading change onto path.

                retrace(ancestor_node, ancestor_node->getParent());
            } else {                    // Keep looking for significant heading changes.
                retrace(base_node, ancestor_node->getParent());
            }
        } else {                        // Found starting node.
            Point first;
            first.x = base_node->getX();
            first.y = base_node->getY();
            first.theta = atan2(
                base_node->getY() - ancestor_node->getY(),
                base_node->getX() - ancestor_node->getX()
            );

            path.push_back(first);      // Push first point onto path.
        }
    }
}

void Pathfinder::retrace() {
    Point last;
    last.x = unvisited.top()->getX();
    last.y = unvisited.top()->getY();
    last.theta = atan2(
        unvisited.top()->getY() - unvisited.top()->getParent()->getY(),
        unvisited.top()->getX() - unvisited.top()->getParent()->getX()
    );

    path.push_back(last);               // Push last point onto path.

    filterHeading(unvisited.top(), unvisited.top()->getParent());

    std::reverse(path.begin(), path.end());
}

void Pathfinder::traverse() {
    Node * unvisited_node = unvisited.top();
    std::unordered_set<Node>::iterator existing_node = visited.find(unvisited_node);

    if(unvisited_node == goal)          // If the addresses match we've hit our goal.
        return;                         // Stop recursion.
    else if(exists == visited.end()) {  // Else if node does not exist in visited set.
        std::for_each(Pathfinder::directions.start(), Pathfinder::directions.end(), [unvisited_node](const coordinate_t& neighbor) {
            visit(unvisited_node, neighbor);
        });
        
        visited.insert(unvisited_node); // Add current node to visited list.
        unvisited.pop();                // Remove current node from priority queue.
    }
    // Else node is in visited list, skip node.

    traverse();   // Traverse next node.
}

// Floating-point values in grid are truncated when stored, so no need to truncate here!
void Pathfinder::visit(Node * node, const coordinate_t& neighbor) {
    Node new_node = *node + neighbor;   // Create child node (Addition operator overloaded).

    std::unordered_set<Node>::iterator existing_node = visited.find(&new_node);
    float traversal_cost = calculateCosts(node);

    if(existing_node != visited.end() { // Node has been visited before.
        // If a shorter path to node has been found, update cost.
        if(existing_node->getHeuristicCost() > traversal_cost) {
            existing_node->setHeuristicCost(traversal_cost);
            existing_node->setParent(node);
        }
    } else {                            // Unvisited node.
        new_node.setHeuristicCost(traversal_cost);
        unvisited.push(new_node);       // Add node to priority queue.
    }
}

// Public Methods
const std::vector<Point> Pathfinder::findPath() {
    std::vector<Point> path;
    traverse();                         // Start pathing algorithm.
    retrace();
    return path;
}