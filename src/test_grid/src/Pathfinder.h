/** 
 *  @file    GridManager.h
 *  @author  Eric Tung (Treayn) & Ronuel Diaz (rdiaz93)
 *  @date    3/09/2018
 *  @version 0.0.2 ALPHA
 *  
 *  @brief Grid Class
 *
 *  @section DESCRIPTION
 *  
 *  The Pathfinder Class serves as an implementation of various
 *    Path Planning algorithms (Djikstra's, A*, etc).
 *  
 *  This implementation is Heuristic-Agnostic - The user
 *    specifies what heuristic to compute via lambda functions
 *    which capture NodeHandles connected to the grid service.
 *  
 *  Givben that Djikstra's/A* will return every point/cell to
 *    the goal, this implementation also filters out most of
 *    the points, only returning points that denote significant
 *    heading changes.
**/

#ifndef PATHFINDER_H
#define PATHFINDER_H

#include <array>
#include <cstdlib>
#include <ctime>
#include <deque>
#include <functional>
#include <queue>
#include <unordered_set>
#include <utility>
#include <vector>

// Point objects are used to interface with the rest of the codebase.
// Internally, Pathfinder uses std::pairs to represent coordinates.
#include "Point.h"

// Pathfinder represents our algorithm, which could be Djikstra's, A*, D* Lite, etc.
class Pathfinder {
private:
    // Use pairs as coordinates.
    typedef std::pair<int8_t, int8_t> coordinate_t;

    // Internal node class.
    class Node {
    private:
        coordinate_t coordinates, parent_coordinates;
        float total_heuristic_cost, heading;
        Node * parent;

    public:
        Node(int8_t, int8_t, float);
        Node(int8_t, int8_t, float, const Node&);   // Copy Constructor.

        Node operator+(const coordinate_t&);
        
        int8_t getX();                              // X & Y set by constructor. No mutators.
        int8_t getY();

        float getHeuristicCost();
        void setHeuristicCost(float);

        float getHeading();                         // Heading set by constructor. No mutators.

        Node * getParent();
        void setParent(const Node *);               // Change parent if shorter path to node is found.
    }

    // Typedefs for various hashing & comparator functions.
    typedef std::function<float(uint8_t, uint8_t)>          cell_query_func_t;
    typedef std::function<std::size_t(const Node&)>         hash_func_t;
    typedef std::function<bool(const Node&, const Node&)>   comparator_t;
    
    // Vector that holds heuristics
    // The pathfinding algorithm will call each function in the vector and accumulate the total.
    // so f(x) + g(x) + ... + n(x).
    // The first function in the vector should be to get the node cost from the grid server.
    // 
    // Now we can have pathfinding objects which search based on different criteria.
    std::vector<cell_query_func_t> heuristics;

    // Core data structures for pathfinding and their associated functions.
    std::priority_queue<Node, std::deque<Node>, comparator_t> unvisited;
    std::unordered_set<Node, hash_func_t, comparator_t> visited;
    Node * goal;

    static const std::array<coordinate_t, 4> directions;

    float last_heading;
    std::vector path;

    void filter();
    float calculateCosts(Node *);
    void retrace(Node *);
    void traverse();
    void visit(Node *, const coordinate_t&);

public:
    Pathfinder(const std::vector<cell_query_func_t>&, const Point&, const Point&);
    ~Pathfinder();

    std::vector<Point> findPath();
};

#endif