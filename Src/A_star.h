#ifndef A_STAR_H
#define A_STAR_H
#include "ilogger.h"
#include "searchresult.h"
#include "environmentoptions.h"
#include "node_comparator.h"
#include <vector>
#include <math.h>
#include <map>
#include <chrono>
#include <set>

class A_star
{
public:
    A_star();
    A_star(int type);
    ~A_star(void);
    SearchResult startSearch(ILogger *Logger, const Map &Map, const EnvironmentOptions &options);

    void setSearchType(int type);

protected:
    //CODE HERE

    //Hint 1. You definitely need class variables for OPEN and CLOSE

    //Hint 2. It's a good idea to define a heuristic calculation function, that will simply return 0
    //for non-heuristic search methods like Dijkstra

    //Hint 3. It's a good idea to define function that given a node (and other stuff needed)
    //will return it's successors, e.g. unordered list of nodes

    //Hint 4. working with OPEN and CLOSE is the core
    //so think of the data structures that needed to be used, about the wrap-up classes (if needed)
    //Start with very simple (and ineffective) structures like list or vector and make it work first
    //and only then begin enhancement!

    SearchResult                    sresult; //This will store the search result
    std::list<Node>                 lppath, hppath;

    //CODE HERE to define other members of the class

    int                             search_type;

    std::set<Node, NodeComparator>  open_list;
    std::unordered_map<int, Node>   close_list;

    void makePrimaryPath(Node& curNode);
    void makeSecondaryPath();   // uses lppath, so it should be called after makePrimaryPath()

    std::vector<int> get_successors(const Node &node, const Map& map, const EnvironmentOptions &options) const;
    double calculate_heuristic(int i1, int j1, int i2, int j2, int type, int search_type) const;
    double calculate_distance(int i1, int j1, int i2, int j2) const;
    bool is_cell_passable(int i, int j, const Map& map) const;
};
#endif

