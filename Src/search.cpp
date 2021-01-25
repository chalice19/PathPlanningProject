#include "search.h"

Search::Search()
{
//set defaults here
}

Search::~Search() {
}


SearchResult Search::startSearch(ILogger *Logger, const Map &map, const EnvironmentOptions &options)
{
    Node start{.i = map.get_start_i(), .j = map.get_start_j(),
               .F = 0, .g = 0, .H = 0, .parent = nullptr};
    int goal_i = map.get_goal_i();
    int goal_j = map.get_goal_j();

    open_list.push_back(start);

    while (!open_list.empty()) {
        double lowest_g = open_list.begin()->g;
        auto current_node_iterator = open_list.begin();
        for (auto it = open_list.begin(); it != open_list.end(); ++it) {
            if (it->g < lowest_g) {
                lowest_g = it->g;
                current_node_iterator = it;
            }
        }

        int index = map.get_global_index(current_node_iterator->i, current_node_iterator->j);
        close_list[index] = *current_node_iterator;
        Node *current_node = &close_list[index];
        open_list.erase(current_node_iterator);


        if (current_node->i == goal_i && current_node->j == goal_j) {
            makePrimaryPath(*current_node);

            sresult.pathfound = true;
            sresult.pathlength = lppath.size();
            sresult.lppath = &lppath;
            sresult.hppath = &hppath;
            sresult.nodescreated = open_list.size() + close_list.size();
            sresult.numberofsteps = close_list.size();
            sresult.time = -1;

            break;
        }

        std::vector<int> successors = get_successors(*current_node, map);
        for (int map_index : successors) {
            if (close_list.find(map_index) != close_list.end()) {
                continue;
            }

            int node_i = map_index / map.getMapWidth();
            int node_j = map_index % map.getMapWidth();
            bool found = false;

            for (Node& opened : open_list) {
                if (opened.i == node_i && opened.j == node_j) {
                    if (opened.g > current_node->g + map.getCellSize()) {
                        opened.g = current_node->g + map.getCellSize();
                        opened.parent = current_node;
                    }
                    found = true;
                }
            }

            if (!found) {
                open_list.emplace_back(Node {.i = node_i, .j = node_j, .F = 0,
                                             .g = current_node->g + map.getCellSize(),
                                             .H = 0, .parent = current_node});
            }
        }
    }

    return sresult;
}


std::vector<int> Search::get_successors(Node &node, const Map &map) const
{
    std::vector<int> successors;
    int i = node.i;
    int j = node.j;
    if (i - 1 >= 0 && !map.CellIsObstacle(i - 1, j)) {
        successors.emplace_back(map.get_global_index(i - 1, j));
    }
    if (i + 1 < map.getMapHeight() && !map.CellIsObstacle(i + 1, j)) {
        successors.emplace_back(map.get_global_index(i + 1, j));
    }
    if (j - 1 >= 0 && !map.CellIsObstacle(i, j - 1)) {
        successors.emplace_back(map.get_global_index(i, j - 1));
    }
    if (j + 1 < map.getMapWidth() && !map.CellIsObstacle(i, j + 1)) {
        successors.emplace_back(map.get_global_index(i, j + 1));
    }
    return successors;
}


void Search::makePrimaryPath(Node curNode)
{
    lppath.push_back(curNode);
    while (curNode.parent) {
        curNode = *curNode.parent;
        lppath.push_front(curNode);
    }
}


/*void Search::makeSecondaryPath()
{
    //need to implement
}*/
