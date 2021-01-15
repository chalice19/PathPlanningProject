#include "search.h"

Search::Search()
{
//set defaults here
}

Search::~Search() {}


SearchResult Search::startSearch(ILogger *Logger, const Map &map, const EnvironmentOptions &options)
{
    //need to implement

    Node start{map.get_start_i(), map.get_start_j(), 0, 0, 0, NULL};
    int goal_i = map.get_goal_i();
    int goal_j = map.get_goal_j();

    open_list.push_back(start);

    while (!open_list.empty()) {
        double lowest_g = open_list[0].g;
        size_t current_node_index = 0;
        for (size_t i = 1; i < open_list.size(); ++i) {
            if (open_list[i].g < lowest_g) {
                lowest_g = open_list[i].g;
                current_node_index = i;
            }
        }

        close_list.push_back(open_list[current_node_index]);
        open_list.erase(open_list.begin() + current_node_index);
        current_node_index = close_list.size() - 1;
        Node current_node = close_list[current_node_index];

        if (current_node.i == goal_i && current_node.j == goal_j) {
            makePrimaryPath(current_node);

            sresult.pathfound = 1;
            sresult.nodescreated = open_list.size() + close_list.size();
            sresult.numberofsteps = -1;
            sresult.time = -1;
            //sresult.hppath = &hppath; //Here is a constant pointer
            sresult.lppath = &lppath;
            return sresult;
        }

        std::vector<Node> successors = get_successors(current_node, map);
        for (Node node : successors) {
            bool found = false;
            for (size_t i = 0; i < close_list.size(); ++i) {
                if (close_list[i].i == node.i && close_list[i].j == node.j) {
                    found = true;
                }
            }
            if (found) continue;

            for (size_t i = 0; i < open_list.size(); ++i) {
                if (open_list[i].i == node.i && open_list[i].j == node.j) {
                    if (open_list[i].g > current_node.g + map.getCellSize()) {
                        open_list[i].g = current_node.g + map.getCellSize();
                        open_list[i].parent = &current_node;
                    }
                    found = true;
                }
            }

            if (!found) {
                node.parent = &current_node;
                node.g = current_node.g + map.getCellSize();
                open_list.push_back(node);
            }
        }
    }

    sresult.pathfound = 0;
    sresult.nodescreated =  0;
    sresult.numberofsteps = 0;
    sresult.time = 0;
    //sresult.hppath = &hppath; //Here is a constant pointer
    //sresult.lppath = &lppath;
    return sresult;
}


std::vector<Node> Search::get_successors(Node &node, const Map &map) const
{
    std::vector<Node> succs;
    int i = node.i;
    int j = node.j;
    if (i - 1 >= 0) {
        succs.emplace_back(Node {i - 1, j, 0, 0, 0, NULL});
    }
    if (i + 1 < map.getMapHeight()) {
        succs.emplace_back(Node {i + 1, j, 0, 0, 0, NULL});
    }
    if (j - 1 >= 0) {
        succs.emplace_back(Node {i, j - 1, 0, 0, 0, NULL});
    }
    if (j + 1 < map.getMapWidth()) {
        succs.emplace_back(Node {i, j + 1, 0, 0, 0, NULL});
    }
    return succs;
//    for (int i = node.i - 1; i <= node.i + 1; ++i) {
//        for (int j = node.j - 1; j <= node.j + 1; ++j) {
//            if (i < 0 || i > map.getMapWidth()) {
//                continue;
//            }
//            if (j < 0 || j > map.getMapHeight()) {
//                continue;
//            }
//            if (i == node.i && j == node.j) {
//                continue;
//            }
//            Node succ = {i, j, 0, 0, 0, NULL};
//            succs.push_back(succ);
//        }
//    }
//    return succs;
}


void Search::makePrimaryPath(Node curNode)
{
    lppath.push_back(curNode);
    while (curNode.parent) {
        curNode = *curNode.parent;
        lppath.push_back(curNode);
    }
}


/*void Search::makeSecondaryPath()
{
    //need to implement
}*/
