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
        double lowest_g = open_list.begin()->g;
        auto current_node_iterator = open_list.begin();
        Node *current_node = &(*current_node_iterator);
        for (auto it = open_list.begin(); it != open_list.end(); ++it) {
            if (it->g < lowest_g) {
                lowest_g = it->g;
                current_node_iterator = it;
                current_node = &(*it);
            }
        }

        close_list.push_back(*current_node);
        open_list.erase(current_node_iterator);

        if (current_node->i == goal_i && current_node->j == goal_j) {
            makePrimaryPath(*current_node);

            sresult.pathfound = 1;
            sresult.nodescreated = open_list.size() + close_list.size();
            sresult.numberofsteps = -1;
            sresult.time = -1;
            //sresult.hppath = &hppath; //Here is a constant pointer
            sresult.lppath = &lppath;
            return sresult;
        }

        std::vector<Node> successors = get_successors(*current_node, map);
        for (Node node : successors) {
            bool found = false;
            for (const Node& closed : close_list) {
                if (closed.i == node.i && closed.j == node.j) {
                    found = true;
                }
            }
            if (found) continue;

            for (Node& opened : open_list) {
                if (opened.i == node.i && opened.j == node.j) {
                    if (opened.g > current_node->g + map.getCellSize()) {
                        opened.g = current_node->g + map.getCellSize();
                        opened.parent = current_node;
                    }
                    found = true;
                }
            }

            if (!found) {
                open_list.emplace_back(Node {node.i, node.j, 0,
                                             current_node->g + map.getCellSize(),
                                             0, current_node});
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
