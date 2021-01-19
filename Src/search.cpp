#include "search.h"

Search::Search()
{
//set defaults here
}

Search::~Search() {
    for (Node *node : close_list) {
        free(node);
    }
    for (Node *node : open_list) {
        free(node);
    }
}


SearchResult Search::startSearch(ILogger *Logger, const Map &map, const EnvironmentOptions &options)
{
    //need to implement

    Node *start = (Node *) malloc(sizeof(*start));
    start->i = map.get_start_i();
    start->j = map.get_start_j();
    start->g = 0;
    start->parent = NULL;
    int goal_i = map.get_goal_i();
    int goal_j = map.get_goal_j();

    open_list.push_back(start);

    while (!open_list.empty()) {
        double lowest_g = (*open_list.begin())->g;
        auto current_node_iterator = open_list.begin();
        Node *current_node = *current_node_iterator;
        for (auto it = open_list.begin(); it != open_list.end(); ++it) {
            if ((*it)->g < lowest_g) {
                lowest_g = (*it)->g;
                current_node_iterator = it;
                current_node = *it;
            }
        }

        close_list.push_back(current_node);
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
            for (const Node *closed : close_list) {
                if (closed->i == node.i && closed->j == node.j) {
                    found = true;
                }
            }
            if (found) continue;

            for (Node *opened : open_list) {
                if (opened->i == node.i && opened->j == node.j) {
                    if (opened->g > current_node->g + map.getCellSize()) {
                        opened->g = current_node->g + map.getCellSize();
                        opened->parent = current_node;
                    }
                    found = true;
                }
            }

            if (!found) {
                Node *new_node = (Node *) malloc(sizeof(*new_node));
                new_node->i = node.i;
                new_node->j = node.j;
                new_node->g = current_node->g + map.getCellSize();
                new_node->parent = current_node;
                open_list.push_back(new_node);
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
    std::vector<Node> successors;
    int i = node.i;
    int j = node.j;
    if (i - 1 >= 0) {
        successors.emplace_back(Node {i - 1, j, 0, 0, 0, NULL});
    }
    if (i + 1 < map.getMapHeight()) {
        successors.emplace_back(Node {i + 1, j, 0, 0, 0, NULL});
    }
    if (j - 1 >= 0) {
        successors.emplace_back(Node {i, j - 1, 0, 0, 0, NULL});
    }
    if (j + 1 < map.getMapWidth()) {
        successors.emplace_back(Node {i, j + 1, 0, 0, 0, NULL});
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
