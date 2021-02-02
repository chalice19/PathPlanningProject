#include "search.h"

Search::Search()
{
    search_type = CN_SP_ST_DIJK;
}

Search::~Search() {}


void Search::setSearchType(int type) {
    search_type = type;
}


SearchResult Search::startSearch(ILogger *Logger, const Map &map, const EnvironmentOptions &options)
{
    auto time_start = std::chrono::high_resolution_clock::now();

    Node start{.i = map.get_start_i(), .j = map.get_start_j(),
               .F = 0, .g = 0, .H = 0, .parent = nullptr};
    int goal_i = map.get_goal_i();
    int goal_j = map.get_goal_j();

    start.H = calculate_heuristic(std::abs(start.i - goal_i),
                           std::abs(start.j - goal_j),
                           options.metrictype);
    start.F = start.H;

    open_list.push_back(start);

    while (!open_list.empty()) {
        double lowest_f = open_list.begin()->F;
        auto current_node_iterator = open_list.begin();
        for (auto it = open_list.begin(); it != open_list.end(); ++it) {
            if (it->F < lowest_f) {
                lowest_f = it->F;
                current_node_iterator = it;
            }
        }

        int index = map.get_global_index(current_node_iterator->i, current_node_iterator->j);
        close_list[index] = *current_node_iterator;
        Node *current_node = &close_list[index];
        open_list.erase(current_node_iterator);


        if (current_node->i == goal_i && current_node->j == goal_j) {
            makePrimaryPath(*current_node);
            makeSecondaryPath();

            sresult.pathfound = true;
            sresult.pathlength = lppath.size();
            sresult.lppath = &lppath;
            sresult.hppath = &hppath;

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
                    if (opened.g > current_node->g + 1) {
                        opened.g = current_node->g + 1;
                        opened.F = opened.g + opened.H;
                        opened.parent = current_node;
                    }
                    found = true;
                    break;
                }
            }

            if (!found) {
                int dx = std::abs(node_i - goal_i);
                int dy = std::abs(node_j - goal_j);
                double h = calculate_heuristic(dx, dy, options.metrictype);
                double g = current_node->g + 1;
                open_list.emplace_back(Node {.i = node_i, .j = node_j, .F = g + h,
                                             .g = g, .H = h, .parent = current_node});
            }
        }
    }

    sresult.nodescreated = open_list.size() + close_list.size();
    sresult.numberofsteps = close_list.size();

    auto time_finish = std::chrono::high_resolution_clock::now();
    auto time = std::chrono::duration_cast<std::chrono::microseconds>(time_finish - time_start);
    sresult.time = time.count() / 1000000.;

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


void Search::makePrimaryPath(Node& curNode)
{
    lppath.push_back(curNode);
    while (curNode.parent) {
        curNode = *curNode.parent;
        lppath.push_front(curNode);
    }
}


void Search::makeSecondaryPath()
{
    if (lppath.empty()) return;

    hppath.push_back(*lppath.begin());
    hppath.push_back(*(--lppath.end()));
    if (lppath.size() < 3) return;

    auto prev = ++lppath.begin();
    int di_prev = prev->i - lppath.begin()->i;
    int dj_prev = prev->j - lppath.begin()->j;

    auto it = prev;
    ++it;
    for (; it != lppath.end(); ++it) {
        int di = it->i - prev->i;
        int dj = it->j - prev->j;

        if (di != di_prev || dj != dj_prev) {
            hppath.push_back(*prev);
        }

        di_prev = di;
        dj_prev = dj;
        prev = it;
    }
}


double Search::calculate_heuristic(int x, int y, int type) const
{
    if (search_type == CN_SP_ST_DIJK)
        return 0;


    if (type == CN_SP_MT_EUCL)
        return std::sqrt(x * x + y * y);

    if (type == CN_SP_MT_MANH)
        return x + y;

    if (type == CN_SP_MT_CHEB)
        return std::max(x, y);

    if (type == CN_SP_MT_DIAG)
        return std::abs(x - y) + std::sqrt(2) * std::min(x, y);

    return 0;
}