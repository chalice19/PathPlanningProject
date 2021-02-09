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

    int goal_i = map.get_goal_i();
    int goal_j = map.get_goal_j();

    Node start{.i = map.get_start_i(), .j = map.get_start_j(),
            .F = 0, .g = 0, .H = 0, .parent = nullptr};

    start.H = calculate_heuristic(start.i, start.j, goal_i, goal_j, options.metrictype);
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
            sresult.lppath = &lppath;
            sresult.hppath = &hppath;

            break;
        }

        std::vector<int> successors = get_successors(*current_node, map, options);
        for (int map_index : successors) {
            if (close_list.find(map_index) != close_list.end()) {
                continue;
            }

            int node_i = map_index / map.getMapWidth();
            int node_j = map_index % map.getMapWidth();
            double edge_weight = calculate_heuristic(node_i, node_j, current_node->i, current_node->j, CN_SP_MT_EUCL);
            bool found = false;

            for (Node& opened : open_list) {
                if (opened.i == node_i && opened.j == node_j) {
                    if (opened.g > current_node->g + edge_weight) {
                        opened.g = current_node->g + edge_weight;
                        opened.F = opened.g + opened.H;
                        opened.parent = current_node;
                    }
                    found = true;
                    break;
                }
            }

            if (!found) {
                double h = calculate_heuristic(node_i, node_j, goal_i, goal_j, options.metrictype);
                double g = current_node->g + edge_weight;
                open_list.emplace_back(Node {.i = node_i, .j = node_j, .F = g + h,
                                             .g = g, .H = h, .parent = current_node});
            }
        }
    }

    auto time_finish = std::chrono::high_resolution_clock::now();
    auto time = std::chrono::duration_cast<std::chrono::microseconds>(time_finish - time_start);
    sresult.time = time.count() / 1000000.;
    sresult.nodescreated = open_list.size() + close_list.size();
    sresult.numberofsteps = close_list.size();

    return sresult;
}


std::vector<int> Search::get_successors(Node &node, const Map &map, const EnvironmentOptions &options) const
{
    std::vector<int> successors;
    int i = node.i;
    int j = node.j;

    std::vector<int> i_ses = {i - 1, i + 1, i, i};
    std::vector<int> j_ses = {j, j, j - 1, j + 1};
    for (size_t index = 0; index < i_ses.size(); ++index) {
        if (is_cell_passable(i_ses[index], j_ses[index], map)) {
            successors.emplace_back(map.get_global_index(i_ses[index], j_ses[index]));
        }
    }
    if (!options.allowdiagonal) return successors;

    j_ses[0] = j - 1;
    j_ses[1] = j + 1;
    i_ses[2] = i + 1;
    i_ses[3] = i - 1;

    for (size_t index = 0; index < i_ses.size(); ++index) {
        if (is_cell_passable(i_ses[index], j_ses[index], map)) {
            int neighbors_passable = 0;
            for (int s : successors) {
                if ((s == map.get_global_index(i_ses[index], j)) ||
                    (s == map.get_global_index(i, j_ses[index]))) {
                    ++neighbors_passable;
                }
            }

            if (options.cutcorners) {
                if (options.allowsqueeze || neighbors_passable >= 1) {
                    successors.emplace_back(map.get_global_index(i_ses[index], j_ses[index]));
                    continue;
                }
            } else if (neighbors_passable == 2) {
                successors.emplace_back(map.get_global_index(i_ses[index], j_ses[index]));
            }
        }
    }

    return successors;
}


void Search::makePrimaryPath(Node& curNode)
{
    lppath.push_back(curNode);
    while (curNode.parent) {
        sresult.pathlength += calculate_heuristic(curNode.i, curNode.j,
                                                  curNode.parent->i, curNode.parent->j, CN_SP_MT_EUCL);
        curNode = *curNode.parent;
        lppath.push_front(curNode);
    }
}


void Search::makeSecondaryPath()
{
    if (lppath.empty()) return;

    hppath.push_back(*lppath.begin());
    if (lppath.size() < 3) {
        hppath.push_back(*(--lppath.end()));
        return;
    }

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

    hppath.push_back(*(--lppath.end()));
}


double Search::calculate_heuristic(int i1, int j1, int i2, int j2, int type) const
{
    if (search_type == CN_SP_ST_DIJK)
        return 0;

    int dx = std::abs(i1 - i2);
    int dy = std::abs(j1 - j2);

    if (type == CN_SP_MT_EUCL)
        return std::sqrt(dx * dx + dy * dy);

    if (type == CN_SP_MT_MANH)
        return dx + dy;

    if (type == CN_SP_MT_CHEB)
        return std::max(dx, dy);

    if (type == CN_SP_MT_DIAG)
        return std::abs(dx - dy) + std::sqrt(2) * std::min(dx, dy);

    return 0;
}

bool Search::is_cell_passable(int i, int j, const Map &map) const {
    if (i >= 0 && i < map.getMapHeight() &&
        j >= 0 && j < map.getMapWidth() &&
        !map.CellIsObstacle(i, j)) {
        return true;
    }
    return false;
}

