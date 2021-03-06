#include "A_star.h"

A_star::A_star() : search_type(CN_SP_ST_DIJK) {}

A_star::A_star(int type) : search_type(type) {}

A_star::~A_star() {}


void A_star::setSearchType(int type) {
    search_type = type;
}


SearchResult A_star::startSearch(ILogger *Logger, const Map &map, const EnvironmentOptions &options)
{
    auto time_start = std::chrono::high_resolution_clock::now();

    int goal_i = map.get_goal_i();
    int goal_j = map.get_goal_j();

    Node start{.i = map.get_start_i(), .j = map.get_start_j(),
            .F = 0, .g = 0, .H = 0, .parent = nullptr};

    start.H = calculate_heuristic(start.i, start.j, goal_i, goal_j, options.metrictype, search_type);
    start.F = start.H;

    open_list.insert(start);

    while (!open_list.empty()) {
        auto min_node = open_list.begin();
        int index = map.get_global_index(min_node->i, min_node->j);

        while (close_list.find(index) != close_list.end()) {
            open_list.erase(min_node);
            min_node = open_list.begin();
            index = map.get_global_index(min_node->i, min_node->j);
        }

        close_list[index] = *min_node;
        Node *current_node = &close_list[index];
        open_list.erase(min_node);

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
            double edge_weight = calculate_distance(node_i, node_j, current_node->i, current_node->j);

            double h = calculate_heuristic(node_i, node_j, goal_i, goal_j, options.metrictype, search_type);
            double g = current_node->g + edge_weight;
            open_list.emplace(Node {.i = node_i, .j = node_j, .F = g + h,
                    .g = g, .H = h, .parent = current_node});
        }
    }

    auto time_finish = std::chrono::high_resolution_clock::now();
    auto time = std::chrono::duration_cast<std::chrono::microseconds>(time_finish - time_start);
    sresult.time = time.count() / 1000000.;
    sresult.nodescreated = open_list.size() + close_list.size();
    sresult.numberofsteps = close_list.size();

    return sresult;
}


std::vector<int> A_star::get_successors(const Node &node, const Map &map, const EnvironmentOptions &options) const
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


void A_star::makePrimaryPath(Node& curNode)
{
    sresult.pathlength = curNode.g;
    lppath.push_back(curNode);
    while (curNode.parent) {
        curNode = *curNode.parent;
        lppath.push_front(curNode);
    }
}


void A_star::makeSecondaryPath()
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


double A_star::calculate_heuristic(int i1, int j1, int i2, int j2, int type, int search_type) const
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


double A_star::calculate_distance(int i1, int j1, int i2, int j2) const {
    int dx = std::abs(i1 - i2);
    int dy = std::abs(j1 - j2);

    if (dx == 0) {
        return dy;
    } else if (dy == 0) {
        return dx;
    } else {
        return std::sqrt(dx * dx + dy * dy);
    }
}


bool A_star::is_cell_passable(int i, int j, const Map &map) const {
    if (i >= 0 && i < map.getMapHeight() &&
        j >= 0 && j < map.getMapWidth() &&
        !map.CellIsObstacle(i, j)) {
        return true;
    }
    return false;
}


