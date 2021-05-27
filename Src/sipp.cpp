#include "sipp.h"

Sipp::Sipp() : A_star() {}

Sipp::~Sipp() {}


SearchResult Sipp::startSearch(ILogger *Logger, const Map &map,
                               const EnvironmentOptions &options,
                               const SafeIntervals &safe_intervals)
{
    auto time_start = std::chrono::high_resolution_clock::now();

    int goal_i = map.get_goal_i();
    int goal_j = map.get_goal_j();

    Node start{.i = map.get_start_i(), .j = map.get_start_j(),
            .F = 0, .g = 0, .H = 0, .parent = nullptr, .si_i = 0};

    start.H = calculate_heuristic(start.i, start.j, goal_i, goal_j, options.metrictype);
    start.F = start.H;

    open_list.insert(start);

    while (!open_list.empty()) {
        auto min_node = open_list.begin();
        int index = get_global_index(min_node->i, min_node->j, min_node->si_i, map);

        while (close_list.find(index) != close_list.end()) {
            open_list.erase(min_node);
            min_node = open_list.begin();
            index = get_global_index(min_node->i, min_node->j, min_node->si_i, map);
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

        std::vector<Node> successors = get_successors_sipp(*current_node, map, options, safe_intervals);
        for (Node& node : successors) {
            int num_of_cells = map.getMapWidth() * map.getMapWidth();
            int map_index = node.si_i * num_of_cells + map.get_global_index(node.i, node.j);

            if (close_list.find(map_index) != close_list.end()) {
                continue;
            }

            double h = calculate_heuristic(node.i, node.j, goal_i, goal_j, options.metrictype);
            double g = node.g;
//            open_list.emplace(Node {.i = node.i, .j = node.j, .F = g + h, .g = g,
//                                    .H = h, .parent = current_node, .si_i = node.si_i});
            node.H = h;
            node.F = g + h;
            node.parent = current_node;
            open_list.insert(node);
        }
    }

    auto time_finish = std::chrono::high_resolution_clock::now();
    auto time = std::chrono::duration_cast<std::chrono::microseconds>(time_finish - time_start);
    sresult.time = time.count() / 1000000.;
    sresult.nodescreated = open_list.size() + close_list.size();
    sresult.numberofsteps = close_list.size();

    return sresult;
}


std::vector<Node> Sipp::get_successors_sipp(Node &node, const Map &map,
                                            const EnvironmentOptions &options,
                                            const SafeIntervals &safe_intervals) const {
    EnvironmentOptions sipp_options;
    sipp_options.metrictype = options.metrictype;
    sipp_options.allowdiagonal = false;

    std::vector<int> motion_successors = get_successors(node, map, sipp_options);
    std::vector<Node> successors;

    for (auto global_index : motion_successors) {
        int i = global_index / map.getMapWidth();
        int j = global_index % map.getMapWidth();
//        consider motions only of cost 1
//        auto motion_time = (unsigned int)calculate_distance(i, j, node.i, node.j);
        unsigned int motion_time = 1;
        unsigned int start_t = node.g + motion_time;
        unsigned int end_t = (safe_intervals.get_interval(node.i, node.j, node.si_i, map)).end + motion_time;

        int si_i = 0;
        for (auto safe_interval : safe_intervals.get_intervals(i, j, map)) {
            unsigned int si_start = safe_interval.begin;
            unsigned int si_end = safe_interval.end;

            if (si_start <= end_t && si_end >= start_t) {
                unsigned int t = std::max(start_t, si_start);
                // add to successors state i, j with time t and safe interval si_i;
                successors.emplace_back(Node{.i = i, .j = j, .F = 0, .g = t, .H = 0,
                                             .parent = nullptr, .si_i = si_i});
            }

            ++si_i;
        }
    }

    return successors;
}

double Sipp::calculate_cost(Node& n1, Node& n2) const {
    return n2.g - n1.g;
}

int Sipp::get_global_index(int i, int j, int si, const Map &map) const {
    int num_of_cells = map.getMapHeight() * map.getMapWidth();
    return si * num_of_cells + map.get_global_index(i, j);
}