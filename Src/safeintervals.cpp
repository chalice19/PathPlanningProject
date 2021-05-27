#include "safeintervals.h"

SafeIntervals::SafeIntervals() {}

SafeIntervals::~SafeIntervals() {}

void SafeIntervals::compute_safe_intervals(const char *FileName, const Map &map) {
    inf_time = std::numeric_limits<unsigned int>::max();

    for (int i = 0; i < map.getMapHeight(); ++i) {
        for (int j = 0; j < map.getMapWidth(); ++j) {
            if (!map.CellIsObstacle(i, j)) {
                std::vector<SafeInterval> si(1);
                si[0] = SafeInterval{.begin = 0, .end = inf_time};
                safe_intervals[map.get_global_index(i, j)] = si;
            }
        }
    }
}

const std::vector<SafeInterval>& SafeIntervals::get_intervals(int i, int j, const Map &map) const {
//    if (map.CellIsObstacle(i, j)) {
//        return {};
//    }
    return safe_intervals.find(map.get_global_index(i, j))->second;
}

const SafeInterval& SafeIntervals::get_interval(int i, int j, int si_i, const Map &map) const {
    return get_intervals(i, j, map)[si_i];
}
