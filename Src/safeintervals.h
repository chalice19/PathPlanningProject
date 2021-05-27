#ifndef SAFE_INTERVALS_H
#define SAFE_INTERVALS_H
#include "map.h"
#include "safeinterval.h"
#include <limits>
#include <unordered_map>
#include <vector>

class SafeIntervals {
public:
    SafeIntervals();
    ~SafeIntervals();

    void compute_safe_intervals(const char *FileName, const Map &map);

    const std::vector<SafeInterval>& get_intervals(int i, int j, const Map &map) const;
    const SafeInterval& get_interval(int i, int j, int si_i, const Map &map) const;

    unsigned int inf_time;

protected:
    std::unordered_map<int, std::vector<SafeInterval>>    safe_intervals;
};

#endif