#ifndef SAFE_INTERVALS_H
#define SAFE_INTERVALS_H
#include "map.h"
#include "safeinterval.h"
#include "tinyxml2.h"
#include <limits>
#include <string>
#include <unordered_map>
#include <list>

class SafeIntervals {
public:
    SafeIntervals();
    ~SafeIntervals();

    bool compute_safe_intervals(const char *FileName, const Map &map);

    const std::list<SafeInterval>& get_intervals(int i, int j, const Map &map) const;
    const SafeInterval& get_interval(int i, int j, int si_i, const Map &map) const;

    int inf_time;

protected:
    int                                                 size;
    std::unordered_map<int, std::list<SafeInterval>>    safe_intervals;
    bool insert_interval(int i, int y, int t, const Map &map);

    std::list<SafeInterval>& get_intervals(int i, int j, const Map &map);
};

#endif