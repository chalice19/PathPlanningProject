#ifndef SIPP_H
#define SIPP_H
#include "A_star.h"
#include "safeintervals.h"
#include <algorithm>

class Sipp : public A_star
{
public:
    Sipp();
    ~Sipp(void);

    SearchResult startSearch(ILogger *Logger, const Map &map,
                             const EnvironmentOptions &options, const SafeIntervals &safe_intervals);

protected:
    int               search_type;
    std::vector<Node> get_successors_sipp(Node &node, const Map &map,
                                          const EnvironmentOptions &options, const SafeIntervals &safe_intervals) const;
    int get_global_index(int i, int j, int si, const Map &map) const;
};

#endif