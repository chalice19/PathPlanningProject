#include "search.h"

Search::Search() : search_type(CN_SP_ST_DIJK) {}

Search::~Search() {}


void Search::setSearchType(int type) {
    search_type = type;
    a_star.setSearchType(search_type);
}


SearchResult Search::startSearch(ILogger *Logger, const Map &map,
                                 const EnvironmentOptions &options,
                                 const SafeIntervals &safe_intervals)
{
    if (search_type == CN_SP_ST_DIJK || search_type == CN_SP_ST_ASTAR) {
        return a_star.startSearch(Logger, map, options);
    }

    if (search_type == CN_SP_ST_SIPP) {
        return sipp.startSearch(Logger, map, options, safe_intervals);
    }

    sresult.pathfound = false;
    sresult.nodescreated = 0;
    sresult.numberofsteps = 0;
    sresult.time = 0;
    sresult.hppath = nullptr;
    sresult.lppath = nullptr;
    return sresult;
}

