#ifndef SEARCH_H
#define SEARCH_H
#include "ilogger.h"
#include "searchresult.h"
#include "environmentoptions.h"
#include "node_comparator.h"
#include "A_star.h"
#include "sipp.h"

class Search
{
    public:
        Search();
        ~Search(void);
        SearchResult startSearch(ILogger *Logger, const Map &Map,
                                 const EnvironmentOptions &options,
                                 const SafeIntervals &safe_intervals);

        void setSearchType(int type);

    protected:
        SearchResult    sresult; //This will store the search result in case it is not a_star or sipp
        int             search_type;
        A_star          a_star;
        Sipp            sipp;
};
#endif
