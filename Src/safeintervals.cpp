#include "safeintervals.h"

SafeIntervals::SafeIntervals() {}

SafeIntervals::~SafeIntervals() {}

bool SafeIntervals::compute_safe_intervals(const char *FileName, const Map &map) {
    inf_time = std::numeric_limits<int>::max();

    for (int i = 0; i < map.getMapHeight(); ++i) {
        for (int j = 0; j < map.getMapWidth(); ++j) {
            if (!map.CellIsObstacle(i, j)) {
                std::list<SafeInterval> si(1);
                *(si.begin()) = SafeInterval{.begin = 0, .end = inf_time};
                safe_intervals[map.get_global_index(i, j)] = si;
            }
        }
    }


    std::string value;
    std::stringstream stream;
    tinyxml2::XMLElement *root = nullptr, *dyn_obstacles = nullptr, *element = nullptr, *node = nullptr;

    tinyxml2::XMLDocument doc;
    if (doc.LoadFile(FileName) != tinyxml2::XMLError::XML_SUCCESS) {
        std::cout << "Error opening XML file!" << std::endl;
        return false;
    }

    root = doc.FirstChildElement(CNS_TAG_ROOT);
    if (!root) {
        std::cout << "Error! No '" << CNS_TAG_ROOT << "' element found in XML file!" << std::endl;
        return false;
    }

    dyn_obstacles = root->FirstChildElement(CNS_TAG_DO);
    if (!dyn_obstacles) {
        std::cout << "Warning! No '" << CNS_TAG_DO << "' tag found in XML file!" << std::endl;
        return false;
    }

    element = dyn_obstacles->FirstChildElement(CNS_TAG_NUM);
    stream << element->GetText();
    if (!((stream >> size) && (size > 0))) {
        std::cout << "Error! Invalid value of '" << CNS_TAG_NUM
                  << "' tag encountered (or could not convert to integer)." << std::endl;
        std::cout << "Value of '" << CNS_TAG_NUM << "' tag should be an integer >=0" << std::endl;
        return false;
    }

    element = dyn_obstacles->FirstChildElement(CNS_TAG_OBST);

    for (int i = 0; i < size; ++i) {
        for (node = element->FirstChildElement(CNS_TAG_POINT); node; node = node->NextSiblingElement()) {
            int i = -1, j = -1, t = -1;
            i = node->IntAttribute("y", i);
            j = node->IntAttribute("x", j);
            t = node->IntAttribute("time", t);

            if (!insert_interval(i, j, t, map)) {
                std::cout << "Error! Could not save obstacle's trajectory!" << std::endl;
                return false;
            }
        }

        element = element->NextSiblingElement();
    }
    return true;
}

std::list<SafeInterval>& SafeIntervals::get_intervals(int i, int j, const Map &map) {
    return safe_intervals[map.get_global_index(i, j)];
}

const std::list<SafeInterval>& SafeIntervals::get_intervals(int i, int j, const Map &map) const {
    return safe_intervals.find(map.get_global_index(i, j))->second;
}

const SafeInterval& SafeIntervals::get_interval(int i, int j, int si_i, const Map &map) const {
    const std::list<SafeInterval>& list_of_intervals = get_intervals(i, j, map);
    int index = 0;
    auto interval = list_of_intervals.begin();
    while (interval != list_of_intervals.end() && index < si_i) {
        ++interval;
        ++index;
    }

    if (index == si_i) {
        return *interval;
    }

    std::cout << "Error! The safe interval on (" << j << ", " << i << ") number " << si_i << " is not found." << std::endl;
    return {-1, -1};
}

bool SafeIntervals::insert_interval(int i, int j, int t, const Map &map) {
    std::list<SafeInterval>& list_of_intervals = get_intervals(i, j, map);
    auto interval = list_of_intervals.begin();
    while (interval != list_of_intervals.end() && interval->end < t) {
        ++interval;
    }

    if (t < interval->begin) {
//        std::cout << "Warning! Obstacles collision on (" << j << ", " << i << ") at time " << t << "!" << std::endl;
        return true;
    }

    if (t == interval->begin) {
        interval->begin += 1;
        return true;
    }
    if (t == interval->end) {
        interval->end -= 1;
        return true;
    }

    SafeInterval new_interval = {.begin = interval->begin, .end = t - 1};
    list_of_intervals.insert(interval, new_interval);
    interval->begin = t + 1;

    return true;
}


bool SafeIntervals::is_there_obstacle(int i, int j, int t, const Map &map) const {
    if (map.CellIsObstacle(i, j)) {
        return true;
    }

    const std::list<SafeInterval>& list_of_intervals = get_intervals(i, j, map);
    auto interval = list_of_intervals.begin();
    while (interval != list_of_intervals.end() && interval->end < t) {
        ++interval;
    }

    if (t < interval->begin) {
        return true;
    }

    return false;
}
