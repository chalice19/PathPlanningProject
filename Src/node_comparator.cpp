#include "node_comparator.h"

bool NodeComparator::operator()(const Node &left, const Node &right) const {
    if (left.F < right.F) { return true; }
    if (left.F > right.F) { return false; }

    // g-min
    if (left.g < right.g) { return true; }
    if (left.g > right.g) { return false; }

    if (left.i < right.i) { return true; }
    if (left.i > right.i) { return false; }

    if (left.j < right.j) { return true; }
    if (left.j > right.j) { return false; }

    return false;
}

