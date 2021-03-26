#ifndef NODE_COMPARATOR_H
#define NODE_COMPARATOR_H
#include "environmentoptions.h"
#include "node.h"

struct NodeComparator {
    bool operator()(const Node &left, const Node &right) const;
};

#endif // NODE_COMPARATOR_H
