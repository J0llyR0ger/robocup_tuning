#include "etl/vector.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>

constexpr uint16_t NO_PARENT = 0xFFFF;

struct HeapEntry {
    float f;
    uint16_t index;
};

struct HeapCompare {
    bool operator()(const HeapEntry &a, const HeapEntry &b) const { return a.f > b.f; }
};

template <int NUM_NODES> struct AStarWorkspace {
    std::array<float, NUM_NODES> gScore;
    std::array<uint16_t, NUM_NODES> cameFrom;
    std::array<bool, NUM_NODES> closed;

    void reset() {
        gScore.fill(std::numeric_limits<float>::infinity());
        closed.fill(false);
        // cameFrom intentionally not cleared -- every read of cameFrom[n] is only reachable
        // for nodes where gScore[n] was set to finite this search, which always happens
        // in the same code path that writes cameFrom[n]. No stale reads possible.
    }
};

// ---------- Generic graph requirements ----------
// Any Graph type passed to aStarSearch must provide:
//
//   static constexpr int NUM_NODES;
//   template <typename Fn> void forEachNeighbor(uint16_t node, Fn&& fn) const;
//         fn(uint16_t neighborIndex, float stepCost) called once per traversable neighbor.
//         The graph is fully responsible for deciding what counts as traversable --
//         aStarSearch never asks "is this node blocked."
//
// Any Heuristic must be callable as: float h(uint16_t node)  -- estimated cost to goal.

// MAX_QUEUE_SIZE: capacity of the open-set priority queue. Since a node can be pushed
// more than once before being closed, this can in theory exceed NUM_NODES; NUM_NODES
// is a reasonable and commonly-sufficient default -- size up if your graph tends to
// revisit nodes heavily before settling them.
template <typename Graph, typename Heuristic, int MAX_QUEUE_SIZE = Graph::NUM_NODES>
etl::vector<uint16_t, Graph::NUM_NODES> aStarSearch(const Graph &graph, uint16_t startIdx,
                                                    uint16_t goalIdx, const Heuristic &heuristic);