#include "etl/priority_queue.h"
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

static const size_t MAX_NODES = 8192;

struct AStarWorkspace {
    std::array<float, MAX_NODES> gScore;
    std::array<uint16_t, MAX_NODES> cameFrom;
    std::array<bool, MAX_NODES> closed;

    void reset() {
        gScore.fill(std::numeric_limits<float>::infinity());
        closed.fill(false);

        // cameFrom intentionally not cleared -- every read of cameFrom[n] is only reachable
        // for nodes where gScore[n] was set to finite this search, which always happens
        // in the same code path that writes cameFrom[n]. No stale reads possible.
    }
};

static AStarWorkspace ws;

static etl::priority_queue<HeapEntry, MAX_NODES, etl::vector<HeapEntry, MAX_NODES>, HeapCompare>
    openSet;

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
                                                    uint16_t goalIdx, const Heuristic &heuristic) {

    static_assert(MAX_QUEUE_SIZE <= MAX_NODES);

    ws.reset();
    openSet.clear();

    ws.gScore[startIdx] = 0.0f;

    ws.cameFrom[startIdx] = NO_PARENT;
    openSet.push({heuristic(startIdx), startIdx});

    while (!openSet.empty()) {
        HeapEntry current = openSet.top();
        openSet.pop();
        uint16_t ci = current.index;

        if (ws.closed[ci])
            continue;

        ws.closed[ci] = true;

        if (ci == goalIdx)
            break;

        graph.forEachNeighbor(ci, [&](uint16_t ni, float stepCost) {
            if (ws.closed[ni])
                return;

            float tentativeG = ws.gScore[ci] + stepCost;
            if (tentativeG < ws.gScore[ni]) {
                ws.gScore[ni] = tentativeG;
                ws.cameFrom[ni] = ci;
                if (!openSet.full()) {
                    openSet.push({tentativeG + heuristic(ni), ni});
                }
            }
        });
    }

    if (ws.gScore[goalIdx] == std::numeric_limits<float>::infinity()) {
        return etl::vector<uint16_t, Graph::NUM_NODES>{}; // no path found
    }

    int count = 0;
    for (uint16_t cur = goalIdx;; cur = ws.cameFrom[cur]) {
        count++;
        if (cur == startIdx)
            break;
    }

    if (static_cast<size_t>(count) > Graph::NUM_NODES) {
        return etl::vector<uint16_t,
                           Graph::NUM_NODES>{}; // shouldn't happen if reachable, but stay safe
    }

    etl::vector<uint16_t, Graph::NUM_NODES> outPath;
    outPath.resize(count);

    int writeIdx = count - 1;
    for (uint16_t cur = goalIdx;; cur = ws.cameFrom[cur]) {
        outPath[writeIdx--] = cur;
        if (cur == startIdx)
            break;
    }

    return outPath;
}