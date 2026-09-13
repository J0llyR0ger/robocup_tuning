#include "lib/a_star.hpp"
#include "etl/priority_queue.h"

template <typename Graph, typename Heuristic, int MAX_QUEUE_SIZE = Graph::NUM_NODES>
etl::vector<uint16_t, Graph::NUM_NODES> aStarSearch(const Graph &graph, uint16_t startIdx,
                                                    uint16_t goalIdx, const Heuristic &heuristic) {

    static AStarWorkspace<Graph::NUM_CELLS> ws;

    ws.reset();

    etl::priority_queue<HeapEntry, MAX_QUEUE_SIZE, etl::vector<HeapEntry, MAX_QUEUE_SIZE>,
                        HeapCompare>
        openSet;

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

    // if (ws.gScore[goalIdx] == std::numeric_limits<float>::infinity())
    //     return -1;

    // Count path length first, since outPath needs to be filled front-to-back
    // but reconstruction walks backward from goal to start.
    int count = 0;

    for (uint16_t cur = goalIdx;; cur = ws.cameFrom[cur]) {
        count++;

        if (cur == startIdx)
            break;
    }

    // if (count > outPath.capacity())
    //     return -1;

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