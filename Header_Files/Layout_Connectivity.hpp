// Layout_Connectivity.hpp
// "What is connected to what" on the tile planes. No routing decisions live here.
//   electricallyAdjacent, geomTouch  : are two tiles on the same net and joined?
//   WholeNetElectricallyConnected    : is a whole net one connected piece?
//   rebuildRectsByLayer, rebuildLayerByNet : cached views of the planes, per layer and per net
//                                      (RectRecord is the cached record)
// Depends on Layers.hpp and Corner_Stitch.hpp only.
#pragma once
#include <bits/stdc++.h>
#include "Corner_Stitch.hpp"
using namespace std;

// Bounding-box test: true if the two tiles overlap or share an edge or a corner.
// Purely geometric: ignores net, layer, plane and the space flag.
// Only used as a quick first check inside electricallyAdjacent().
inline bool geomTouch(const CornerStitch* a, const CornerStitch* b) {
    long ax0 = a->getllx();
    long ay0 = a->getlly();
    long ax1 = a->geturx();
    long ay1 = a->getury();

    long bx0 = b->getllx();
    long by0 = b->getlly();
    long bx1 = b->geturx();
    long by1 = b->getury();

    // Touch or overlap in X and Y
    bool xTouch = !(ax1 < bx0 || bx1 < ax0);
    bool yTouch = !(ay1 < by0 || by1 < ay0);

    return xTouch && yTouch;
}

// True if tiles a and b are electrically connected: there is a chain of same-net,
// non-space, non-bloat tiles joining them.
//  - Quick answers: either tile is space, or the nets differ -> false;
//    a == b, or the tiles touch (geomTouch) -> true.
//  - Otherwise a BFS from `a` over edge-sharing neighbours within the same plane,
//    never leaving the net. Only the net is compared, NOT the layer.
//  - The BFS changes plane only through contact tiles (polycon, librdrcon, mcon),
//    by following their ContactBrother links (poly<->li, li<->m1).
// a and b should be real (non-bloat) tiles. Each call is a full BFS, so it isn't cheap.
bool electricallyAdjacent(CornerStitch* a, CornerStitch* b) {
    if (!a || !b) return false;
    if (a->isSpace() || b->isSpace()) return false;
    if (a->getNet() != b->getNet()) return false;
    if (a == b) return true;
    if (geomTouch(a, b)) return true;

    unsigned int net = a->getNet();
    unordered_set<CornerStitch*> visited;
    deque<CornerStitch*> q;
    q.push_back(a);
    visited.insert(a);

    while (!q.empty()) {
        CornerStitch* cur = q.front(); q.pop_front();

        // Collect only TRUE geometric neighbors via neighbor functions
        vector<CornerStitch*> neighbors;
        for (auto* t : rightNeighbors(cur))   neighbors.push_back(t);
        for (auto* t : leftNeighbors(cur))    neighbors.push_back(t);
        for (auto* t : topNeighbors(cur))     neighbors.push_back(t);
        for (auto* t : bottomNeighbors(cur))  neighbors.push_back(t);

        // Also follow contact brothers if cur is a contact tile
        if (cur->getAttr() == L_POLYCON ||
            cur->getAttr() == L_LIBRDRCON ||
            cur->getAttr() == L_MCON) {
            if (auto* cb1 = cur->getContactBrother_1()) neighbors.push_back(cb1);
            if (auto* cb2 = cur->getContactBrother_2()) neighbors.push_back(cb2);
        }

        for (auto* t : neighbors) {
            if (!t || t->isSpace() || t->isBloat()) continue;
            if (t->getNet() != net) continue;
            if (t == b) return true;
            if (!visited.count(t)) {
                visited.insert(t);
                q.push_back(t);
            }
        }
    }
    return false;
}

// Snapshot of one filled tile as a plain rectangle (plane, layer, net, extent).
// rectsByLayer is a cache of these, grouped by attr; it goes stale whenever tiles change.
struct RectRecord {
    int plane;
    string layer;
    unsigned int attr;
    unsigned int net;
    long lx, ly;
    unsigned long wx, wy;
    unsigned int virt;
};

// True if every tile in `polys` (the tiles of ONE net on one layer) is electrically
// connected to the rest. Flood-fills from polys[0] using electricallyAdjacent() on each
// candidate pair, so it makes up to n^2 BFS calls.
bool WholeNetElectricallyConnected(const vector<CornerStitch*>& polys) {
    if (polys.empty()) return true;

    unordered_set<CornerStitch*> visited;
    deque<CornerStitch*> q;

    q.push_back(polys[0]);
    visited.insert(polys[0]);

    while (!q.empty()) {
        CornerStitch* cur = q.front();
        q.pop_front();

        for (CornerStitch* other : polys) {
            if (visited.count(other)) continue;
            if (!electricallyAdjacent(cur, other)) continue;

            visited.insert(other);
            q.push_back(other);
        }
    }

    return visited.size() == polys.size();
}

// Clears rectsByLayer and refills it from the tiles currently in the planes: one RectRecord
// per filled tile, grouped by attr. Call it after anything that inserts, deletes or merges
// tiles. A nullptr root is skipped, so passing a partial array drops that plane's records.
void rebuildRectsByLayer(
    CornerStitch* roots[3],
    unordered_map<int, vector<RectRecord>>& rectsByLayer
) {
    rectsByLayer.clear();
    for(int i=0; i<3; i++){
        unordered_set<CornerStitch*> seen;
        deque<CornerStitch*> dq;
        dq.push_back(roots[i]);
        seen.insert(roots[i]);

        while (!dq.empty()) {
            CornerStitch* t = dq.front(); dq.pop_front();
            if (!t) continue;

            if (!t->isSpace()) {
                long llx = t->getllx();
                long lly = t->getlly();
                long urx = t->geturx();
                long ury = t->getury();

                rectsByLayer[t->getAttr()].push_back({
                    i,                              // plane
                    attrToLayerName(t->getAttr()),  // layer string
                    t->getAttr(),
                    t->getNet(),
                    llx,
                    lly,
                    (unsigned long)urx - llx,
                    (unsigned long)ury - lly,
                    t->getVirt()
                });
            }

            CornerStitch* nbrs[4] = {
                t->left(), t->right(), t->bottom(), t->top()
            };
            for (auto nb : nbrs) {
                if (nb && seen.insert(nb).second)
                    dq.push_back(nb);
            }
        }
    }
}

// Groups the tiles of one layer by net. For every record of `Layer` in rectsByLayer it
// looks up the tile now at the record's lower-left corner and appends it to
// layerByNet[that tile's net]. Appends only (callers clear the map first) and skips space.
// rectsByLayer should be fresh: a record since merged into a larger tile can list that tile twice.
void rebuildLayerByNet(
    int Layer,
    CornerStitch* roots[3],
    unordered_map<unsigned int, vector<CornerStitch*>>& layerByNet,
    unordered_map<int, vector<RectRecord>>& rectsByLayer)
{
    for (auto& [attr, rects] : rectsByLayer) {
        if (attr != Layer) continue;

        for (auto& r : rects) {
            CornerStitch* t = findTileContaining(
                roots[r.plane],
                r.lx + 0.1,
                r.ly + 0.1
            );
            if (t && !t->isSpace()) {
                layerByNet[t->getNet()].push_back(t);
            }
        }
    }
}