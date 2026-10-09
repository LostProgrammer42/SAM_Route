// Ports_Paths.hpp
// Where a wire can leave a tile (ports) and how it runs inside a tile (paths).
//   Dir, Point, Port        : small geometry types
//   getPorts, getPortsForNet: candidate exit points of one tile / of a whole connected group
//   pathInTile, inferPreferHorizontal, findClosestPoint : Manhattan path pieces within one tile
// Depends on Layers.hpp, Corner_Stitch.hpp and Layout_Connectivity.hpp (electricallyAdjacent).
// The search that chooses between ports lives in Modular_Route.
#pragma once
#include <bits/stdc++.h>
#include "Layout_Connectivity.hpp"
using namespace std;

enum Dir { LEFT=0, RIGHT, UP, DOWN };
struct Point {
    long x, y;
    bool operator==(const Point& other) const {
        return x == other.x &&
               y == other.y;    }
};

struct Port {
    long x, y;
    Dir normal;

    bool operator==(const Port& other) const {
        return x == other.x &&
               y == other.y &&
               normal == other.normal;
    }
};

struct RoutePair {
    long src_x, src_y;
    long dst_x, dst_y;
    unsigned int net;
};

vector<Port> getPorts(
    CornerStitch* t,              // NORMAL plane tile
    CornerStitch* bloatedRoot ,     // BLOATED plane root
    bool multiSample 
) {
    vector<Port> ports;
    vector<Port> result;

    long lx = t->getllx();
    long rx = t->geturx();
    long ly = t->getlly();
    long ry = t->getury();
    
    long cx = (lx + rx) / 2;
    long cy = (ly + ry) / 2;

    auto freeInBloated = [&](float x, float y) {
        CornerStitch* b = findTileContaining(bloatedRoot, x, y);
        if (!b) return false;
        if (b->isSpace()) return true;
        return (b->getNet() == t->getNet());  // allow same-net adjacency
    };

    // LEFT port
    if (freeInBloated(lx - 0.1, cy))
        if (!((cy < ly + 0.45*routeWidth(t->getAttr())) || (cy > ry - 0.45*routeWidth(t->getAttr()))))
            ports.push_back({lx, cy, LEFT});

    // RIGHT port
    if (freeInBloated(rx + 0.1, cy))
        if (!((cy < ly + 0.45*routeWidth(t->getAttr())) || (cy > ry - 0.45*routeWidth(t->getAttr()))))    
            ports.push_back({rx, cy, RIGHT});

    // BOTTOM port
    if (freeInBloated(cx, ly - 0.1))
        if (!((cx < lx + 0.45*routeWidth(t->getAttr())) || (cx > rx - 0.45*routeWidth(t->getAttr()))))
            ports.push_back({cx, ly, DOWN});

    // TOP port
    if (freeInBloated(cx, ry + 0.1))
        if (!((cx < lx + 0.45*routeWidth(t->getAttr())) || (cx > rx - 0.45*routeWidth(t->getAttr()))))
            ports.push_back({cx, ry, UP});
    
    if(multiSample){
        long w = rx - lx;
        long h = ry - ly;

        // Conservative sample count
        int NSx = max(1L, min(w / 4, 4L));
        int NSy = max(1L, min(h / 4, 4L));

        // LEFT / RIGHT edges (sample Y)
        for (int i = 1; i <= NSy; i++) {
            long y = ly + (i * h) / (NSy + 1);

            if ((y < ly + 0.5*routeWidth(t->getAttr())) || (y > ry - 0.5*routeWidth(t->getAttr())))
                continue;
            if (freeInBloated(lx - 0.1, y))
                ports.push_back({lx, y, LEFT});
            if (freeInBloated(rx + 0.1, y))
                ports.push_back({rx, y, RIGHT});
        }

        // TOP / BOTTOM edges (sample X)
        for (int i = 1; i <= NSx; i++) {
            long x = lx + (i * w) / (NSx + 1);

            if ((x < lx + 0.5*routeWidth(t->getAttr())) || (x > rx - 0.5*routeWidth(t->getAttr())))
                continue;
            if (freeInBloated(x, ly - 0.1))
                ports.push_back({x, ly, DOWN});
            if (freeInBloated(x, ry + 0.1))
                ports.push_back({x, ry, UP});
        }
    }
    for(auto port : ports) if(port.normal<0 || port.normal > 3) cout << LEFT;
    return ports;
}

vector<Port> getPortsForNet(
    CornerStitch* seedTile,
    CornerStitch* bloatedRoot,
    bool useExtraPorts
) {
    if (!seedTile) return {};

    unordered_set<CornerStitch*> visited;
    deque<CornerStitch*> q;
    vector<Port> ports;

    q.push_back(seedTile);
    visited.insert(seedTile);

    while (!q.empty()) {
        CornerStitch* cur = q.front();
        q.pop_front();

        auto tilePorts = getPorts(cur, bloatedRoot, useExtraPorts);
        ports.insert(ports.end(), tilePorts.begin(), tilePorts.end());

        vector<vector<CornerStitch*>> neighborGroups = {
            rightNeighbors(cur),
            topNeighbors(cur),
            leftNeighbors(cur),
            bottomNeighbors(cur)
        };

        for (auto& group : neighborGroups) {
            for (auto* nb : group) {
                if (!nb) continue;
                if (visited.count(nb)) continue;
                if (electricallyAdjacent(cur, nb)) {
                    visited.insert(nb);
                    q.push_back(nb);
                }
            }
        }
    }

    // --- Collapse ports on the same line into one center port ---
    // LEFT/RIGHT ports share the same x → group by (x, normal)
    // UP/DOWN   ports share the same y → group by (y, normal)

    // Key: (coord, normal) where coord = x for LEFT/RIGHT, y for UP/DOWN
    map<pair<long,int>, vector<Port*>> lineGroups;
    for (auto& p : ports) {
        long key_coord = (p.normal == LEFT || p.normal == RIGHT) ? p.x : p.y;
        lineGroups[{key_coord, p.normal}].push_back(&p);
    }

    vector<Port> result;
    for (auto& [key, group] : lineGroups) {
        if (group.size() == 1) {
            result.push_back(*group[0]);
        } else {
            // Compute center along the perpendicular axis
            if (group[0]->normal == LEFT || group[0]->normal == RIGHT) {
                // Same x, varying y → center y
                long minY = LONG_MAX, maxY = LONG_MIN;
                for (auto* p : group) { minY = min(minY, p->y); maxY = max(maxY, p->y); }
                Port center = *group[0];
                center.y = (minY + maxY) / 2;
                result.push_back(center);
            } else {
                // Same y, varying x → center x
                long minX = LONG_MAX, maxX = LONG_MIN;
                for (auto* p : group) { minX = min(minX, p->x); maxX = max(maxX, p->x); }
                Port center = *group[0];
                center.x = (minX + maxX) / 2;
                result.push_back(center);
            }
        }
    }

    return result;
}

bool inferPreferHorizontal(const vector<vector<Point>>& pathPieces, bool defaultDir)
{
    if (pathPieces.empty()) return defaultDir;

    const auto& last = pathPieces.back();
    if (last.size() < 2) return defaultDir;

    const Point& a = last[last.size() - 2];
    const Point& b = last[last.size() - 1];

    if (a.y == b.y) return true;
    if (a.x == b.x) return false;

    return defaultDir;
}


vector<Point> pathInTile(CornerStitch* t, Point entry, Point exit, bool preferHorizontal)
{
    vector<Point> result;
    if (!t) return result;

    long x1 = entry.x, y1 = entry.y;
    long x2 = exit.x,  y2 = exit.y;

    result.push_back({x1, y1});

    // Trivial cases
    if (x1 == x2 && y1 == y2) return result;
    if (x1 == x2 || y1 == y2) {
        result.push_back({x2, y2});
        return result;
    }

    // Determine possible bend points
    Point hFirst = {x2, y1};  // horizontal → vertical
    Point vFirst = {x1, y2};  // vertical → horizontal

    auto insideTile = [&](Point p) {
        return p.x >= t->getllx() && p.x <= t->geturx() &&
               p.y >= t->getlly() && p.y <= t->getury();
    };

    // Prefer continuing direction if valid
    if (preferHorizontal && insideTile(hFirst)) {
        result.push_back(hFirst);
    }
    else if (!preferHorizontal && insideTile(vFirst)) {
        result.push_back(vFirst);
    }
    else if (insideTile(hFirst)) {
        result.push_back(hFirst);
    }
    else if (insideTile(vFirst)) {
        result.push_back(vFirst);
    }
    else {
        // Fallback: midpoint (should be rare)
        long cx = (x1 + x2) / 2;
        long cy = (y1 + y2) / 2;
        result.push_back({cx, y1});
        result.push_back({cx, y2});
    }

    result.push_back({x2, y2});
    return result;
}

Point findClosestPoint(CornerStitch* t, Point dest){
    if(!t) return dest;
    if(t->containsPointAllEdges(dest.x,dest.y)) return dest;
    if(t->getllx()<=dest.x && dest.x<t->geturx()){
        if(llabs(dest.y-t->getlly()) > llabs(dest.y-t->getury())) return {dest.x,t->getury()};
        else return {dest.x,t->getlly()};
    }
    if(t->getlly()<=dest.y && dest.y<t->getury()){
        if(llabs(dest.x-t->getllx()) > llabs(dest.x-t->geturx())) return {t->geturx(),dest.y};
        else return {t->getllx(),dest.y};
    }
    long x,y;
    if(llabs(dest.y-t->getlly()) > llabs(dest.y-t->getury())) y = t->getury();
    else y = t->getlly();
    if(llabs(dest.x-t->getllx()) > llabs(dest.x-t->geturx())) x = t->geturx();
    else x = t->getllx();
    return {x,y};
}