// Author: Stavan Mehta
// Version: Pilot Initial (V0)

#include <bits/stdc++.h>
#include "Corner_Stitch.cpp"
using namespace std;

int layerToPlane(const string& layer) {
    if (layer == "ndiff" || layer == "pdiff" ||
        layer == "ntransistor" || layer == "ptransistor")
        return 0; // diffusion, devices

    if (layer == "polysilicon")
        return 0    ; // poly

    if (layer == "m2")
        return 2; // metal

    return -1; 
}

unordered_map<string, unsigned int> netMap;
unsigned int nextNetId = 1;

unsigned int getNetId(const string& net) {
    if (netMap.count(net)) return netMap[net];
    netMap[net] = nextNetId;
    return nextNetId++;
}

bool parseRectLine(
    const string& line,
    string& net,
    string& layer,
    long& lx, long& ly,
    long& wx, long& wy
) {
    if (line.empty() || line[0] == '#')
        return false;

    string kind;
    long x1, y1, x2, y2;

    stringstream ss(line);
    ss >> kind;

    if (kind != "rect" && kind != "inrect" && kind != "outrect")
        return false;

    ss >> net >> layer >> x1 >> y1 >> x2 >> y2;
    if (!ss) return false;

    lx = x1;
    ly = y1;
    wx = x2 - x1;
    wy = y2 - y1;

    if (wx <= 0 || wy <= 0)
        return false;

    return true;
}



unsigned int layerToAttr(const string& layer) {
    if (layer == "ndiff")        return L_NDIFF;
    if (layer == "pdiff")        return L_PDIFF;
    if (layer == "ntransistor")  return L_NTRANS;
    if (layer == "ptransistor")  return L_PTRANS;
    if (layer == "polysilicon")  return L_POLY;
    if (layer == "m2")           return L_M2;
    return L_NONE;
}

long layerBloat(const string& layer) {
    if (layer == "ndiff" || layer == "pdiff") return 0;
    if (layer == "ntransistor" || layer == "ptransistor") return 0;
    if (layer == "polysilicon") return 1;
    if (layer == "m2") return 0;
    return 0;
}

struct RectRec {
    int plane;
    string layer;
    unsigned int attr;
    unsigned int net;
    long lx, ly, wx, wy;
};

vector<RectRec> rects;
unordered_map<unsigned int, vector<CornerStitch*>> polyByNet;

struct Port {
    long x, y;
};

vector<Port> getPolyPorts(
    CornerStitch* t,              // NORMAL plane tile
    CornerStitch* bloatedRoot      // BLOATED plane root
) {
    vector<Port> ports;

    long lx = t->getllx();
    long rx = t->geturx();
    long ly = t->getlly();
    long ry = t->getury();
    
    long cx = (lx + rx) / 2;
    long cy = (ly + ry) / 2;

    auto freeInBloated = [&](long x, long y) {
        CornerStitch* b = findTileContaining(bloatedRoot, x, y);
        return b && b->isSpace();
    };

    // LEFT port
    if (freeInBloated(lx - 1, cy))
        ports.push_back({lx, cy});

    // RIGHT port
    if (freeInBloated(rx + 1, cy))
        ports.push_back({rx, cy});

    // BOTTOM port
    if (freeInBloated(cx, ly - 1))
        ports.push_back({cx, ly});

    // TOP port
    if (freeInBloated(cx, ry + 1))
        ports.push_back({cx, ry});

    return ports;
}
bool segmentClear(
    CornerStitch* bloatedRoot,
    long x0, long y0,
    long x1, long y1,
    long halfWidth
) {
    if (x0 == x1) {
        // vertical segment
        long lx = x0 - halfWidth;
        long rx = x0 + halfWidth;
        long ly = min(y0, y1);
        long ry = max(y0, y1);

        auto tiles = tilesInRect(bloatedRoot, lx, ly, rx - lx, ry - ly);
        for (auto t : tiles)
            if (!t->isSpace())
                return false;
    } else {
        // horizontal segment
        long ly = y0 - halfWidth;
        long ry = y0 + halfWidth;
        long lx = min(x0, x1);
        long rx = max(x0, x1);

        auto tiles = tilesInRect(bloatedRoot, lx, ly, rx - lx, ry - ly);
        for (auto t : tiles)
            if (!t->isSpace())
                return false;
    }
    return true;
}

bool horizontalChannel(
    CornerStitch* bloatedRoot,
    long y,
    long xStart,
    long& xl,
    long& xr,
    long halfW
) {
    xl = xStart;
    xr = xStart;

    // left expand
    while (true) {
        CornerStitch* t = findTileContaining(bloatedRoot, xl - 1, y);
        if (!t || !t->isSpace()) break;
        if (!segmentClear(bloatedRoot, xl - 1, y, xl - 1, y, halfW)) break;
        xl = t->getllx();
    }

    // right expand
    while (true) {
        CornerStitch* t = findTileContaining(bloatedRoot, xr + 1, y);
        if (!t || !t->isSpace()) break;
        if (!segmentClear(bloatedRoot, xr + 1, y, xr + 1, y, halfW)) break;
        xr = t->geturx();
    }

    return xl < xr;
}

bool verticalChannel(
    CornerStitch* bloatedRoot,
    long x,
    long yStart,
    long& yb,
    long& yt,
    long halfW
) {
    yb = yStart;
    yt = yStart;

    while (true) {
        CornerStitch* t = findTileContaining(bloatedRoot, x, yb - 1);
        if (!t || !t->isSpace()) break;
        if (!segmentClear(bloatedRoot, x, yb - 1, x, yb - 1, halfW)) break;
        yb = t->getlly();
    }

    while (true) {
        CornerStitch* t = findTileContaining(bloatedRoot, x, yt + 1);
        if (!t || !t->isSpace()) break;
        if (!segmentClear(bloatedRoot, x, yt + 1, x, yt + 1, halfW)) break;
        yt = t->getury();
    }

    return yb < yt;
}

bool tryVHV(
    CornerStitch* bloatedRoot,
    Port s,
    Port t,
    vector<pair<long,long>>& path
) {
    const long HALF = 1;

    long yb, yt;
    if (!verticalChannel(bloatedRoot, s.x, s.y, yb, yt, HALF))
        return false;

    vector<long> ys = {
        (yb + yt) / 2,
        s.y,
        t.y
    };

    for (long ybnd : ys) {
        if (ybnd < yb || ybnd > yt) continue;

        if (!segmentClear(bloatedRoot, s.x, s.y, s.x, ybnd, HALF))
            continue;
        if (!segmentClear(bloatedRoot, s.x, ybnd, t.x, ybnd, HALF))
            continue;
        if (!segmentClear(bloatedRoot, t.x, ybnd, t.x, t.y, HALF))
            continue;

        path = {
            {s.x, s.y},
            {s.x, ybnd},
            {t.x, ybnd},
            {t.x, t.y}
        };
        return true;
    }
    return false;
}

long routeCost(const vector<pair<long,long>>& r) {
    long cost = 0;
    for (int i = 1; i < r.size(); i++) {
        cost += llabs(r[i].first  - r[i-1].first)
              + llabs(r[i].second - r[i-1].second);
    }
    // penalize extra bends
    cost += 50 * (int(r.size()) - 2);
    return cost;
}
void compressRoute(vector<pair<long,long>>& r) {
    vector<pair<long,long>> out;
    for (auto& p : r) {
        if (out.empty() || out.back() != p)
            out.push_back(p);
    }
    r.swap(out);
}

int main(int argc, char** argv){
    if (argc < 2) {
        cerr << "Usage: ./Route <file.rect>\n";
        return 1;
    }

    ifstream fin(argv[1]);
    if (!fin) {
        cerr << "Error: cannot open " << argv[1] << "\n";
        return 1;
    }

    CornerStitch* planeRoots[3];
    CornerStitch* bloatedRoots[3];
    for (int i = 0; i < 3; i++){
        planeRoots[i] = new CornerStitch(0, 0);
        bloatedRoots[i] = new CornerStitch(0, 0);
    }

    string line;
    int lineNo = 0;
    while (getline(fin, line)) {
        lineNo++;

        string net, layer;
        long lx, ly, wx, wy;

        if (!parseRectLine(line, net, layer, lx, ly, wx, wy))
            continue;

        int plane = layerToPlane(layer);
        if (plane < 0) continue; // skip unknown layers

        unsigned int netId = getNetId(net);
        unsigned int attr = layerToAttr(layer);

        bool ok = insertTileRect(planeRoots[plane],lx, ly,wx, wy,attr,netId);
        
        if (!ok) {
            cout << "Insert failed at line " << lineNo
                 << " (plane " << plane << "): "
                 << line << "\n";
        }
        rects.push_back({plane,layer,attr,netId,lx, ly, wx, wy});
    }

    for (auto& r : rects) {
        if (r.attr != L_POLY) continue;

        CornerStitch* t = findTileContaining(
            planeRoots[r.plane],
            r.lx + 1,
            r.ly + 1
        );
        if (t)
            polyByNet[r.net].push_back(t);
    }
    CornerStitch* src;
    CornerStitch* dst;
    long net_route;

    src = findTileContaining(planeRoots[0], 38, 19);
    dst = findTileContaining(planeRoots[0], 30, 28);
    net_route = src->getNet();
    for (const auto& r : rects) {
        int plane = r.plane;

        // find the exact tile that was inserted
        CornerStitch* t = findTileContaining(
            planeRoots[plane],
            r.lx + 1,
            r.ly + 1
        );
        if (!t) continue;
        
        long b = layerBloat(r.layer);

        long bl = 0, br = 0, bb = 0, bt = 0;

        // obstruction check uses FINAL normal layout
        if (!t->left()   || t->left()->isSpace())   bl = b;
        if (!t->right()  || t->right()->isSpace())  br = b;
        if (!t->bottom() || t->bottom()->isSpace()) bb = b;
        if (!t->top()    || t->top()->isSpace())    bt = b;
        if (t == src || t == dst) {bl = 0; br = 0; bb = 0; bt = 0;}
        long bloated_lx = r.lx - bl;
        long bloated_ly = r.ly - bb;
        long bloated_wx = r.wx + bl + br;
        long bloated_wy = r.wy + bb + bt;

        bool ok = insertTileRect(bloatedRoots[plane],bloated_lx,bloated_ly,bloated_wx,bloated_wy,r.attr,r.net);

        if (!ok) {
            cout << "Bloated insert failed for rect at ("
                << r.lx << "," << r.ly << ")\n";
        }
    }

    
    cout << "\n=== PLANE " << 0 << " ===\n"; 
    exportTiles(planeRoots[0], "plane0.sam");
    cout << "\n=== PLANE " << 1 << " ===\n"; 
    exportTiles(planeRoots[1], "plane1.sam");
    cout << "\n=== PLANE " << 2 << " ===\n"; 
    exportTiles(planeRoots[2], "plane2.sam");   


    cout << "\n=== BLOATED PLANE " << 0 << " ===\n"; 
    exportTiles(bloatedRoots[0], "plane0_bloated.sam");

    cout << "\n=== BLOATED PLANE " << 1 << " ===\n"; 
    exportTiles(bloatedRoots[1], "plane1_bloated.sam");
    
    // Actual Routing
    // ROUTE src to dst
    auto srcPorts = getPolyPorts(src, bloatedRoots[0]);
    auto dstPorts = getPolyPorts(dst, bloatedRoots[0]);
    cout << "\n=== SRC PORTS ===\n";
    for (int i = 0; i < srcPorts.size(); i++) {
        cout << "S" << i << ": (" << srcPorts[i].x
            << "," << srcPorts[i].y << ")\n";
    }

    cout << "\n=== DST PORTS ===\n";
    for (int i = 0; i < dstPorts.size(); i++) {
        cout << "D" << i << ": (" << dstPorts[i].x
            << "," << dstPorts[i].y << ")\n";
    }

    bool routed = false;
    vector<pair<long,long>> bestRoute;
    long bestCost = LONG_MAX;
    int attempt = 0;
    for (auto& sp : srcPorts) {
        for (auto& dp : dstPorts) {
            attempt++;
            cout << "\n--- TRY " << attempt << " ---\n";
            cout << "SRC: (" << sp.x << "," << sp.y << ") ";
            cout << "DST: (" << dp.x << "," << dp.y << ")\n";

            vector<pair<long,long>> trialRoute;
            if (tryVHV(bloatedRoots[0], sp, dp, trialRoute)){
                compressRoute(trialRoute);
                cout << "Route points (" << trialRoute.size() << "):\n";
                for (int i = 0; i < trialRoute.size(); i++) {
                    cout << "  P" << i << ": (" << trialRoute[i].first << ","<< trialRoute[i].second << ")\n";
                }

                long c = routeCost(trialRoute);
                if (c < bestCost) {
                    bestCost = c;
                    bestRoute = trialRoute;
                    routed = true;
                }
            }
        }
    }

    if (!routed) {
        cout << "Routing failed for net " << net_route << "\n";
    }
    else{
        const long POLY_W = 2;
        const long HALF = POLY_W / 2;
        compressRoute(bestRoute);
        cout << "Best Route points (" << bestRoute.size() << "):\n";
                for (int i = 0; i < bestRoute.size(); i++) {
                    cout << "  P" << i << ": (" << bestRoute[i].first << ","<< bestRoute[i].second << ")\n";
                }

        for (int i = 1; i < bestRoute.size(); i++) {
            auto [x0, y0] = bestRoute[i-1];
            auto [x1, y1] = bestRoute[i];

            bool first = (i == 1);
            bool last  = (i == bestRoute.size() - 1);

            if (x0 == x1) {
                // vertical segment
                long ylo = min(y0, y1);
                long yhi = max(y0, y1);

                if (!first) ylo += HALF;
                if (!last)  yhi -= HALF;

                if (yhi > ylo) {
                    insertTileRect(planeRoots[0],x0 - HALF,ylo,POLY_W,yhi - ylo,L_POLY,net_route);
                }
            } else {
                // horizontal segment
                long xlo = min(x0, x1);
                long xhi = max(x0, x1);

                if (!first) xlo += HALF;
                if (!last)  xhi -= HALF;

                if (xhi > xlo) {
                    insertTileRect(planeRoots[0],xlo,y0 - HALF,xhi - xlo,POLY_W,L_POLY,net_route);
                }
            }
        }
        for (int i = 1; i + 1 < bestRoute.size(); i++) {
            auto [x_prev, y_prev] = bestRoute[i-1];
            auto [x, y]           = bestRoute[i];
            auto [x_next, y_next] = bestRoute[i+1];

            // Only if direction changes (i.e., a bend)
            bool turn =
                (x_prev == x && y_next == y) ||
                (y_prev == y && x_next == x);

            if (!turn) continue;

            insertTileRect(planeRoots[0],x - HALF,y - HALF,POLY_W,POLY_W,L_POLY,net_route);
        }

    }
    cout << "BEFORE 1st DELETE\n";
    deleteTileAndCoalesce(planeRoots[0], findTileContaining(planeRoots[0],42,13));
    cout << "BEFORE 2nd DELETE\n";
    deleteTileAndCoalesce(planeRoots[0], findTileContaining(planeRoots[0],32,13));


    // CornerStitch* poly = findTileContaining(planeRoots[0],30,40);
    
    // if (poly->left())   {poly->left()->setAttr(L_PTR_LEFT); poly->left()->setSpace(0);}
    // if (poly->right())  {poly->right()->setAttr(L_PTR_RIGHT); poly->right()->setSpace(0);}
    // if (poly->top())    {poly->top()->setAttr(L_PTR_TOP); poly->top()->setSpace(0);}
    // if (poly->bottom()) {poly->bottom()->setAttr(L_PTR_BOTTOM); poly->bottom()->setSpace(0);}

    cout << "\n=== PLANE " << 0 << " ===\n"; 
    exportTiles(planeRoots[0], "plane0_route.sam");
    return 0;
}