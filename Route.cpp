// Author: Stavan Mehta
// Version: Pilot Initial (V0)

#include <bits/stdc++.h>
#include "Corner_Stitch.cpp"
using namespace std;

int layerToPlane(const string& layer) {
    if (layer == "ndiff"  || layer == "pdiff" ||
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
    if (layer == "ndiff" || layer == "pdiff") return 2;
    if (layer == "ntransistor" || layer == "ptransistor") return 0;
    if (layer == "polysilicon") return 3;
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
enum Dir { LEFT, RIGHT, UP, DOWN };
struct Port {
    long x, y;
    Dir normal;
};

vector<Port> getPolyPorts(
    CornerStitch* t,              // NORMAL plane tile
    CornerStitch* bloatedRoot ,     // BLOATED plane root
    bool multiSample 
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
        if (!b) return false;
        if (b->isSpace()) return true;
        return (b->getNet() == t->getNet());  // allow same-net adjacency
    };


    // LEFT port
    if (freeInBloated(lx - 0.1, cy))
        ports.push_back({lx, cy, LEFT});

    // RIGHT port
    if (freeInBloated(rx + 0.1, cy))
        ports.push_back({rx, cy, RIGHT});

    // BOTTOM port
    if (freeInBloated(cx, ly - 0.1))
        ports.push_back({cx, ly, DOWN});

    // TOP port
    if (freeInBloated(cx, ry + 0.1))
        ports.push_back({cx, ry, UP});
    
    if(!multiSample)
        return ports;
    
    long w = rx - lx;
    long h = ry - ly;

    // Conservative sample count
    int NSx = max(1L, min(w / 4, 4L));
    int NSy = max(1L, min(h / 4, 4L));

    // LEFT / RIGHT edges (sample Y)
    for (int i = 1; i <= NSy; i++) {
        long y = ly + (i * h) / (NSy + 1);

        if (freeInBloated(lx - 1, y))
            ports.push_back({lx, y, LEFT});

        if (freeInBloated(rx + 1, y))
            ports.push_back({rx, y, RIGHT});
    }

    // TOP / BOTTOM edges (sample X)
    for (int i = 1; i <= NSx; i++) {
        long x = lx + (i * w) / (NSx + 1);

        if (freeInBloated(x, ly - 1))
            ports.push_back({x, ly, DOWN});

        if (freeInBloated(x, ry + 1))
            ports.push_back({x, ry, UP});
    }

    return ports;
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
        xl = t->getllx();
    }

    // right expand
    while (true) {
        CornerStitch* t = findTileContaining(bloatedRoot, xr + 1, y);
        if (!t || !t->isSpace()) break;
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
        yb = t->getlly();
    }

    while (true) {
        CornerStitch* t = findTileContaining(bloatedRoot, x, yt + 1);
        if (!t || !t->isSpace()) break;
        yt = t->getury();
    }

    return yb < yt;
}

bool tryVHV(
    CornerStitch* bloatedRoot,
    Port s,
    Port t,
    vector<pair<long,long>>& path,
    unsigned int net
) {
    auto isFree = [&](long x, long y) {
        CornerStitch* tile = findTileContaining(bloatedRoot, x, y);
        if (!tile) return false;

        // Space is always free
        if (tile->isSpace()) return true;

        // Same-net obstacles are allowed
        if (tile->getNet() == net) return true;

        return false;
    };

    cout << "Trying Straight Horizontal\n";
    // 1. Straight horizontal
    if (s.y == t.y) {
        if (isFree((s.x + t.x) / 2, s.y)) {
            path = { {s.x, s.y}, {t.x, t.y} };
            return true;
        }
    }

    cout << "Trying Straight Vertical\n";
    // 2. Straight vertical
    if (s.x == t.x) {
        if (isFree(s.x, (s.y + t.y) / 2)) {
            path = { {s.x, s.y}, {t.x, t.y} };
            return true;
        }
    }

    cout << "Trying HV \n";
    // 3. H → V
    if (isFree(t.x, s.y) && isFree(t.x, t.y)) {
        path = {
            {s.x, s.y},
            {t.x, s.y},
            {t.x, t.y}
        };
        return true;
    }

    cout << "Trying VH\n";
    // 4. V → H
    if (isFree(s.x, t.y) && isFree(t.x, t.y)) {
        path = {
            {s.x, s.y},
            {s.x, t.y},
            {t.x, t.y}
        };
        return true;
    }

    cout << "Trying VHV\n";
    // 5. V → H → V (use vertical channel)
    long yb, yt;
    if (!verticalChannel(bloatedRoot, s.x, s.y, yb, yt, 0))
        return false;

    vector<long> ys = {
        (yb + yt) / 2,
        s.y,
        t.y
    };

    for (long y : ys) {
        if (y < yb || y > yt) continue;

        if (isFree(s.x, y) &&
            isFree((s.x + t.x) / 2, y) &&
            isFree(t.x, y)) {

            path = {
                {s.x, s.y},
                {s.x, y},
                {t.x, y},
                {t.x, t.y}
            };
            return true;
        }
    }   

    cout << "Trying HVH\n";
    long xl, xr;
    // 6. H → V → H

    if (horizontalChannel(bloatedRoot, s.y, s.x, xl, xr, 0)) {

        // candidate x-bends
        vector<long> xs = {
            (xl + xr) / 2,
            s.x,
            t.x
        };

        for (long x : xs) {
            if (x < xl || x > xr) continue;

            CornerStitch* a = findTileContaining(bloatedRoot, x, s.y);
            CornerStitch* b = findTileContaining(bloatedRoot, x, t.y);
            CornerStitch* c = findTileContaining(bloatedRoot, t.x, t.y);

            if (!a || !b || !c) continue;
            if (!a->isSpace() || !b->isSpace() || !c->isSpace()) continue;

            path = {
                {s.x, s.y},
                {x, s.y},
                {x, t.y},
                {t.x, t.y}
            };
            return true;
        }
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

void linearizeRoot(std::vector<std::pair<long,long>>& r) {
    if (r.size() <= 2) return;

    std::vector<std::pair<long,long>> out;
    out.push_back(r[0]);

    for (size_t i = 1; i + 1 < r.size(); i++) {
        auto [x0, y0] = out.back();
        auto [x1, y1] = r[i];
        auto [x2, y2] = r[i + 1];

        // collinear in vertical direction
        if (x0 == x1 && x1 == x2) {
            continue;
        }

        // collinear in horizontal direction
        if (y0 == y1 && y1 == y2) {
            continue;
        }

        // real corner
        out.push_back(r[i]);
    }

    out.push_back(r.back());
    r.swap(out);
}

unordered_map<int, vector<RectRec>> rectsByLayer;
unordered_map<unsigned int, vector<CornerStitch*>> polysByNet;

struct RoutePair {
    long src_x, src_y;
    long dst_x, dst_y;
    unsigned int net;
};

bool checkPortAndRouteNormal(const Port& p,
                        const pair<long,long>& next) {
    long dx = next.first  - p.x;
    long dy = next.second - p.y;

    switch (p.normal) {
        case LEFT:  return dx < 0;
        case RIGHT: return dx > 0;
        case UP:    return dy > 0;
        case DOWN:  return dy < 0;
    }
    return false;
}

void rebuildRectsByLayer(
    CornerStitch* root,
    unordered_map<int, vector<RectRec>>& rectsByLayer
) {
    rectsByLayer.clear();

    unordered_set<CornerStitch*> seen;
    deque<CornerStitch*> dq;
    dq.push_back(root);
    seen.insert(root);

    while (!dq.empty()) {
        CornerStitch* t = dq.front(); dq.pop_front();
        if (!t) continue;

        if (!t->isSpace()) {
            long llx = t->getllx();
            long lly = t->getlly();
            long urx = t->geturx();
            long ury = t->getury();

            rectsByLayer[t->getAttr()].push_back({
                0,                              // plane
                attrToLayer(t->getAttr()),      // layer string
                t->getAttr(),
                t->getNet(),
                llx,
                lly,
                urx - llx,
                ury - lly
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
        bool ok1 = insertTileRect(bloatedRoots[plane],lx, ly,wx, wy,attr,netId);

        coalesceAfterDeletion(planeRoots[0],10);
        coalesceAfterDeletion(bloatedRoots[0],10);
        
        if (!ok || !ok1) {
            cout << "Insert failed at line " << lineNo
                 << " (plane " << plane << "): "
                 << line << "\n";
        }
        rectsByLayer[attr].push_back({plane,layer,attr,netId,lx, ly, wx, wy});
    }

    exportTiles(planeRoots[0], "plane0.sam");
    int MAX_ITERS = 8;
    int iter = 0;
    while(true){
        cout << "Iteration: " << iter + 1 << " \n"; 
        if (iter ++ > MAX_ITERS) {cout << "Max Iterations Reached\n"; break;}
        for (auto& [attr, rects] : rectsByLayer) {
            if (attr != L_POLY) continue;

            for (auto& r : rects) {
                CornerStitch* t = findTileContaining(
                    planeRoots[r.plane],
                    r.lx + 1,
                    r.ly + 1
                );
                if (t && !t->isSpace()) {
                    polysByNet[t->getNet()].push_back(t);
                }
            }
        }
        vector<RoutePair> routePairs;
        vector<string> layers = {"ndiff","pdiff","ntransistor","ptransistor","polysilicon","m2"};
        vector<unsigned int> toErase;
        for(auto& [net, polys]: polysByNet){
            if (net == getNetId("#")) continue;
            if(WholeNetElectricallyConnected(polys)){
                cout << "Net: " << net << " fully routed\n";
                toErase.push_back(net);
            }
        }

        for(unsigned int net: toErase){
            polysByNet.erase(net);
        }
        if(polysByNet.size() == 1){break;}

        for (auto& [net, polys] : polysByNet) {
            if (net == getNetId("#")) continue;
            if (polys.size() < 2) continue;

            // ---------- ITERATION 1 : SAME-X (VERTICAL FINGERS) ----------
            if (iter == 1) {
                sort(polys.begin(), polys.end(),
                    [](CornerStitch* a, CornerStitch* b) {
                        if (a->getllx() != b->getllx())
                            return a->getllx() < b->getllx();
                        return a->getlly() < b->getlly();
                    });

                for (size_t i = 0; i + 1 < polys.size(); ) {
                    CornerStitch* a = polys[i];
                    CornerStitch* b = polys[i + 1];

                    if (a->getllx() == b->getllx()) {
                        routePairs.push_back({a->getllx(), a->getlly(), b->getllx(), b->getlly(), net});
                        i += 2;
                    } else {
                        i += 1;
                    }
                }
                continue;
            }

            // ---------- ITERATION 2 : SAME-Y (HORIZONTAL MERGE) ----------
            if (iter == 2) {
                sort(polys.begin(), polys.end(),
                    [](CornerStitch* a, CornerStitch* b) {
                        if (a->getlly() != b->getlly())
                            return a->getlly() < b->getlly();
                        return a->getllx() < b->getllx();
                    });

                vector<bool> used(polys.size(), false);

                for (size_t i = 0; i < polys.size(); i++) {
                    for (size_t j = i + 1; j < polys.size(); j++) {
                        if (polys[i]->getlly() == polys[j]->getlly()) {
                            routePairs.push_back({polys[i]->getllx(), polys[i]->getlly(), polys[j]->getllx(), polys[j]->getlly(), net});
                            break;
                        }
                    }
                }
                continue;
            }

            // ---------- ITERATION == 3 : Normal X-Sorted POly Selection ----------
            // if (iter >= 3 && iter%2 == 1){
            //     sort(polys.begin(), polys.end(), [](CornerStitch* a, CornerStitch* b) {return a->getllx() > b->getllx();});
            //     for (int i = 0; i + 1 < polys.size(); i += 2) {
            //             routePairs.push_back({polys[i]->getllx(), polys[i]->getlly(), polys[i+1]->getllx(), polys[i+1]->getlly(), net});
            //     }
            // }
            // // ---------- ITERATION == 4 : Normal Reverse X-Sorted POly Selection ----------
            // sort(polys.begin(), polys.end(), [](CornerStitch* a, CornerStitch* b) {return a->getllx() < b->getllx();});
            // for (int i = 0; i + 1 < polys.size(); i += 2) {
            //         routePairs.push_back({polys[i]->getllx(), polys[i]->getlly(), polys[i+1]->getllx(), polys[i+1]->getlly(), net});
            // }

            // ---------- ITERATION >= 3 : CENTER-BASED CONNECT ----------
            vector<CornerStitch*> connected;
            vector<CornerStitch*> unconnected = polys;

            // anchor = leftmost poly
            auto anchor = *min_element(unconnected.begin(), unconnected.end(),
                [](CornerStitch* a, CornerStitch* b) {
                    return a->getllx() < b->getllx();
                });

            connected.push_back(anchor);
            unconnected.erase(
                remove(unconnected.begin(), unconnected.end(), anchor),
                unconnected.end()
            );

            while (!unconnected.empty()) {
                CornerStitch* bestSrc = nullptr;
                CornerStitch* bestDst = nullptr;
                long bestCost = LONG_MAX;

                for (auto* s : connected) {
                    for (auto* d : unconnected) {
                        long cx1 = (s->getllx() + s->geturx()) / 2;
                        long cy1 = (s->getlly() + s->getury()) / 2;
                        long cx2 = (d->getllx() + d->geturx()) / 2;
                        long cy2 = (d->getlly() + d->getury()) / 2;

                        long cost = llabs(cx1 - cx2) + llabs(cy1 - cy2);

                        if (cost < bestCost) {
                            bestCost = cost;
                            bestSrc = s;
                            bestDst = d;
                        }
                    }
                }

                if (!bestSrc || !bestDst) break;

                routePairs.push_back({bestSrc->getllx(), bestSrc->getlly(), bestDst->getllx(), bestDst->getlly(), net});
                connected.push_back(bestDst);
                unconnected.erase(
                    remove(unconnected.begin(), unconnected.end(), bestDst),
                    unconnected.end()
                );
            }
        }



        for (int rp = 0; rp < routePairs.size(); rp++) {
            auto& rpair = routePairs[rp];
            CornerStitch* src = findTileContaining(
                planeRoots[0],
                rpair.src_x,
                rpair.src_y
            );

            CornerStitch* dst = findTileContaining(
                planeRoots[0],
                rpair.dst_x,
                rpair.dst_y
            );

            if (electricallyAdjacent(src, dst)) {cout << "Not routing Electrically Adjacent Tiles\n"; continue;}
            unsigned int net_route = rpair.net;
            deleteRectAndCoalesce(bloatedRoots[0],0,0,200,200);
            // Initial bloated plane = exact copy of planeRoots
            for (auto& [attr, rects] : rectsByLayer) {
                for (auto& r : rects) {
                    insertTileRect(
                        bloatedRoots[r.plane],
                        r.lx, r.ly, r.wx, r.wy,
                        r.attr, r.net
                    );
                }
            }
            coalesceAfterDeletion(bloatedRoots[0], 50);
            exportTiles(bloatedRoots[0], "plane0_pre_route_bloated_1.sam");
            //Bloating
            for(const auto layer : layers){
                for (const auto& r : rectsByLayer[layerToAttr(layer)]) {
                    int plane = r.plane;

                    // find the exact tile that was inserted
                    CornerStitch* t = findTileContaining(
                        bloatedRoots[plane],
                        r.lx + 0.1,
                        r.ly + 0.1
                    );
                    CornerStitch* src_in_bloated_plane = findTileContaining(
                        bloatedRoots[plane],
                        rpair.src_x + 0.1,
                        rpair.src_y + 0.1
                    );
                    
                    CornerStitch* dst_in_bloated_plane = findTileContaining(
                        bloatedRoots[plane],
                        rpair.dst_x + 0.1,
                        rpair.dst_y + 0.1
                    );
                    
                    if (!t) continue;
                    if (t == src_in_bloated_plane || t == dst_in_bloated_plane) continue;
                    long b = layerBloat(r.layer);
                    bool ok1 = bloatByRect(t,b,b,b,b); 
                    
                    if (!ok1) {cout << "Bloating Failed\n";}
                }
                coalesceAfterDeletion(bloatedRoots[0],50);

            }
            exportTiles(bloatedRoots[0], "plane0_pre_route_bloated_2.sam");
            

            //Routing

            auto srcPorts = getPolyPorts(src, bloatedRoots[0], iter > 4);
            auto dstPorts = getPolyPorts(dst, bloatedRoots[0], iter > 4);
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
                    if (tryVHV(bloatedRoots[0], sp, dp, trialRoute, net_route)){
                        compressRoute(trialRoute);
                        linearizeRoot(trialRoute);
                        cout << "Route points (" << trialRoute.size() << "):\n";
                        for (int i = 0; i < trialRoute.size(); i++) {
                            cout << "  P" << i << ": (" << trialRoute[i].first << ","<< trialRoute[i].second << ")\n";
                        }
                        if (trialRoute.size() >= 2) {
                            if (!checkPortAndRouteNormal(sp, trialRoute[1]))
                                continue;
                        }

                        if (trialRoute.size() >= 2) {
                            if (!checkPortAndRouteNormal(dp, trialRoute[trialRoute.size()-2]))
                                continue;
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
                linearizeRoot(bestRoute);
                cout << "Best Route points (" << bestRoute.size() << "):\n";
                        for (int i = 0; i < bestRoute.size(); i++) {
                            cout << "  P" << i << ": (" << bestRoute[i].first << ","<< bestRoute[i].second << ")\n";
                        }
                vector<bool> isBend(bestRoute.size(), false);

                for (int i = 1; i + 1 < bestRoute.size(); i++) {
                    auto [x0,y0] = bestRoute[i-1];
                    auto [x1,y1] = bestRoute[i];
                    auto [x2,y2] = bestRoute[i+1];

                    if ((x0 == x1 && y1 == y2) ||
                        (y0 == y1 && x1 == x2)) {
                        isBend[i] = true;
                    }
                }

                for (int i = 1; i < bestRoute.size(); i++) {
                    auto [x0, y0] = bestRoute[i-1];
                    auto [x1, y1] = bestRoute[i];



                    bool trimStart = (i > 1 && isBend[i-1]);
                    bool trimEnd   = (i < bestRoute.size()-1 && isBend[i]);

                    if (x0 == x1) {
                        // vertical segment
                        long ylo = min(y0, y1);
                        long yhi = max(y0, y1);
                        if (y1 < y0){
                            if (trimEnd) ylo += HALF;
                            if (trimStart)  yhi -= HALF;
                        }
                        else{
                            if (trimStart) ylo += HALF;
                            if (trimEnd)  yhi -= HALF;
                        }
                        if (yhi > ylo) {
                            insertTileRect(planeRoots[0],x0 - HALF,ylo,POLY_W,yhi - ylo,L_POLY,net_route);

                        }
                    } else {
                        // horizontal segment
                        long xlo = min(x0, x1);
                        long xhi = max(x0, x1);
                        if (x1 < x0){
                            if (trimEnd) xlo += HALF;
                            if (trimStart   )  xhi -= HALF;
                        }
                        else{
                            if (trimStart) xlo += HALF;
                            if (trimEnd)  xhi -= HALF;
                        }
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
            coalesceAfterDeletion(planeRoots[0]);
            rebuildRectsByLayer(planeRoots[0], rectsByLayer);
            polysByNet.clear();
            exportTiles(planeRoots[0], "plane0_route.sam");

        }
    }
    exportTiles(planeRoots[0], "plane0_route.sam");
    

    return 0;
}