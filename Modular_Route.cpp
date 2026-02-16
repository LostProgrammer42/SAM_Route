// Author: Stavan Mehta, Affaan Fakih
// Version: Pilot Initial (V0)

#include <bits/stdc++.h>
#include "Routing_Helpers.cpp"
using namespace std;

unordered_map<int, vector<RectRecord>> rectsByLayer;
unordered_map<unsigned int, vector<CornerStitch*>> polysByNet, metalsByNet, lisByNet, substrateByNet;
unordered_map<int, unordered_set<unsigned int>> fullyRoutedNetsByAttr;

// Load rectangles from .rect file, initialize planeRoots and bloatedRoots and rectsByLayer
void loadRectsFromFile(
    const string &filename,
    CornerStitch* planeRoots[3],
    CornerStitch* bloatedRoots[3],
    unordered_map<int, vector<RectRecord>> &rectsByLayer
) {
    ifstream fin(filename);
    if (!fin) {
        cerr << "Error: cannot open " << filename << "\n";
        return;
    }

    string line;
    int lineNo = 0;
    while (getline(fin, line)) {
        lineNo++;

        string net, layer, inout;
        long lx, ly;
        unsigned long wx, wy;

        if (!parseRectLine(line, inout, net, layer, lx, ly, wx, wy))
            continue;
        ioMap[net] = inout;
        int plane = layerToPlane(layer);
        if (plane < 0) continue;

        unsigned int netId = getNetId(net);
        unsigned int attr = layerToAttr(layer);

        bool ok = insertTileRect(planeRoots[plane], lx, ly, wx, wy, attr, netId);
        bool ok1 = insertTileRect(bloatedRoots[plane], lx, ly, wx, wy, attr, netId);

        if (!ok || !ok1) {
            cout << "Insert failed at line " << lineNo
                 << " (plane " << plane << "): "
                 << line << "\n";
        }

        rectsByLayer[attr].push_back({
            plane,
            layer,
            attr,
            netId,
            lx, ly, wx, wy,
            0
        });
    }
}

// Given a layerByNet map and current iteration number produce route pairs. This function is generic for any layer -> returns vector<RoutePair>.
vector<RoutePair> generateRoutePairs(
    const unordered_map<unsigned int, vector<CornerStitch*>> &layerByNet,
    int iter
) {
    vector<RoutePair> routePairs;

    for (auto &kv : layerByNet) {
        unsigned int net = kv.first;
        const auto &layers = kv.second;
        if (net == getNetId("#")) continue;
        if (layers.size() < 2) continue;
        // ---------- ITERATION 1 : SAME-X (VERTICAL FINGERS) ----------
        if (iter == 1) {
            auto tmp = layers;
            sort(tmp.begin(), tmp.end(),
            [](CornerStitch* a, CornerStitch* b) {
                float cxa = (a->getllx()+a->geturx())*0.5f;
                float cxb = (b->getllx()+b->geturx())*0.5f;
                float cya = (a->getlly()+a->getury())*0.5f;
                float cyb = (b->getlly()+b->getury())*0.5f;
                if (abs(cxa-cxb) > 0.1) 
                    return cxa < cxb;
                return cya < cyb;
            });

            for (size_t i = 0; i < tmp.size(); i++) {
                for (size_t j = i + 1; j < tmp.size(); j++) {
                    float a = (tmp[i]->getllx()+tmp[i]->geturx())*0.5f;
                    float b = (tmp[j]->getllx()+tmp[j]->geturx())*0.5f;
                    if (abs(a-b) < 0.1) {
                        routePairs.push_back({tmp[i]->getllx(), tmp[i]->getlly(), tmp[j]->getllx(), tmp[j]->getlly(), net});
                    }
                }
            }

            continue;
        }
        // ---------- ITERATION 2 : SAME-Y (HORIZONTAL MERGE) ----------
        if (iter == 2) {
            auto tmp = layers;
            sort(tmp.begin(), tmp.end(),
            [](CornerStitch* a, CornerStitch* b) {
                    float cxa = (a->getllx()+a->geturx())*0.5f;
                    float cxb = (b->getllx()+b->geturx())*0.5f;
                    float cya = (a->getlly()+a->getury())*0.5f;
                    float cyb = (b->getlly()+b->getury())*0.5f;
                    if (abs(cya-cyb) > 0.1) 
                        return cya < cyb;
                    return cxa < cxb;
            });

            for (size_t i = 0; i < tmp.size(); i++) {
                    for (size_t j = i + 1; j < tmp.size(); j++) {
                        float a = (tmp[i]->getlly()+tmp[i]->getury())*0.5f;
                        float b = (tmp[j]->getlly()+tmp[j]->getury())*0.5f;
                        if (abs(a-b) < 0.1) {
                            routePairs.push_back({tmp[i]->getllx(), tmp[i]->getlly(), tmp[j]->getllx(), tmp[j]->getlly(), net});
                        }
                    }
            }

            continue;
        }

        // ---------- ITERATION >= 3 : CENTER-BASED CONNECT ----------
        vector<CornerStitch*> connected;
        vector<CornerStitch*> unconnected = layers;

        // anchor = leftmost poly
        auto anchor = *min_element(unconnected.begin(), unconnected.end(),
            [](CornerStitch* a, CornerStitch* b) {
                return a->getllx() < b->getllx();
            });

        connected.push_back(anchor);
        unconnected.erase(remove(unconnected.begin(), unconnected.end(), anchor), unconnected.end());
        struct CandidatePair {
            CornerStitch* s;
            CornerStitch* d;
            float cost;
            CandidatePair(CornerStitch* s_, CornerStitch* d_, float c_): s(s_), d(d_), cost(c_) {}
        };
        vector<CandidatePair> Candidates;

        while (!unconnected.empty()) {
            Candidates.clear();
            CornerStitch* bestSrc = nullptr;
            CornerStitch* bestDst = nullptr;
            long bestCost = LONG_MAX;

            for (auto* s : connected) {
                for (auto* d : unconnected) {
                    if(!s || !d) continue;
                    float cx1 = (s->getllx() + s->geturx())*0.5;
                    float cy1 = (s->getlly() + s->getury())*0.5;
                    float cx2 = (d->getllx() + d->geturx())*0.5;
                    float cy2 = (d->getlly() + d->getury())*0.5;

                    float cost = llabs(cx1 - cx2) + llabs(cy1 - cy2);

                    if (cost < bestCost) {
                        bestCost = cost;
                        bestSrc = s;
                        bestDst = d;
                    }
                    if(iter > 3){
                        Candidates.push_back({s, d, cost});
                    }
                }
            }
            if(iter > 3){
                if (Candidates.empty()) break;
                sort(Candidates.begin(), Candidates.end(), [](const CandidatePair& a, const CandidatePair& b) {
                    return a.cost < b.cost;
                });
                int idx = 0;
                idx = min((int)Candidates.size() - 1, iter - 3); 
                cout << "ig I should try a new route pair, trying idx: " << idx << "\n";
                bestSrc = Candidates[idx].s;
                bestDst = Candidates[idx].d;
            }
            if (!bestSrc || !bestDst) break;

            routePairs.push_back({bestSrc->getllx(), bestSrc->getlly(), bestDst->getllx(), bestDst->getlly(), net});
            connected.push_back(bestDst);
            unconnected.erase(remove(unconnected.begin(), unconnected.end(), bestDst), unconnected.end());
        }
    }
    sort(routePairs.begin(), routePairs.end(),
    [](RoutePair& a, RoutePair& b) {
            long cost_a = llabs(a.src_x-a.dst_x) + llabs(a.src_y-a.dst_y);
            long cost_b = llabs(b.src_x-b.dst_x) + llabs(b.src_y-b.dst_y);                         
            return cost_a < cost_b;
    });

    return routePairs;
}
int x=0;
// attemptRoutePair: route a single RoutePair on the specified plane only
bool attemptRoutePair(
    const RoutePair &rpair,
    int routePlane,
    CornerStitch* planeRoots[3],
    CornerStitch* bloatedRoots[3],
    unordered_map<int, vector<RectRecord>> &rectsByLayer,
    int iter
) {
    // Find src/dst tiles strictly on the routePlane
    CornerStitch* src = findTileContaining(planeRoots[routePlane], rpair.src_x, rpair.src_y);
    CornerStitch* dst = findTileContaining(planeRoots[routePlane], rpair.dst_x, rpair.dst_y);
    if (!src || !dst) return false;
    if (electricallyAdjacent(src, dst)) {
        cout << "Not routing Electrically Adjacent Tiles\n";
        return false;
    }

    // Reset bloated roots and re-insert all rects
    for (int p = 0; p < 3; ++p) {
        deleteRect(bloatedRoots[p], -20, -20, 200, 200);
    }
    for (auto &kv : rectsByLayer) {
        for (auto &r : kv.second) {
            insertTileRect(bloatedRoots[r.plane], r.lx, r.ly, r.wx, r.wy, r.attr, r.net, r.virt);
        }
    }

    
    // Bloating (per-plane)
    exportTiles(bloatedRoots[routePlane],"plane_prebloated.sam");
    vector<string> allLayers = {"ndiff","pdiff","ntransistor","ptransistor","polysilicon","m1"};
    CornerStitch* src_in_bloated_plane = findTileContaining(bloatedRoots[routePlane], (src->getllx())+0.1, (src->getlly())+0.1);
    CornerStitch* dst_in_bloated_plane = findTileContaining(bloatedRoots[routePlane], (dst->getllx())+0.1, (dst->getlly())+0.1);

    for (const auto &layer : allLayers) {
        int a = layerToAttr(layer);
        int plane = layerToPlane(layer);
        unsigned long b = layerBloat(layer);
        auto it = rectsByLayer.find(a);
        if (it != rectsByLayer.end()){
            auto& vec = it->second;
            std::stable_partition(
                vec.begin(), vec.end(),
                [&](const RectRecord& r){
                    // keep these FIRST
                    return (r.net != src->getNet());
                }
            );
        }
        for (const auto &r : rectsByLayer[a]) {
            CornerStitch* t = findTileContaining(bloatedRoots[plane], r.lx + (long)r.wx*0.5, r.ly + (long)r.wy*0.5);
            if (!t) continue;
            if (t == src_in_bloated_plane || t == dst_in_bloated_plane) continue;
            bool ok1 = bloatByRect(t, b, b, b, b);
            if (!ok1) { 
                cout << "Bloating Failed " << plane << "\n"; 
            }
        }
    }
    for (int i = 0; i < 3; ++i) coalesce(bloatedRoots[i],20);
    exportTiles(bloatedRoots[routePlane],"plane_bloated.sam");
    // if(routePlane==1) x++;
    // if(x==7 && routePlane==1){
    //     for (const auto &r : rectsByLayer[L_M1]) {
    //         CornerStitch* t = findTileContaining(bloatedRoots[r.plane], r.lx + (long)r.wx*0.5, r.ly + (long)r.wy*0.5);
    //         if (!t) continue;
    //         if (t == src_in_bloated_plane || t == dst_in_bloated_plane) t->setAttr(L_PTR_RIGHT);
    //         else t->setAttr(L_PTR_LEFT);
    //         exportTiles(bloatedRoots[routePlane],"plane_bloated.sam");
    //     }
    //     assert(false);
    // }

    // Ports: use bloated plane corresponding to routePlane
    bool useExtraPorts = (iter > 2);
    auto srcPorts = getPorts(src, bloatedRoots[routePlane], useExtraPorts);
    auto dstPorts = getPorts(dst, bloatedRoots[routePlane], useExtraPorts);

    cout << "\n=== SRC PORTS === (plane " << routePlane << ")\n";
    for (size_t i = 0; i < srcPorts.size(); ++i) {
        cout << "S" << i << ": (" << srcPorts[i].x << "," << srcPorts[i].y <<")" << " , Normal: " << srcPorts[i].normal<<"\n";
    }
    cout << "\n=== DST PORTS === (plane " << routePlane << ")\n";
    for (size_t i = 0; i < dstPorts.size(); ++i) {
        cout << "D" << i << ": (" << dstPorts[i].x << "," << dstPorts[i].y << ")" << " , Normal: " << dstPorts[i].normal<<"\n";
    }

    if (srcPorts.empty() || dstPorts.empty()) return false;

    vector<pair<Port, Port>> failedRoutes;
    bool routed = false;
    vector<vector<Point>> pathPiecesG;
    vector<unsigned long> routeCostsG;

    while (true) {
        Port bestSrc, bestDst;
        long bestCost = LONG_MAX;
        for (auto s : srcPorts) {
            for (auto d : dstPorts) {
                if (count(failedRoutes.begin(), failedRoutes.end(), make_pair(s, d)) > 0) continue;
                long cost = llabs(s.x - d.x) + llabs(s.y - d.y);
                if (cost < bestCost) {
                    bestCost = cost;
                    bestSrc = s;
                    bestDst = d;
                }
            }
        }

        if (failedRoutes.size() == srcPorts.size() * dstPorts.size()) break;

        CornerStitch* start = src_in_bloated_plane;
        CornerStitch* end = dst_in_bloated_plane;                
        CornerStitch* curS;
        switch(bestSrc.normal){
            case LEFT:  curS = findTileContaining(bloatedRoots[routePlane], bestSrc.x-0.1,bestSrc.y); break;
            case RIGHT: curS = findTileContaining(bloatedRoots[routePlane], bestSrc.x+0.1,bestSrc.y); break;
            case UP:    curS = findTileContaining(bloatedRoots[routePlane], bestSrc.x,bestSrc.y+0.1); break;
            case DOWN:  curS = findTileContaining(bloatedRoots[routePlane], bestSrc.x,bestSrc.y-0.1); break;
            default: break;
        }
        CornerStitch* curD;
        switch(bestDst.normal){
            case LEFT:  curD = findTileContaining(bloatedRoots[routePlane], bestDst.x-0.1,bestDst.y); break;
            case RIGHT: curD = findTileContaining(bloatedRoots[routePlane], bestDst.x+0.1,bestDst.y); break;
            case UP:    curD = findTileContaining(bloatedRoots[routePlane], bestDst.x,bestDst.y+0.1); break;
            case DOWN:  curD = findTileContaining(bloatedRoots[routePlane], bestDst.x,bestDst.y-0.1); break;
            default: break;
        }
        if(!start || !curS || !curD){
            failedRoutes.push_back(make_pair(bestSrc,bestDst));
            continue;
        }

        // ---------- BIDIRECTIONAL ROUTING ATTEMPT ----------
        pathPiecesG.clear();
        routeCostsG.clear();

        // SRC front
        Point curPointS = {bestSrc.x, bestSrc.y};
        vector<vector<Point>> pathPiecesS;
        vector<CornerStitch*> pathTilesS;
        vector<CornerStitch*> visitedTilesS;
        vector<unsigned long> routeCostsS;
        pathTilesS.push_back(curS);
        visitedTilesS.push_back(start);

        // DST front
        Point curPointD = {bestDst.x, bestDst.y};
        vector<vector<Point>> pathPiecesD;
        vector<CornerStitch*> pathTilesD;
        vector<CornerStitch*> visitedTilesD;
        vector<unsigned long> routeCostsD;
        pathTilesD.push_back(curD);
        visitedTilesD.push_back(end);

        int route_ittr = 0;
        bool meet = false;

        auto advanceOneStep = [&](CornerStitch*& cur, Point& curPoint, const Point& targetPoint,
            vector<unsigned long>& routeCosts, vector<vector<Point>>& pathPieces, vector<CornerStitch*>& pathTiles,
            vector<CornerStitch*>& visitedTiles, bool isSrc) -> bool{

            long MAX_TRIES = 1000;
            long i = 0;

            while(cur && i++<1000){
                vector<CornerStitch*> rightTiles  = rightNeighbors(cur);
                vector<CornerStitch*> topTiles    = topNeighbors(cur);
                vector<CornerStitch*> leftTiles   = leftNeighbors(cur);
                vector<CornerStitch*> bottomTiles = bottomNeighbors(cur);
                CornerStitch* next = nullptr;
                CornerStitch* prev = pathTiles.back();
                Point exit;
                long bestCost = LONG_MAX;
                for (auto nbrs : {rightTiles, topTiles, leftTiles, bottomTiles}) {
                    for (auto nbr : nbrs) {
                        if(nbr == pathTiles.back() && nbr == visitedTiles.back()) continue;
                        if(count(visitedTiles.begin(),visitedTiles.end(),nbr) > 0 && count(pathTiles.begin(),pathTiles.end(),nbr) == 0 ) continue;
                        if(cur->isBloat()){
                            if(pathTiles.size()==2 || (prev && prev->isBloat())){
                                if(nbr->isBloat()) continue;
                                else if(!(nbr->isSpace() || (nbr->getAttr()==src->getAttr() && nbr->getNet()==src->getNet()))) continue;
                            }
                            else if(!(!nbr->isBloat() && cur->getAttr()==nbr->getAttr() && cur->getNet()==nbr->getNet())) continue;
                        }
                        else if(!(nbr->isSpace() || (nbr->getAttr()==src->getAttr() && nbr->getNet()==src->getNet()))) continue;
                        

                        Point candidate = findClosestPoint(nbr, targetPoint);
                        long cost = llabs(candidate.x - targetPoint.x) +
                                    llabs(candidate.y - targetPoint.y);

                        if (cost < bestCost) {
                            bestCost = cost;
                            next = nbr;
                        }
                    }
                }

                if(!next || bestCost==LONG_MAX){
                    if(pathPieces.empty()) return false;
                    curPoint = pathPieces.back().front();
                    pathPieces.pop_back();
                    routeCosts.pop_back();
                    next = pathTiles.back();
                    pathTiles.pop_back();
                    visitedTiles.pop_back();
                    visitedTiles.push_back(cur);
                    cur = next;
                    continue;
                }
                if(count(pathTiles.begin(),pathTiles.end(),next) > 0){
                    while(cur != next){
                        cur = pathTiles.back();
                        curPoint = pathPieces.back().front();
                        pathTiles.pop_back();
                        pathPieces.pop_back();
                        routeCosts.pop_back();
                    }
                    continue;
                }
                if(next->getNet() == start->getNet()){
                    CornerStitch* origin = isSrc ? start : end;
                    if(electricallyAdjacent(next,origin)){
                        cur = next;
                        curPoint = findClosestPoint(next,targetPoint);
                        routeCosts.clear(); pathTiles.clear(); pathPieces.clear();
                        pathTiles.push_back(cur);
                        visitedTiles.push_back(cur);
                    }
                }
                
                exit = findClosestPoint(next, curPoint); 
                bool dir = inferPreferHorizontal(pathPieces, true);
                pathPieces.push_back(pathInTile(cur, curPoint, exit, dir));
                pathTiles.push_back(cur);
                visitedTiles.push_back(cur);
                unsigned long pathCost = llabs(curPoint.x - exit.x) + llabs(curPoint.y - exit.y);
                routeCosts.push_back(pathCost);

                cur = next;
                curPoint = exit;
                return true;
            }
            return false;
        };

        // -------- alternating SRC → DST → SRC → DST --------
        while (curS && curD && route_ittr++ < 50) {

            // SRC advances toward DST
            if (!advanceOneStep(curS, curPointS, curPointD, routeCostsS,
                                pathPiecesS, pathTilesS, visitedTilesS, true))
                break;
            if (curS == curD ||
                curS->containsPointAllEdges(curPointD.x, curPointD.y)) {
                meet = true;
                break;
            }

            // DST advances toward SRC
            if (!advanceOneStep(curD, curPointD, curPointS, routeCostsD,
                                pathPiecesD, pathTilesD, visitedTilesD, false))
                break;
            if (curD == curS ||
                curD->containsPointAllEdges(curPointS.x, curPointS.y)) {
                meet = true;
                break;
            }
        }
        // -------- merge paths if meeting occurred --------

        if (meet) {
            bool dir = inferPreferHorizontal(pathPiecesS, true);
            pathPiecesS.push_back(pathInTile(curS, curPointS, curPointD, dir));
            routeCostsS.push_back(llabs(curPointS.x - curPointD.x) + llabs(curPointS.y - curPointD.y));

            reverse(pathPiecesD.begin(), pathPiecesD.end());
            reverse(routeCostsD.begin(), routeCostsD.end());
            for (auto& piece : pathPiecesD) {
                reverse(piece.begin(), piece.end());
                pathPiecesS.push_back(piece);
            }
            for (auto& cost : routeCostsD) {
                routeCostsS.push_back(cost);
            }

            pathPiecesG = pathPiecesS;
            routeCostsG = routeCostsS;
            routed = true;
        }
        
        if (routed) {
            Point current = {bestSrc.x, bestSrc.y};
            cout << bestSrc.x << "," << bestSrc.y << " " << bestDst.x << "," << bestDst.y << " : Routed \n";
            for (size_t i = 0; i < pathPiecesG.size(); i++) {
                cout << "PathPiece " << i << ": ";
                for (size_t j = 0; j < pathPiecesG[i].size(); j++) {
                    cout << "(" << pathPiecesG[i][j].x << ", " << pathPiecesG[i][j].y << ") ";
                }
                cout << endl;
            }


            for (auto &pathPiece : pathPiecesG) {
                for (auto &point : pathPiece) {
                    if(electricallyAdjacent(src,dst)) return true;
                    if (point.x == current.x && point.y == current.y) continue;
                    else if (point.x == current.x) {
                        long y = point.y > current.y ? current.y : point.y;
                        long w = llabs(point.y - current.y) + 2;
                        bool ok = insertTileRect(planeRoots[routePlane], point.x - 1, y - 1, 2, w, src->getAttr(), src->getNet(), 1);
                        if(!ok) cout << "Insertion failed" << endl;
                    } else {
                        long x = point.x > current.x ? current.x : point.x;
                        long w = llabs(point.x - current.x) + 2;
                        bool ok = insertTileRect(planeRoots[routePlane], x - 1, point.y - 1, w, 2, src->getAttr(), src->getNet(), 1);
                        if(!ok) cout << "Insertion failed" << endl;
                    }
                    current = point;
                }
            }
            return true;
        } 
        else {
            failedRoutes.push_back(make_pair(bestSrc, bestDst));
            cout << "Routing between " << bestSrc.x << "," << bestSrc.y << " and "
                 << bestDst.x << "," << bestDst.y << " : fail\n";
        }
    }

    return false;
}

// Route one layer (layerAttr) for a single iteration. This function will rebuild the per-net lists,
// filter already connected nets and try to route routePairs for that layer.
void routeLayer(
    int layerAttr,
    CornerStitch* planeRoots[3],
    CornerStitch* bloatedRoots[3],
    unordered_map<int, vector<RectRecord>> &rectsByLayer,
    int iter
) {
    // Rebuild layer-by-net for requested layer (tiles on that layer/attr)
    unordered_map<unsigned int, vector<CornerStitch*>> layerByNet;
    rebuildLayerByNet(layerAttr, planeRoots, layerByNet, rectsByLayer);
    
    // Remove nets already recorded as fully routed for this attribute
    if (fullyRoutedNetsByAttr.count(layerAttr)) {
        for (auto netId : fullyRoutedNetsByAttr[layerAttr]) {
            layerByNet.erase(netId);
        }
    }

    // Find nets already fully electrically connected and record them permanently
    unordered_map<unsigned int, vector<unsigned int>> toErase;
    for (auto &kv : layerByNet) {
        unsigned int net = kv.first;
        if (net == getNetId("#")) continue;
        if (WholeNetElectricallyConnected(kv.second)) { 
            cout << "Net: " << net << " fully routed on attr " << layerAttr << "\n";
            fullyRoutedNetsByAttr[layerAttr].insert(net);
            toErase[kv.second[0]->getAttr()].push_back(net);
        }
    }
    for (auto &kv : toErase) {
        for (unsigned int net : kv.second) {
            layerByNet.erase(net);
        }
    }
    bool allTrivial = true;
    for (auto &kv : layerByNet) {
        if (kv.second.size() > 1) {
            allTrivial = false;
            break;
        }
    }
    if (allTrivial) return;

    // Generate route pairs for this layer
    vector<RoutePair> routePairs = generateRoutePairs(layerByNet, iter);

    // Determine which plane corresponds to this attribute and route only on that plane
    int routePlane = layerToPlane(attrToLayer(layerAttr));
    if (routePlane < 0) return;
    
    // For each route pair attempt routing (on routePlane only)
    rebuildRectsByLayer(planeRoots, rectsByLayer);
    for (auto rpair : routePairs) {
        attemptRoutePair(rpair, routePlane, planeRoots, bloatedRoots, rectsByLayer, iter);
        rebuildRectsByLayer(planeRoots, rectsByLayer);
    }
    insertedTileCommit(planeRoots[routePlane]);
    rebuildRectsByLayer(planeRoots, rectsByLayer);
}