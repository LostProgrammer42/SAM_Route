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
int z=0;

bool attemptSwitchRouting(
    const RoutePair &rpair,
    int iter,
    CornerStitch* planeRoots[],
    CornerStitch* bloatedRoots[],
    unordered_map<int, vector<RectRecord>>& rectsByLayer
);

bool attemptSwitchRoutingMetal(
    const RoutePair &rpair,
    int iter,
    CornerStitch* planeRoots[],
    CornerStitch* bloatedRoots[],
    unordered_map<int, vector<RectRecord>>& rectsByLayer
);

// attemptRoutePair: route a single RoutePair on the specified plane only
bool attemptRoutePair(
    const RoutePair &rpair,
    int routePlane,
    CornerStitch* planeRoots[3],
    CornerStitch* bloatedRoots[3],
    unordered_map<int, vector<RectRecord>> &rectsByLayer,
    int iter,
    bool try_switch = true,
    bool bloat_src = true
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
    vector<string> allLayers = {"ndiff","pdiff","ntransistor","ptransistor","polysilicon","m1", "li"};
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
            if(!bloat_src){
                if (t == src_in_bloated_plane || t == dst_in_bloated_plane) continue;
            }
            bool ok1 = bloatByRect(t, b, b, b, b);
            if (!ok1) { 
                cout << "Bloating Failed " << plane << "\n"; 
            }
        }
    }
    for (int i = 0; i < 3; ++i) coalesce(bloatedRoots[i],20);
    exportTiles(bloatedRoots[routePlane],"plane_bloated.sam");

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
    bool noRoute = false;
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
        if (failedRoutes.size() == srcPorts.size() * dstPorts.size() || bestCost == LONG_MAX) {
            attemptSwitchRoutingMetal(rpair,iter,planeRoots,bloatedRoots,rectsByLayer);
            noRoute = true;
            break;}
        if (count(failedRoutes.begin(), failedRoutes.end(), make_pair(bestSrc, bestDst)) > 0) continue;

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
                if(next->getNet() == start->getNet() && !next->isBloat()){
                    CornerStitch* origin = isSrc ? start : end;
                    // CornerStitch* check = findTileContaining(planeRoots[layerToPlane(attrToLayer(next->getAttr()))],next->getllx()+0.1)
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
            if (!advanceOneStep(curS, curPointS, curPointD, routeCostsS, pathPiecesS, pathTilesS, visitedTilesS, true))
                break;
            if (curS == curD ||
                curS->containsPointAllEdges(curPointD.x, curPointD.y)) {
                meet = true;
                break;
            }

            // DST advances toward SRC
            if (!advanceOneStep(curD, curPointD, curPointS, routeCostsD, pathPiecesD, pathTilesD, visitedTilesD, false))
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
            unsigned long totalCost = 0;
            for (auto &c : routeCostsG) totalCost += c;

            // ideal Manhattan distance between the source/destination tile centers (use rpair coords)
            long ideal = llabs(rpair.src_x - rpair.dst_x) + llabs(rpair.src_y - rpair.dst_y);

            // tuning params: require BOTH an additive and multiplicative exceed to trigger switch
            const unsigned long OVERHEAD_ADD = 12;   // absolute extra allowed
            const double OVERHEAD_FACTOR = 1.2;      // multiplicative factor

            bool excessive = false;
            bool excessive_m = false;

            if (ideal > 0) {
                excessive = (totalCost > ideal + OVERHEAD_ADD) && ((double)totalCost > OVERHEAD_FACTOR * (double)ideal);
                excessive_m = (totalCost > ideal + 2*OVERHEAD_ADD) && ((double)totalCost > 1.5*OVERHEAD_FACTOR * (double)ideal);

            } else {
                excessive = false;
                excessive_m = false;
            }
            if (excessive && try_switch && routePlane == 0) {
                cout << "Route found but cost " << totalCost << " >> ideal " << ideal
                    << " ; shoudld do switch routing for net " << rpair.net << "\n";
                bool ok = attemptSwitchRouting(rpair,iter,planeRoots,bloatedRoots,rectsByLayer);
                if (ok) return true;

            }
            if (excessive_m && try_switch && routePlane == 2) {
                cout << "Route found but cost " << totalCost << " >> ideal " << ideal
                    << " ; shoudld do switch routing for net " << rpair.net << "\n";
                bool ok = attemptSwitchRoutingMetal(rpair,iter,planeRoots,bloatedRoots,rectsByLayer);
                if (ok) return true;

            }
            // Resolve src/dst on the REAL plane once, before the insertion loop
            CornerStitch* realSrc = findTileContaining(planeRoots[routePlane], rpair.src_x, rpair.src_y);
            CornerStitch* realDst = findTileContaining(planeRoots[routePlane], rpair.dst_x, rpair.dst_y);

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
                    if (point.x == current.x && point.y == current.y) continue;

                    bool ok = false;
                    if (point.x == current.x) {
                        long y = point.y > current.y ? current.y : point.y;
                        long w = llabs(point.y - current.y) + 2;
                        ok = insertTileRect(planeRoots[routePlane], point.x - 1, y - 1, 2, w, src->getAttr(), src->getNet(), 1);
                        if (!ok) cout << "Insertion failed" << endl;
                    } else {
                        long x = point.x > current.x ? current.x : point.x;
                        long w = llabs(point.x - current.x) + 2;
                        ok = insertTileRect(planeRoots[routePlane], x - 1, point.y - 1, w, 2, src->getAttr(), src->getNet(), 1);
                        if (!ok) cout << "Insertion failed" << endl;
                    }
                    current = point;

                    // Re-resolve on real plane after each insertion (tiles may have merged via coalesce)
                    realSrc = findTileContaining(planeRoots[routePlane], rpair.src_x, rpair.src_y);
                    realDst = findTileContaining(planeRoots[routePlane], rpair.dst_x, rpair.dst_y);
                    if (ok && realSrc && realDst && electricallyAdjacent(realSrc, realDst)) {
                        cout << "Elec - genuinely connected on real plane\n";
                        return true;
                    }
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
    if (!routed && noRoute) {
        // All port pairs failed - net is trapped!
        if (try_switch && routePlane == 1) {  // For M1 plane
            cout << "All routes failed for net " << rpair.net 
                << " on M1 - attempting switch to LI\n";
            // if (ok) return true;
        }
    }
    if (!routed && bloat_src) {
        cout << "Retrying with bloat_src=false for net " << rpair.net << "\n";
        return attemptRoutePair(rpair, routePlane, planeRoots, bloatedRoots, rectsByLayer, iter, try_switch, false);
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


// ViaPlacer: place a 5x5 poly square electrically connected to the given poly tile.
pair<pair<CornerStitch*, CornerStitch*>,int> placeViaOnPoly(
    CornerStitch* PolyplaneRoot, 
    CornerStitch* LiplaneRoot, 
    CornerStitch* polyTile, 
    CornerStitch* bloatedRoots[3],  // NEW
    unordered_map<int, vector<RectRecord>>& rectsByLayer,  // NEW
    int iter,  // NEW
    int AvoidDir = 0
) {
    if (!PolyplaneRoot || !LiplaneRoot || !polyTile) return {{nullptr, nullptr}, 0};

    const int S = 5;
    const int H = 7;
    unsigned int srcNet = polyTile->getNet();
    unsigned int polyAttr = polyTile->getAttr();
    const unsigned int HASH_NET = getNetId("#");

    int polyPlane = layerToPlane("polysilicon");
    int liPlane   = layerToPlane("li");

    // Check if a rect is free in a bloated plane (space or same net)
    auto freeInBloated = [&](CornerStitch* bloatedRoot, long lx, long ly, long wx, long wy, unsigned int net) -> bool {
        auto tiles = tilesInRect(bloatedRoot, lx, ly, wx, wy);
        for (auto t : tiles) {
            if (!t) continue;
            if (!t->isSpace() && t->getNet() != net) return false;
        }
        return true;
    };

    // Try to shift a candidate (lx, ly) in the Li bloated plane to find a free spot
    // Returns true and sets out_lx, out_ly if found
    auto findFreeInLiBloated = [&](long lx, long ly, long& out_lx, long& out_ly) -> bool {
        // Try small shifts in all 4 directions
        vector<pair<long,long>> shifts = {
            {0, 0},
            {0, 2}, {0, -2}, {2, 0}, {-2, 0},
            {0, 4}, {0, -4}, {4, 0}, {-4, 0},
            {2, 2}, {2, -2}, {-2, 2}, {-2, -2}
        };
        for (auto [dx, dy] : shifts) {
            long tx = lx + dx;
            long ty = ly + dy;
            if (freeInBloated(bloatedRoots[liPlane], tx, ty, S, H, srcNet) &&
                freeInBloated(bloatedRoots[polyPlane], tx, ty, S, H, srcNet)) {
                out_lx = tx;
                out_ly = ty;
                return true;
            }
        }
        return false;
    };

    // Collect candidate extrusions
    vector<CornerStitch*> candidates;
    unordered_set<CornerStitch*> seen;
    deque<CornerStitch*> dq;
    dq.push_back(PolyplaneRoot);
    seen.insert(PolyplaneRoot);
    while (!dq.empty()) {
        CornerStitch* t = dq.front(); dq.pop_front();
        if (!t) continue;
        if (t->getAttr() == polyAttr && t->getNet() == HASH_NET)
            candidates.push_back(t);
        CornerStitch* nbrs[4] = { t->left(), t->right(), t->bottom(), t->top() };
        for (auto nb : nbrs)
            if (nb && seen.insert(nb).second) dq.push_back(nb);
    }

    if (candidates.empty()) return {{nullptr, nullptr}, 0};

    for (CornerStitch* ext : candidates) {
        if (!ext) continue;

        int Dir = 0;
        CornerStitch* trans = nullptr;
        if (ext->bottom()->getAttr() == L_NTRANS || ext->bottom()->getAttr() == L_PTRANS) {
            Dir = -1; trans = ext->bottom(); 
        } else if (ext->top()->getAttr() == L_NTRANS || ext->top()->getAttr() == L_PTRANS) {
            Dir = 1;  trans = ext->top();
        }
        // if (Dir != 0){
        //     trans->setNet(srcNet);
        // }
        if (Dir == 0 || Dir == AvoidDir) continue;

        CornerStitch* corresponding_poly = (Dir == 1) ? trans->top() : trans->bottom();
        if (!electricallyAdjacent(corresponding_poly, polyTile)) continue;

        ext->setNet(srcNet);
        trans->setNet(srcNet);
        long ex_llx = ext->getllx();
        long ex_lly = ext->getlly();
        long ex_urx = ext->geturx();
        long ex_ury = ext->getury();

        // Candidate base positions
        vector<pair<long,long>> candPos = {
            { ex_llx,                   ex_ury },
            { ex_llx,                   ex_lly - H },
            { ex_llx - S,               ex_lly },
            { ex_urx,                   ex_lly },
            { ex_llx - (S - (ex_urx - ex_llx + 1)) / 2, ex_ury },
            { ex_llx + ((ex_urx - ex_llx + 1) - S) / 2, ex_lly + ((ex_ury - ex_lly + 1) - H) / 2 }
        };

        for (auto& [base_lx, base_ly] : candPos) {
            long final_lx, final_ly;

            // Find a free shifted position in both bloated planes
            if (!findFreeInLiBloated(base_lx, base_ly, final_lx, final_ly))
                continue;

            // Try inserting in poly plane
            bool polyOk = insertTileRect(PolyplaneRoot, final_lx, final_ly, S, H, L_POLY, srcNet);
            if (!polyOk) continue;
            coalesce(PolyplaneRoot, 10);

            // Try inserting in Li plane
            bool liOk = insertTileRect(LiplaneRoot, final_lx, final_ly, S, H, L_LI, srcNet);
            if (!liOk) {
                // Rollback poly insertion
                deleteRect(PolyplaneRoot, final_lx, final_ly, S, H);
                continue;
            }
            coalesce(LiplaneRoot, 10);

            // Place contacts in center 3x3
            changeAttrRect(PolyplaneRoot, final_lx+1, final_ly+2, 3, 3, L_POLYCON);
            changeAttrRect(LiplaneRoot,   final_lx+1, final_ly+2, 3, 3, L_LIBRDRCON);

            auto lower = findTileContaining(PolyplaneRoot, final_lx+1.1, final_ly+2.1);
            auto upper = findTileContaining(LiplaneRoot,   final_lx+1.1, final_ly+2.1);
            if (!lower || !upper) { cout << "Contact brother setup failed\n"; continue; }

            lower->setContactBrother_1(upper);
            upper->setContactBrother_1(lower);
            trans->setNet(srcNet);

            // Rebuild rectsByLayer after insertions
            rebuildRectsByLayer(new CornerStitch*[3]{PolyplaneRoot, LiplaneRoot, nullptr}, rectsByLayer);

            // Route between poly extrusion and the new poly rect
            RoutePair extToVia = {
                ext->getllx(), ext->getlly(),
                final_lx, final_ly,
                srcNet
            };

            CornerStitch* planeRoots[3] = { PolyplaneRoot, LiplaneRoot, nullptr };
            bool routeOk = attemptRoutePair(
                extToVia,
                polyPlane,
                planeRoots,
                bloatedRoots,
                rectsByLayer,
                iter,
                false  // don't try switch routing here
            );

            if (!routeOk) {
                cout << "Via poly extrusion routing failed, trying next position\n";
                // Rollback
                deleteRect(PolyplaneRoot, final_lx, final_ly, S, H);
                deleteRect(LiplaneRoot,   final_lx, final_ly, S, H);
                continue;
            }

            auto polyViaTile = findTileContaining(PolyplaneRoot, final_lx, final_ly);
            auto liViaTile   = findTileContaining(LiplaneRoot,   final_lx, final_ly);
            return {{polyViaTile, liViaTile}, Dir};
        }
    }

    return {{nullptr, nullptr}, 0};
}

bool attemptSwitchRouting(
    const RoutePair &rpair,
    int iter,
    CornerStitch* planeRoots[],
    CornerStitch* bloatedRoots[],
    unordered_map<int, vector<RectRecord>>& rectsByLayer
) {
    const int SWITCH_THRESHOLD = 0;

    long sx = rpair.src_x, sy = rpair.src_y;
    long dx = rpair.dst_x, dy = rpair.dst_y;

    long manhattanDist = llabs(sx - dx) + llabs(sy - dy);
    if (manhattanDist <= SWITCH_THRESHOLD) return false;

    // Get tiles at src and dst (assumes you already have this function)
    CornerStitch* srcTile = findTileContaining(planeRoots[layerToPlane("polysilicon")], sx, sy);
    CornerStitch* dstTile = findTileContaining(planeRoots[layerToPlane("polysilicon")], dx, dy);

    if (!srcTile || !dstTile) return false;
    auto Comp_via1 = placeViaOnPoly(
        planeRoots[layerToPlane("polysilicon")],
        planeRoots[layerToPlane("li")],
        srcTile,bloatedRoots,rectsByLayer,iter
    );
    auto Via1_Dir = Comp_via1.second;
    auto Comp_via2 = placeViaOnPoly(
        planeRoots[layerToPlane("polysilicon")],
        planeRoots[layerToPlane("li")],
        dstTile, bloatedRoots,rectsByLayer,iter,- Via1_Dir
    );

    auto via1 = Comp_via1.first;
    auto via2 = Comp_via2.first;

    

    if (!via1.first || !via2.first) {
        cout << "Switch: via placement failed\n";
        return false;
    }
    rebuildRectsByLayer(planeRoots, rectsByLayer);

    //
    RoutePair liRoute = {
        via1.second->getllx(), via1.second->getlly(),
        via2.second->getllx(), via2.second->getlly(),
        rpair.net
    };
    //

    bool li_ok = attemptRoutePair(
        liRoute,
        layerToPlane("li"),
        planeRoots,
        bloatedRoots,
        rectsByLayer,
        iter,
        false
    );

    if (!li_ok) {
        cout << "Switch: LI routing failed\n";
        return false;
    }

    cout << "Switch routing succeeded for net " << rpair.net << "\n";
    return true;

}

// For each net present on both poly (plane 0) and M1 (plane 2),
// place a polycon+LI via on a suitable poly tile, then drop an M1
// tile on top of the LI block so M1 routing can connect it.
void placeInterlayerVias(
    CornerStitch* planeRoots[3],
    CornerStitch* bloatedRoots[3],
    unordered_map<int, vector<RectRecord>>& rectsByLayer
) {
    int polyPlane = layerToPlane("polysilicon");
    int liPlane   = layerToPlane("li");
    int m1Plane   = layerToPlane("m1");
    const long S  = 5;
    const long H  = 7;

    unordered_map<unsigned int, vector<CornerStitch*>> localPolyByNet, localM1ByNet;
    rebuildLayerByNet(L_POLY, planeRoots, localPolyByNet, rectsByLayer);
    rebuildLayerByNet(L_M1,   planeRoots, localM1ByNet,   rectsByLayer);

    auto freeOnPlane = [&](CornerStitch* root, long lx, long ly, long wx, long wy, unsigned int net, unsigned int attr ) -> bool {
        auto tiles = tilesInRect(root, lx, ly, wx, wy);
        for (auto t : tiles) {
            if (!t) continue;
            if (!t->isSpace() && !(t->getNet() == net && t->getAttr() == attr)) return false;
        }
        return true;
    };

    auto freeOnAllPlanes = [&](long lx, long ly, unsigned int net) -> bool {
        return freeOnPlane(bloatedRoots[polyPlane], lx, ly, S, H, net, L_POLY) &&
               freeOnPlane(bloatedRoots[liPlane],   lx, ly, S, H, net, L_LI) &&
               freeOnPlane(bloatedRoots[m1Plane],   lx, ly, S, H, net, L_M1);
    };

    vector<pair<long,long>> shifts;
    for (long s = 0; s <= 30; s += 2) {
        if (s == 0) { shifts.push_back({0,0}); continue; }
        shifts.push_back({ 0,  s}); shifts.push_back({ 0, -s});
        shifts.push_back({ s,  0}); shifts.push_back({-s,  0});
        // shifts.push_back({ s,  s}); shifts.push_back({ s, -s});
        // shifts.push_back({-s,  s}); shifts.push_back({-s, -s});
    }

    for (auto& [net, polyTiles] : localPolyByNet) {
        if (net == getNetId("#")) continue;
        if (!localM1ByNet.count(net)) continue;

        cout << "Inter-layer via for net " << net << "\n";

        // === Find extrusion adjacent to transistor, same as placeViaOnPoly ===
        // Collect all #-net poly tiles (extrusions)
        unordered_set<CornerStitch*> seen;
        deque<CornerStitch*> dq;
        dq.push_back(planeRoots[polyPlane]);
        seen.insert(planeRoots[polyPlane]);
        vector<CornerStitch*> candidates;
        while (!dq.empty()) {
            CornerStitch* t = dq.front(); dq.pop_front();
            if (!t) continue;
            if (t->getAttr() == L_POLY && t->getNet() == getNetId("#"))
                candidates.push_back(t);
            CornerStitch* nbrs[4] = { t->left(), t->right(), t->bottom(), t->top() };
            for (auto nb : nbrs)
                if (nb && seen.insert(nb).second) dq.push_back(nb);
        }

        CornerStitch* chosenExt   = nullptr;
        CornerStitch* chosenTrans = nullptr;
        int chosenDir = 0;

        for (CornerStitch* ext : candidates) {
            if (!ext) continue;
            int Dir = 0;
            CornerStitch* trans = nullptr;
            if (ext->bottom() && (ext->bottom()->getAttr() == L_NTRANS || ext->bottom()->getAttr() == L_PTRANS)) {
                Dir = -1; trans = ext->bottom();
            } else if (ext->top() && (ext->top()->getAttr() == L_NTRANS || ext->top()->getAttr() == L_PTRANS)) {
                Dir = 1; trans = ext->top();
            }
            if (Dir == 0) continue;

            // Check the poly on the other side of the transistor belongs to our net
            CornerStitch* corresponding_poly = (Dir == 1) ? trans->top() : trans->bottom();
            if (!corresponding_poly) continue;
            if (!electricallyAdjacent(corresponding_poly, polyTiles[0])) continue;

            chosenExt   = ext;
            chosenTrans = trans;
            chosenDir   = Dir;
            break;
        }

        if (!chosenExt) {
            cout << "  No extrusion found for net " << net << "\n";
            continue;
        }

        // Set net on extrusion and transistor, same as placeViaOnPoly
        chosenExt->setNet(net);
        chosenTrans->setNet(net);

        // Find free position starting near the extrusion
        long base_lx = chosenExt->getllx();
        long base_ly = chosenExt->getlly();
        long found_lx = -1, found_ly = -1;

        for (auto [dx, dy] : shifts) {
            long try_lx = base_lx + dx;
            long try_ly = base_ly + dy;
            if (freeOnAllPlanes(try_lx, try_ly, net)) {
                found_lx = try_lx;
                found_ly = try_ly;
                cout << "  Found free position at (" << found_lx << "," << found_ly
                     << ") shift=(" << dx << "," << dy << ")\n";
                break;
            }
        }

        if (found_lx == -1) {
            cout << "  No free 3-plane position found for net " << net << "\n";
            continue;
        }

        // === Insert poly block ===
        bool polyOk = insertTileRect(planeRoots[polyPlane], found_lx, found_ly, S, H, L_POLY, net);
        if (!polyOk) { cout << "  Poly insertion failed for net " << net << "\n"; continue; }
        coalesce(planeRoots[polyPlane], 10);

        // === Insert LI block ===
        bool liOk = insertTileRect(planeRoots[liPlane], found_lx, found_ly, S, H, L_LI, net);
        if (!liOk) {
            deleteRect(planeRoots[polyPlane], found_lx, found_ly, S, H);
            cout << "  LI insertion failed for net " << net << "\n";
            continue;
        }
        coalesce(planeRoots[liPlane], 10);
         // === Insert M1 seed ===
        bool m1Ok = insertTileRect(planeRoots[m1Plane], found_lx, found_ly, S, H, L_M1, net);
        if (!m1Ok) {
            cout << "  M1 seed insertion failed for net " << net << "\n";
            continue;
        }
        coalesce(planeRoots[m1Plane], 10);


        // === Route from extrusion to via block, same as placeViaOnPoly ===
        RoutePair extToVia = {
            chosenExt->getllx(), chosenExt->getlly(),
            found_lx, found_ly,
            net
        };

        CornerStitch* localRoots[3]   = { planeRoots[polyPlane],   planeRoots[liPlane],   nullptr };
        CornerStitch* localBloated[3] = { bloatedRoots[polyPlane], bloatedRoots[liPlane], nullptr };
        bool routeOk = attemptRoutePair(extToVia, polyPlane, localRoots, localBloated, rectsByLayer, 3, false, false);
        if (!routeOk) cout << "  Poly routing to via failed for net " << net << " (continuing anyway)\n";

       // === Place mcon,polycon and librdrcon in center 3x3 ===
        changeAttrRect(planeRoots[m1Plane], found_lx+1, found_ly+2, 3, 3, L_MCON);
        changeAttrRect(planeRoots[polyPlane], found_lx+1, found_ly+2, 3, 3, L_POLYCON);
        changeAttrRect(planeRoots[liPlane],   found_lx+1, found_ly+2, 3, 3, L_LIBRDRCON);

        // === Set up contact brothers ===
        auto polyc = findTileContaining(planeRoots[polyPlane], found_lx+1.1, found_ly+2.1);
        auto librdr = findTileContaining(planeRoots[liPlane],   found_lx+1.1, found_ly+2.1);
        auto mcon = findTileContaining(planeRoots[m1Plane],   found_lx+1.1, found_ly+2.1);

        if (polyc && librdr && mcon) {
            polyc->setContactBrother_1(librdr);
            librdr->setContactBrother_1(polyc);
            mcon->setContactBrother_2(librdr);
            librdr->setContactBrother_2(mcon);
        } else {
            cout << "  Contact brother setup failed for net " << net << "\n";
        }

        cout << "  Via stack placed at (" << found_lx << "," << found_ly << ") for net " << net << "\n";
    }

    rebuildRectsByLayer(planeRoots, rectsByLayer);
}

// Returns the LI tile placed (or nullptr on failure / already-exposed returns existing librdrcon's parent)
CornerStitch* placeViaOnMetal(
    CornerStitch* planeRoots[3],
    CornerStitch* srcTile,
    CornerStitch* bloatedRoots[3],
    unordered_map<int, vector<RectRecord>>& rectsByLayer
) {
    int m1Plane = layerToPlane("m1");
    int liPlane = layerToPlane("li");

    const long S = 5, H = 5;

    if (!srcTile || srcTile->isSpace()) return nullptr;

    unsigned int srcNet = srcTile->getNet();

    // ------------------------------------------------------------------
    // 1. Already exposed? BFS M1 tiles, find existing librdrcon brother.
    //    Return the LI tile it lives on so caller can route to it.
    // ------------------------------------------------------------------
    {
        unordered_set<CornerStitch*> visited;
        deque<CornerStitch*> bfsQ;
        bfsQ.push_back(srcTile);
        visited.insert(srcTile);
        while (!bfsQ.empty()) {
            CornerStitch* cur = bfsQ.front(); bfsQ.pop_front();
            for (auto* cb : { cur->getContactBrother_1(), cur->getContactBrother_2() }) {
                if (cb && cb->getAttr() == L_LIBRDRCON) {
                    cout << "placeViaOnMetal: net " << srcNet << " already exposed to LI\n";
                    // Return the LI tile containing this contact
                    CornerStitch* liTile = findTileContaining(
                        planeRoots[liPlane], cb->getllx() + 0.1, cb->getlly() + 0.1);
                    return liTile;
                }
            }
            vector<CornerStitch*> nbrs;
            for (auto* t : rightNeighbors(cur))  nbrs.push_back(t);
            for (auto* t : leftNeighbors(cur))   nbrs.push_back(t);
            for (auto* t : topNeighbors(cur))    nbrs.push_back(t);
            for (auto* t : bottomNeighbors(cur)) nbrs.push_back(t);
            for (auto* nb : nbrs) {
                if (!nb || visited.count(nb)) continue;
                if (nb->getNet() == srcNet && !nb->isSpace())
                    { visited.insert(nb); bfsQ.push_back(nb); }
            }
        }
    }

    // ------------------------------------------------------------------
    // 2. BFS M1 tiles to find a free S×H spot on the LI plane.
    // ------------------------------------------------------------------
    auto freeOnLI = [&](long lx, long ly) -> bool {
        for (auto* t : tilesInRect(bloatedRoots[liPlane], lx, ly, S, H)) {
            if (!t) continue;
            if (!t->isSpace() && t->getNet() != srcNet) return false;
        }
        return true;
    };

    auto positionsInTile = [&](CornerStitch* tile) -> vector<pair<long,long>> {
        vector<pair<long,long>> out;
        long llx = tile->getllx(), lly = tile->getlly();
        long urx = tile->geturx(), ury = tile->getury();
        for (long y = lly; y + H <= ury + 1; y += 2)
            for (long x = llx; x + S <= urx + 1; x += 2)
                out.push_back({x, y});
        for (long d = 2; d <= 6; d += 2) {
            out.push_back({llx - d, lly});
            out.push_back({llx,     lly - d});
            out.push_back({urx - S + d, lly});
            out.push_back({llx,     ury - H + d});
        }
        return out;
    };

    long found_lx = -1, found_ly = -1;
    {
        float srcCx = (srcTile->getllx() + srcTile->geturx()) * 0.5f;
        float srcCy = (srcTile->getlly() + srcTile->getury()) * 0.5f;
        using Entry = pair<float, CornerStitch*>;
        priority_queue<Entry, vector<Entry>, greater<Entry>> pq;
        unordered_set<CornerStitch*> visited;
        pq.push({0.f, srcTile});
        visited.insert(srcTile);

        while (!pq.empty() && found_lx == -1) {
            auto [dist, cur] = pq.top(); pq.pop();
            for (auto [cx, cy] : positionsInTile(cur)) {
                if (freeOnLI(cx, cy)) { found_lx = cx; found_ly = cy; break; }
            }
            if (found_lx != -1) break;

            vector<CornerStitch*> nbrs;
            for (auto* t : rightNeighbors(cur))  nbrs.push_back(t);
            for (auto* t : leftNeighbors(cur))   nbrs.push_back(t);
            for (auto* t : topNeighbors(cur))    nbrs.push_back(t);
            for (auto* t : bottomNeighbors(cur)) nbrs.push_back(t);
            for (auto* nb : nbrs) {
                if (!nb || visited.count(nb)) continue;
                if (nb->getNet() == srcNet && !nb->isSpace()) {
                    visited.insert(nb);
                    float nx = (nb->getllx() + nb->geturx()) * 0.5f;
                    float ny = (nb->getlly() + nb->getury()) * 0.5f;
                    pq.push({fabsf(nx - srcCx) + fabsf(ny - srcCy), nb});
                }
            }
        }
    }

    if (found_lx == -1) {
        cout << "placeViaOnMetal: no free LI position for net " << srcNet << "\n";
        return nullptr;
    }

    // ------------------------------------------------------------------
    // 3. Insert LI block + contact stack, wire brothers.
    // ------------------------------------------------------------------
    bool liOk = insertTileRect(planeRoots[liPlane], found_lx, found_ly, S, H, L_LI, srcNet);
    if (!liOk) {
        cout << "placeViaOnMetal: LI insertion failed for net " << srcNet << "\n";
        return nullptr;
    }
    coalesce(planeRoots[liPlane], 10);

    long cx = found_lx + 1, cy = found_ly + 1;
    changeAttrRect(planeRoots[liPlane], cx, cy, 3, 3, L_LIBRDRCON);
    changeAttrRect(planeRoots[m1Plane], cx, cy, 3, 3, L_MCON);

    auto librdr = findTileContaining(planeRoots[liPlane], cx + 0.1, cy + 0.1);
    auto mcon   = findTileContaining(planeRoots[m1Plane], cx + 0.1, cy + 0.1);
    if (librdr && mcon) {
        mcon->setContactBrother_2(librdr);
        librdr->setContactBrother_2(mcon);
    } else {
        cout << "placeViaOnMetal: contact brother setup failed for net " << srcNet << "\n";
    }

    rebuildRectsByLayer(planeRoots, rectsByLayer);

    // Return the LI tile (the full L_LI block, not the contact sub-tile)
    return findTileContaining(planeRoots[liPlane], found_lx + 0.1, found_ly + 0.1);
}


// ------------------------------------------------------------------
// Switch routing for M1: place LI vias on both ends, route LI between.
// ------------------------------------------------------------------
bool attemptSwitchRoutingMetal(
    const RoutePair &rpair,
    int iter,
    CornerStitch* planeRoots[],
    CornerStitch* bloatedRoots[],
    unordered_map<int, vector<RectRecord>>& rectsByLayer
) {
    int m1Plane = layerToPlane("m1");
    int liPlane = layerToPlane("li");

    CornerStitch* srcM1 = findTileContaining(planeRoots[m1Plane], rpair.src_x, rpair.src_y);
    CornerStitch* dstM1 = findTileContaining(planeRoots[m1Plane], rpair.dst_x, rpair.dst_y);

    if (!srcM1 || srcM1->isSpace()) {
        cout << "SwitchRoutingMetal: src not on M1 for net " << rpair.net << "\n";
        return false;
    }
    if (!dstM1 || dstM1->isSpace()) {
        cout << "SwitchRoutingMetal: dst not on M1 for net " << rpair.net << "\n";
        return false;
    }

    // Place (or find existing) LI via on src
    CornerStitch* srcLI = placeViaOnMetal(planeRoots, srcM1, bloatedRoots, rectsByLayer);
    if (!srcLI) {
        cout << "SwitchRoutingMetal: src via placement failed for net " << rpair.net << "\n";
        return false;
    }

    // Place (or find existing) LI via on dst
    CornerStitch* dstLI = placeViaOnMetal(planeRoots, dstM1, bloatedRoots, rectsByLayer);
    if (!dstLI) {
        cout << "SwitchRoutingMetal: dst via placement failed for net " << rpair.net << "\n";
        return false;
    }

    // Sanity: if both vias landed on the same LI tile they're already connected
    if (srcLI == dstLI || electricallyAdjacent(srcLI, dstLI)) {
        cout << "SwitchRoutingMetal: vias already adjacent for net " << rpair.net << "\n";
        return true;
    }

    rebuildRectsByLayer(planeRoots, rectsByLayer);

    // Route LI between the two via tiles
    RoutePair liRoute = {
        srcLI->getllx(), srcLI->getlly(),
        dstLI->getllx(), dstLI->getlly(),
        rpair.net
    };

    bool ok = attemptRoutePair(
        liRoute,
        liPlane,
        planeRoots,
        bloatedRoots,
        rectsByLayer,
        iter,
        false   // don't recurse into switch routing again
    );

    if (!ok) {
        cout << "SwitchRoutingMetal: LI routing failed for net " << rpair.net << "\n";
        return false;
    }

    cout << "SwitchRoutingMetal: success for net " << rpair.net << "\n";
    return true;
}