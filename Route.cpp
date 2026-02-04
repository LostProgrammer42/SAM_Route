// Author: Stavan Mehta, Affaan Fakih
// Version: Pilot Initial (V0)

#include <bits/stdc++.h>
#include "Routing_Helpers.cpp"
using namespace std;


unordered_map<int, vector<RectRec>> rectsByLayer;
unordered_map<unsigned int, vector<CornerStitch*>> polysByNet, metalsByNet, lisByNet;
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
        planeRoots[i] = new CornerStitch(-20, -20);
        bloatedRoots[i] = new CornerStitch(-20, -20);
    }
    string line;
    int lineNo = 0;
    while (getline(fin, line)) {
        lineNo++;

        string net, layer, inout;
        long lx, ly, wx, wy;

        if (!parseRectLine(line, inout, net, layer, lx, ly, wx, wy))
            continue;
        ioMap[net] = inout; //Global mapping of nets which are io ports stored in helper cpp file
        int plane = layerToPlane(layer);
        if (plane < 0) continue; // skip unknown layers

        unsigned int netId = getNetId(net);
        unsigned int attr = layerToAttr(layer);

        bool ok = insertTileRect(planeRoots[plane],lx, ly,wx, wy,attr,netId);
        bool ok1 = insertTileRect(bloatedRoots[plane],lx, ly,wx, wy,attr,netId);

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
        lx, ly, wx, wy
        });
    }
    
    cout << "\n=== PLANE " << 0 << " ===\n"; 
    exportTiles(planeRoots[0], "plane0.sam");
    cout << "\n=== PLANE " << 1 << " ===\n"; 
    exportTiles(planeRoots[1], "plane1.sam");
    cout << "\n=== PLANE " << 2 << " ===\n"; 
    exportTiles(planeRoots[2], "plane2.sam"); 
    
    vector<long> routeCosts;
    vector<vector<Point>> pathPieces;
    vector<CornerStitch*> pathTiles;
    vector<string> layers = {"ndiff","pdiff","ntransistor","ptransistor","polysilicon","m2"};
    int MAX_ITERS = 8;
    int iter = 0;
    int noProgressIters = 0;

    while(true){
        cout << "Iteration: " << iter + 1 << " \n"; 
        if (iter ++ > MAX_ITERS) {cout << "Max Iterations Reached\n"; break;}
        polysByNet.clear(); metalsByNet.clear(); lisByNet.clear(); //clear old net lists and rebuild
        rebuildLayerByNet(L_POLY, planeRoots, polysByNet, rectsByLayer);
        rebuildLayerByNet(L_LI, planeRoots, lisByNet, rectsByLayer);
        rebuildLayerByNet(L_M1, planeRoots, metalsByNet, rectsByLayer);
        
        vector<RoutePair> routePairs;
        vector<string> layers = {"ndiff","pdiff","ntransistor","ptransistor","polysilicon","m2"};
        unordered_map<unsigned int, vector<unsigned int>> toErase;
        for(auto& [net, polys]: polysByNet){
            if (net == getNetId("#")) continue;
            if(WholeNetElectricallyConnected(polys)){
                cout << "Net: " << net << " fully routed\n";
                toErase[polys[0]->getAttr()].push_back(net);
            }
        }
        
        for(unsigned int net: toErase[L_POLY]){
            polysByNet.erase(net);
        }
        for(unsigned int net: toErase[L_LI]){
            lisByNet.erase(net);
        }
        for(unsigned int net: toErase[L_M1]){
            metalsByNet.erase(net);
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
                        if(!s || !d) continue;
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
            

            int routePlane = layerToPlane(attrToLayer(src->getAttr()));

            if (electricallyAdjacent(src, dst)) {cout << "Not routing Electrically Adjacent Tiles\n"; continue;}
            unsigned int net_route = rpair.net;
            deleteRectAndCoalesce(bloatedRoots[0],-20,-20,200,200);
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
            exportTiles(bloatedRoots[0], "plane0_pre_route.sam");
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
                for(int i=0; i<3;i++){
                    coalesce(bloatedRoots[i],50);
                }
            }
            exportTiles(bloatedRoots[0], "plane0_pre_route_bloated.sam");

            //Ports
            bool useExtraPorts = (iter > 2);

            auto srcPorts = getPorts(src, bloatedRoots[0], useExtraPorts);
            auto dstPorts = getPorts(dst, bloatedRoots[0], useExtraPorts);
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
            vector<pair<Port,Port>> failedRoutes;
            bool routed = false;
            
            //Routing
            while(true){
                Port bestSrc, bestDst;
                long bestCost = LONG_MAX;
               
                
                for (auto s : srcPorts) {
                    for (auto d : dstPorts) {
                        if(count(failedRoutes.begin(),failedRoutes.end(),make_pair(s,d)) > 0) continue;
                        long cost = llabs(s.x - d.x) + llabs(s.y - d.y);

                        if (cost < bestCost) {
                            bestCost = cost;
                            bestSrc = s;
                            bestDst = d;
                        }
                    }
                }
                

                
                if(failedRoutes.size() == srcPorts.size() *  dstPorts.size()) break;
                CornerStitch* start = findTileContaining(bloatedRoots[routePlane], src->getllx()+0.1, src->getlly()+0.1);
                CornerStitch* cur;
                switch(bestSrc.normal){
                    case LEFT:  cur = findTileContaining(bloatedRoots[routePlane], bestSrc.x-0.1,bestSrc.y); break;
                    case RIGHT: cur = findTileContaining(bloatedRoots[routePlane], bestSrc.x+0.1,bestSrc.y); break;
                    case UP:    cur = findTileContaining(bloatedRoots[routePlane], bestSrc.x,bestSrc.y+0.1); break;
                    case DOWN:  cur = findTileContaining(bloatedRoots[routePlane], bestSrc.x,bestSrc.y-0.1); break;
                    default: break;
                }
                if(!start || !cur){
                    failedRoutes.push_back(make_pair(bestSrc,bestDst));
                    continue;
                }
                
                Point dstPoint = {bestDst.x,bestDst.y};
                Point curPoint = {bestSrc.x,bestSrc.y};

                //Routing attempt for current source destination pair
                pathPieces.clear();
                pathTiles.clear();
                routeCosts.clear();
                pathTiles.push_back(start);
                int route_ittr = 0;

                while(cur && route_ittr++<50){
                    if(cur->containsPoint(dstPoint.x,dstPoint.y)){
                        if(!(cur->getAttr()==dst->getAttr() && cur->getNet()==dst->getNet())){
                            pathTiles.push_back(cur);
                            pathPieces.push_back(pathInTile(cur,curPoint,dstPoint));
                            routeCosts.push_back(llabs(curPoint.x - dstPoint.x) + llabs(curPoint.y - dstPoint.y));
                        }
                        routed = true;
                        break;
                    }

                    vector<CornerStitch*> rightTiles    = rightNeighbors(cur);
                    vector<CornerStitch*> topTiles      = topNeighbors(cur);
                    vector<CornerStitch*> leftTiles     = leftNeighbors(cur);
                    vector<CornerStitch*> bottomTiles   = bottomNeighbors(cur);

                    CornerStitch* next;
                    Point exit;
                    long bestCost = LONG_MAX;
                    for(auto nbrs : {rightTiles,topTiles,leftTiles,bottomTiles}){
                        for(auto nbr : nbrs){
                            if(!(nbr->isSpace() || (nbr->getAttr()==src->getAttr() && nbr->getNet()==src->getNet()))
                                || count(pathTiles.begin(),pathTiles.end(),nbr)>0) continue;
                            Point candidate = findClosestPoint(nbr,dstPoint);
                            long cost = llabs(candidate.x - dstPoint.x) + llabs(candidate.y - dstPoint.y);
                            if(cost < bestCost){
                                bestCost = cost;
                                next = nbr;
                            }
                        }
                    }

                    
                    if(!next || bestCost==LONG_MAX){
                        if(pathPieces.empty()) break;
                        curPoint = pathPieces.back().front();
                        pathPieces.pop_back();
                        routeCosts.pop_back();
                        next = pathTiles.back();
                        pathTiles.pop_back();
                        pathTiles.push_back(cur);
                        cur = next;
                        continue;
                    }

                    exit = findClosestPoint(next,curPoint);
                    long pathCost = llabs(curPoint.x - exit.x) + llabs(curPoint.y - exit.y);
                    pathPieces.push_back(pathInTile(cur,curPoint,exit));
                    routeCosts.push_back(pathCost);
                    pathTiles.push_back(cur);
                    

                    cur = next;
                    curPoint = exit;
                    
                }
                

                if(routed){
                    Point current = {bestSrc.x,bestSrc.y}; 
                    cout << bestSrc.x << "," << bestSrc.y << " " << bestDst.x << "," << bestDst.y << " : Routed \n";
                    for(auto pathPiece : pathPieces){
                        for(auto point : pathPiece){
                            if(point.x==current.x && point.y==current.y) continue;
                            else if(point.x==current.x) {
                                long y = point.y>current.y ? current.y : point.y;
                                long w = llabs(point.y-current.y)+2;
                                insertTileRect(planeRoots[routePlane],point.x-1,y-1,2,w,src->getAttr(),src->getNet(),1);;
                            }
                            else{
                                long x = point.x>current.x ? current.x : point.x;
                                long w = llabs(point.x-current.x)+2;
                                insertTileRect(planeRoots[routePlane],x-1,point.y-1,w,2,src->getAttr(),src->getNet(),1);
                            }
                            current = point;
                        }
                    }
                    rebuildRectsByLayer(planeRoots,rectsByLayer);
                    break;
                }
                else failedRoutes.push_back(make_pair(bestSrc,bestDst));
                cout << "\nRouting between " << bestSrc.x << "," << bestSrc.y << " and " << bestDst.x << "," << bestDst.y << " : " << "fail\n";

            }
        }
        virtualTileCommit(planeRoots[0]);
        rebuildRectsByLayer(planeRoots,rectsByLayer);
    }
    
    
    exportTiles(planeRoots[0], "plane0_routed.sam");
    exportRect(planeRoots,"XNOR2X1_routed.rect");

    return 0;
}