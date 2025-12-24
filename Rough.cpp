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
    for (const auto& r : rects) {
        int plane = r.plane;

        // find the exact tile that was inserted
        CornerStitch* t = findTileContaining(
            planeRoots[plane],
            r.lx + 1,
            r.ly + 1
        );
        deleteTileAndCoalesce(planeRoots[0],t);
    }


    // CornerStitch* poly = findTileContaining(planeRoots[0],30,40);
    
    // if (poly->left())   {poly->left()->setAttr(L_PTR_LEFT); poly->left()->setSpace(0);}
    // if (poly->right())  {poly->right()->setAttr(L_PTR_RIGHT); poly->right()->setSpace(0);}
    // if (poly->top())    {poly->top()->setAttr(L_PTR_TOP); poly->top()->setSpace(0);}
    // if (poly->bottom()) {poly->bottom()->setAttr(L_PTR_BOTTOM); poly->bottom()->setSpace(0);}

    cout << "\n=== PLANE " << 0 << " ===\n"; 
    exportTiles(planeRoots[0], "plane0_route.sam");
    return 0;
