// Vias.hpp
// Vias and contacts, and the switch routing built on them.
//
// Switch routing is the router's fallback (see Router.hpp): when a direct route on poly or M1
// fails or comes out too long, hop to local interconnect (LI) through a via at each end,
// route on LI, then come back.
//   placeViaOnPoly, attemptSwitchRouting       : poly -> LI hop (a 5x7 poly+LI via block next to a
//                                                transistor-gate extrusion, with polycon/librdrcon)
//   placeViaOnMetal, attemptSwitchRoutingMetal : M1 -> LI hop (may rip up other nets' LI that blocks it)
//   placeInterlayerVias : once M1 is routed, builds the poly-LI-M1 via stacks (polycon, librdrcon, mcon)
//   placeContactSquare  : puts a 5x5 M1 contact square on a diffusion tile (near the top for
//                         pdiff, near the bottom for ndiff)
//
// These functions call back into attemptRoutePair (Router.hpp), which in turn calls the two
// attemptSwitchRouting functions, so the two headers must be used together: include Vias.hpp,
// never Router.hpp alone.
#pragma once
#include <bits/stdc++.h>
#include "Ports_Paths.hpp"
using namespace std;

// Cache of every filled tile as a RectRecord, grouped by layer (attr). Rebuilt by
// rebuildRectsByLayer after tiles change; attemptRoutePair uses it to rebuild the bloated planes.
inline unordered_map<int, vector<RectRecord>> rectsByLayer;

// Nets already known to be fully connected on a layer (attr -> net ids), so routeLayer can
// skip them. placeViaOnMetal erases a net here when it rips up that net's LI, forcing a re-route.
inline unordered_map<int, unordered_set<unsigned int>> fullyRoutedNetsByAttr;

// attemptRoutePair: route a single RoutePair on the specified plane only
// Body of function in Router.hpp
bool attemptRoutePair(
    const RoutePair &rpair,
    int routePlane,
    CornerStitch* planeRoots[3],
    CornerStitch* bloatedRoots[3],
    unordered_map<int, vector<RectRecord>> &rectsByLayer,
    int iter,
    bool try_switch = true,
    bool bloat_src = true,
    bool advance_routepair = false,
    bool attemptedMetalSwitch = false
);

bool placeContactSquare(
    CornerStitch* substrateRoot,
    CornerStitch* squareRoot,
    unsigned int squareAttr,
    int side,
    CornerStitch* substrateTile,
    unsigned int net = 0
) {
    if (!substrateTile || !squareRoot) return false;

    long lx = substrateTile->getllx();
    long rx = substrateTile->geturx();
    long ly = substrateTile->getlly();
    long ry = substrateTile->getury();

    const long SQ = 5;
    const long MIN_MARGIN = 1;

    long tileWidth  = rx - lx;
    long tileHeight = ry - ly;
    long MARGIN = (tileWidth - SQ) / 2;

    long square_lx = lx + MARGIN;
    long square_ly;
    if (side == 0) {
        square_ly = ry - MARGIN - SQ;
    } else {
        square_ly = ly + MARGIN;
    }

    bool ok = insertTileRect(
        squareRoot,
        square_lx,
        square_ly,
        SQ,
        SQ,
        squareAttr,
        substrateTile->getNet()
    );

    if (!ok) return false;

    coalesce(squareRoot, 10);
    return true;
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

    int polyPlane = layerNameToPlane("polysilicon");
    int liPlane   = layerNameToPlane("li");
    
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
                0,
                false,  // don't try switch routing here
                false,
                true
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
    CornerStitch* srcTile = findTileContaining(planeRoots[layerNameToPlane("polysilicon")], sx, sy);
    CornerStitch* dstTile = findTileContaining(planeRoots[layerNameToPlane("polysilicon")], dx, dy);

    if (!srcTile || !dstTile) return false;
    auto Comp_via1 = placeViaOnPoly(
        planeRoots[layerNameToPlane("polysilicon")],
        planeRoots[layerNameToPlane("li")],
        srcTile,bloatedRoots,rectsByLayer,iter
    );
    auto Via1_Dir = Comp_via1.second;
    auto Comp_via2 = placeViaOnPoly(
        planeRoots[layerNameToPlane("polysilicon")],
        planeRoots[layerNameToPlane("li")],
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
        layerNameToPlane("li"),
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
    int polyPlane = layerNameToPlane("polysilicon");
    int liPlane   = layerNameToPlane("li");
    int m1Plane   = layerNameToPlane("m1");
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
        //Re-Bloat
        rebuildRectsByLayer(planeRoots, rectsByLayer);
        for (int p = 0; p < 3; ++p)
                deleteRect(bloatedRoots[p], -20, -20, 200, 200);
        for (auto& kv : rectsByLayer)
            for (auto& r : kv.second)
                insertTileRect(bloatedRoots[r.plane], r.lx, r.ly, r.wx, r.wy, r.attr, r.net, r.virt);

        // Bloating (per-plane)
        vector<string> allLayers = {"ndiff","pdiff","ntransistor","ptransistor","polysilicon","m1", "li"};
        for (const auto &layer : allLayers) {
            int a = layerNameToAttr(layer);
            int plane = layerNameToPlane(layer);
            unsigned long b = layerBloat(layer);
            
            for (const auto &r : rectsByLayer[a]) {
                CornerStitch* t = findTileContaining(bloatedRoots[plane], r.lx + (long)r.wx*0.5, r.ly + (long)r.wy*0.5);
                if (!t) continue;
                bool ok1 = bloatByRect(t, b, b, b, b);
                if (!ok1) { 
                    cout << "Bloating Failed " << plane << "\n"; 
                }
            }
        } 
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
        bool routeOk = attemptRoutePair(extToVia, polyPlane, localRoots, localBloated, rectsByLayer, 0, false, false, true);
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


CornerStitch* placeViaOnMetal(
    CornerStitch* planeRoots[3],
    CornerStitch* srcTile,
    CornerStitch* bloatedRoots[3],
    unordered_map<int, vector<RectRecord>>& rectsByLayer
) {
    int m1Plane = layerNameToPlane("m1");
    int liPlane = layerNameToPlane("li");
 
    const long S = 5, H = 5;
 
    if (!srcTile || srcTile->isSpace()) return nullptr;
 
    unsigned int srcNet = srcTile->getNet();
        // ------------------------------------------------------------------
    // Helper: given any LI-plane tile (may be L_LIBRDRCON or L_LI),
    // return the nearest enclosing L_LI tile for the same net.
    // After coalesce/changeAttrRect a librdrcon is a sub-tile carved out
    // of an L_LI block; we need the surrounding L_LI block for routing.
    // ------------------------------------------------------------------
    auto resolveToLITile = [&](CornerStitch* t) -> CornerStitch* {
        if (!t) return nullptr;
        // Already an L_LI tile — good.
        if (t->getAttr() == L_LI && t->getNet() == srcNet) return t;
        // It's a librdrcon (or something else): BFS outward on the LI plane
        // to find an adjacent L_LI tile belonging to the same net.
        unordered_set<CornerStitch*> seen;
        deque<CornerStitch*> q;
        q.push_back(t);
        seen.insert(t);
        while (!q.empty()) {
            CornerStitch* cur = q.front(); q.pop_front();
            if (cur->getAttr() == L_LI && cur->getNet() == srcNet) return cur;
            vector<CornerStitch*> nbrs;
            for (auto* n : rightNeighbors(cur))  nbrs.push_back(n);
            for (auto* n : leftNeighbors(cur))   nbrs.push_back(n);
            for (auto* n : topNeighbors(cur))    nbrs.push_back(n);
            for (auto* n : bottomNeighbors(cur)) nbrs.push_back(n);
            for (auto* nb : nbrs) {
                if (!nb || seen.count(nb)) continue;
                if (nb->getNet() == srcNet) { seen.insert(nb); q.push_back(nb); }
            }
        }
        return nullptr;  // no L_LI parent found
    };

 
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
                    CornerStitch* raw = findTileContaining(planeRoots[liPlane], cb->getllx() + 0.1, cb->getlly() + 0.1);
                    CornerStitch* liTile = resolveToLITile(raw);
                    if (liTile) return liTile;
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
    // 2. BFS M1 tiles to find a free S×H spot on BOTH the LI and M1
    //    bloated planes.  The via needs a clear column on both layers so
    //    we can insert LI + M1 seed + mcon all at the same coordinate.
    // ------------------------------------------------------------------
    // FIX: check LI *and* M1 bloated planes together so the mcon we
    //      stamp actually lands on a real M1 tile, not floating in space.
    auto freeOnBothLayers = [&](long lx, long ly) -> bool {
        for (auto* t : tilesInRect(bloatedRoots[liPlane], lx, ly, S, H)) {
            if (!t) continue;
            if (!t->isSpace() && t->getNet() != srcNet) return false;
        }
        for (auto* t : tilesInRect(bloatedRoots[m1Plane], lx, ly, S, H)) {
            if (!t) continue;
            // allow same-net M1 (we'll merge into it) or empty space
            if (!t->isSpace() && t->getNet() != srcNet) return false;
        }
        return true;
    };
 
    // Prefer positions that are already *on* the M1 tile (overlap = 0 routing needed)
    auto positionsInTile = [&](CornerStitch* tile) -> vector<pair<long,long>> {
        vector<pair<long,long>> out;
        long llx = tile->getllx(), lly = tile->getlly();
        long urx = tile->geturx(), ury = tile->getury();
        // Inside the M1 tile first (best: no routing gap)
        for (long y = lly; y + H <= ury + 1; y += 2)
            for (long x = llx; x + S <= urx + 1; x += 2)
                out.push_back({x, y});
        // Then small outward shifts (overlap: 1 or 2 units with the M1 tile edge)
        for (long d = 2; d <= 8; d += 2) {
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
            for (auto [px, py] : positionsInTile(cur)) {
                // FIX: use freeOnBothLayers, not just LI
                if (freeOnBothLayers(px, py)) { found_lx = px; found_ly = py; break; }
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
        cout << "placeViaOnMetal: simple search failed for net " << srcNet
             << " — scanning for rippable LI blockers\n";
 
        // Collect all foreign-net virtual LI tiles that are within a
        // search radius of srcTile.  "Virtual" (virt==1) means they were
        // placed by the router and can be removed; fixed device tiles
        // (virt==0) must not be touched.
        const long SEARCH_R = 20;
        long sx0 = srcTile->getllx() - SEARCH_R, sy0 = srcTile->getlly() - SEARCH_R;
        long sw  = (srcTile->geturx() - srcTile->getllx()) + 2*SEARCH_R;
        long sh  = (srcTile->getury() - srcTile->getlly()) + 2*SEARCH_R;
 
        // tiles in the real LI plane (bloated plane doesn't carry virt flag)
        vector<CornerStitch*> liTilesNear = tilesInRect(planeRoots[liPlane], sx0, sy0, sw, sh);
 
        // group rippable tiles by net so we can re-route them afterward
        unordered_map<unsigned int, vector<RectRecord>> rippedByNet;
 
        for (auto* t : liTilesNear) {
            if (!t || t->isSpace()) continue;
            if (t->getNet() == srcNet) continue;
            if (t->getAttr() != L_LI && t->getAttr() != L_LIBRDRCON) continue;
            if (t->getVirt() != 1) continue;   // fixed device — do not touch
 
            unsigned int blockerNet = t->getNet();
            // Record geometry before deletion
            rippedByNet[blockerNet].push_back({
                liPlane,
                "li",
                (unsigned int)t->getAttr(),
                blockerNet,
                t->getllx(), t->getlly(),
                (unsigned long)(t->geturx() - t->getllx()),
                (unsigned long)(t->getury() - t->getlly()),
                1  // virt
            });
        }
 
        if (rippedByNet.empty()) {
            cout << "placeViaOnMetal: no rippable blockers found for net " << srcNet << "\n";
            return nullptr;
        }
        for (auto& [bNet, recs] : rippedByNet) {
            cout << "placeViaOnMetal: ripping " << recs.size()
                 << " LI tiles of net " << bNet << " to unblock net " << srcNet << "\n";
            for (auto& r : recs)
                deleteRect(planeRoots[liPlane], r.lx, r.ly, r.wx, r.wy);
            // Mark this net as needing re-route by clearing it from
            // fullyRoutedNetsByAttr so the outer routing loop picks it up again
            fullyRoutedNetsByAttr[L_LI].erase(bNet);
            fullyRoutedNetsByAttr[L_M1].erase(bNet);
        }
        coalesce(planeRoots[liPlane], 10);
        
        for (int p = 0; p < 3; ++p)
            deleteRect(bloatedRoots[p], -20, -20, 200, 200);
        for (auto& kv : rectsByLayer)
            for (auto& r : kv.second)
                insertTileRect(bloatedRoots[r.plane], r.lx, r.ly, r.wx, r.wy, r.attr, r.net, r.virt);
        rebuildRectsByLayer(planeRoots, rectsByLayer);
        
        return placeViaOnMetal(planeRoots,srcTile,bloatedRoots, rectsByLayer);
    }

    // ------------------------------------------------------------------
    // 3. Insert M1 seed, LI block, then stamp mcon+librdrcon contacts.
    //    FIX: insert M1 first so changeAttrRect(m1Plane,...) has a tile.
    // ------------------------------------------------------------------
 
    // Insert M1 seed at via location so the mcon lands on real M1 metal.
    // If the position already overlaps srcTile the insert may be a no-op
    // or merge — both are fine.
    bool m1SeedOk = insertTileRect(planeRoots[m1Plane], found_lx, found_ly, S, H, L_M1, srcNet);
    if (!m1SeedOk) {
        // Tolerate: position may already be covered by srcTile itself
        CornerStitch* existing = findTileContaining(planeRoots[m1Plane], found_lx + 0.1, found_ly + 0.1);
        if (!existing || existing->isSpace() || existing->getNet() != srcNet) {
            cout << "placeViaOnMetal: M1 seed insertion failed for net " << srcNet << "\n";
            return nullptr;
        }
        cout << "placeViaOnMetal: M1 seed skipped (already covered by same-net tile)\n";
    }
    coalesce(planeRoots[m1Plane], 10);

    // Now connect the M1 seed back to srcTile with a route if they aren't
    // already electrically adjacent (position was outside srcTile boundary).
    {
        CornerStitch* m1At = findTileContaining(planeRoots[m1Plane], found_lx + 0.1, found_ly + 0.1);
        CornerStitch* m1Src = findTileContaining(planeRoots[m1Plane], srcTile->getllx() + 0.1, srcTile->getlly() + 0.1);
        if (m1At && m1Src && !electricallyAdjacent(m1At, m1Src)) {
            RoutePair m1Bridge = {
                srcTile->getllx(), srcTile->getlly(),
                found_lx, found_ly,
                srcNet
            };
            attemptRoutePair(m1Bridge, m1Plane, planeRoots, bloatedRoots, rectsByLayer,
                             0, false, false, true);
        }
    }
 
    bool liOk = insertTileRect(planeRoots[liPlane], found_lx, found_ly, S, H, L_LI, srcNet);
    if (!liOk) {
        cout << "placeViaOnMetal: LI insertion failed for net " << srcNet << "\n";
        // Roll back M1 seed
        deleteRect(planeRoots[m1Plane], found_lx, found_ly, S, H);
        return nullptr;
    }
    coalesce(planeRoots[liPlane], 10);
 
    // FIX: use +1,+2 offsets to match placeInterlayerVias convention
    //      (+1,+1 left the mcon 1 unit short of center in H=5 tiles)
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
    coalesce(planeRoots[0]);
    coalesce(planeRoots[1]);
    coalesce(planeRoots[2]);
 
    rebuildRectsByLayer(planeRoots, rectsByLayer);
    cout << "placeViaOnMetal: via placed at (" << found_lx << "," << found_ly << ") for net " << srcNet << "\n";
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
    int m1Plane = layerNameToPlane("m1");
    int liPlane = layerNameToPlane("li");

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
    cout << srcLI->getllx() << " " << srcLI->getlly() << " "<< srcLI->getAttr() <<endl;
    cout << dstLI->getllx() << " " << dstLI->getlly() << " "<< dstLI->getAttr() <<endl;

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
        0,
        false,   // don't recurse into switch routing again
        false,
        true,
        true
    );
    if (!ok) {
        cout << "SwitchRoutingMetal: LI routing failed for net " << rpair.net << "\n";
        return false;
    }

    cout << "SwitchRoutingMetal: success for net " << rpair.net << "\n";
    return true;
}