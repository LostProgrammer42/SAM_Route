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

// A legal pad position and how far it is from the anchor.
struct ViaSite { long x, y; long gap; long outside; long cdist; };

// Is the whole pad free on every layer of the stack (bloated planes)? Own net is allowed.
bool padFreeBloated(CornerStitch* bloatedRoots[3], ViaStack stack, long x, long y, unsigned int net) {
    ViaRule rule = viaRule(stack);
    for (unsigned int attr : viaLayers(stack)) {
        CornerStitch* root = bloatedRoots[attrToPlane(attr)];
        for (auto* t : tilesInRect(root, x, y, rule.padW, rule.padH))
            if (t && !t->isSpace() && t->getNet() != net) return false;
    }
    return true;
}

// Candidate pad positions near the anchor rect [alx,aurx) x [aly,aury), nearest first.
// radius = how far (in gap) from the anchor we are willing to look.
vector<ViaSite> findViaSites(CornerStitch* bloatedRoots[3], ViaStack stack, unsigned int net,
                             long alx, long aly, long aurx, long aury,
                             int maxSites = 8, long radius = 12) {
    ViaRule rule = viaRule(stack);
    vector<ViaSite> all;
    for (long x = alx - rule.padW - radius; x <= aurx + radius; ++x)
        for (long y = aly - rule.padH - radius; y <= aury + radius; ++y) {
            long gx = max(0L, max(x - aurx, alx - (x + rule.padW)));
            long gy = max(0L, max(y - aury, aly - (y + rule.padH)));
            if (gx + gy > radius) continue;
            long cx2 = (2*x + rule.padW) - (alx + aurx);   // 2x centre offsets, stays integer
            long cy2 = (2*y + rule.padH) - (aly + aury);
            long ox = max(0L, min(x + rule.padW, aurx) - max(x, alx));
            long oy = max(0L, min(y + rule.padH, aury) - max(y, aly));
            long outside = rule.padW * rule.padH - ox * oy;     // new metal the pad adds beyond the anchor
            all.push_back({x, y, gx + gy, outside, labs(cx2) + labs(cy2)});
        }
    sort(all.begin(), all.end(), [](const ViaSite& a, const ViaSite& b) {
        return tie(a.gap, a.outside, a.cdist, a.y, a.x) < tie(b.gap, b.outside, b.cdist, b.y, b.x);
    });
    vector<ViaSite> out;
    for (auto& s : all) {                       // test lazily, stop at maxSites
        if (padFreeBloated(bloatedRoots, stack, s.x, s.y, net)) out.push_back(s);
        if ((int)out.size() >= maxSites) break;
    }
    return out;
}

// Insert one pad per layer of `stack` at (x,y). All-or-nothing: if a plane refuses, the pads this
// call created are removed again. A plane where the pad overlapped existing geometry is left alone.
bool insertViaPads(CornerStitch* planeRoots[3], long x, long y, unsigned int net, ViaStack stack, vector<pair<int,bool>>* placedOut = nullptr) {
    ViaRule rule = viaRule(stack);
    vector<pair<int,bool>> done;                       // plane, was the pad area empty before?

    for (unsigned int attr : viaLayers(stack)) {
        int plane = attrToPlane(attr);
        CornerStitch*& root = planeRoots[plane];       // reference: coalesce may replace the anchor

        bool fresh = true;
        for (auto* t : tilesInRect(root, x, y, rule.padW, rule.padH))
            if (t && !t->isSpace()) { fresh = false; break; }

        if (!insertTileRect(root, x, y, rule.padW, rule.padH, attr, net)) {
            for (auto [p, wasFresh] : done)
                if (wasFresh) deleteRect(planeRoots[p], x, y, rule.padW, rule.padH);
            return false;
        }
        coalesce(root, 10);
        done.push_back({plane, fresh});
    }
    if (placedOut) *placedOut = done;
    return true;
}

// Undo a successful insertViaPads: remove the pad only on planes that were empty before.
void removeViaPads(CornerStitch* planeRoots[3], long x, long y, ViaStack stack,
                   const vector<pair<int,bool>>& placed) {
    ViaRule rule = viaRule(stack);
    for (auto [plane, fresh] : placed)
        if (fresh) deleteRect(planeRoots[plane], x, y, rule.padW, rule.padH);
}

// Turn the centre of each pad into a contact tile and link the contacts across planes.
// Call after insertViaPads, at whatever point the caller wants the contacts to appear.
bool carveViaContacts(CornerStitch* planeRoots[3], long x, long y, ViaStack stack) {
    ViaRule rule = viaRule(stack);
    vector<unsigned int> layers = viaLayers(stack);
    long cx = x + rule.offX(), cy = y + rule.offY();

    vector<CornerStitch*> contacts;
    for (unsigned int attr : layers) {
        CornerStitch*& root = planeRoots[attrToPlane(attr)];
        if (!changeAttrRect(root, cx, cy, rule.contactSize, rule.contactSize, contactAttrOf(attr)))
            return false;
        contacts.push_back(findTileContaining(root, cx + 0.1, cy + 0.1));
    }
    for (size_t i = 0; i + 1 < layers.size(); ++i) {
        CornerStitch *lo = contacts[i], *hi = contacts[i + 1];
        if (!lo || !hi) return false;
        if (layers[i] == L_POLY) { lo->setContactBrother_1(hi); hi->setContactBrother_1(lo); }
        else                     { lo->setContactBrother_2(hi); hi->setContactBrother_2(lo); }
    }
    return true;
}

// What the via attaches to. `tile` is on `plane`; `claim` are tiles that get relabelled with the
// net while we try (the poly gate tab + its transistor), and restored if nothing works.
struct ViaAnchor {
    int plane;
    CornerStitch* tile;
    vector<CornerStitch*> claim;
    int dir = 0;                 // poly only: which side of the gate the tab is on
};

vector<ViaAnchor> findViaAnchors(CornerStitch* planeRoots[3], int anchorPlane, CornerStitch* tile, int avoidDir)
{
    vector<ViaAnchor> out;
    const int polyPlane = layerNameToPlane("polysilicon");

    // metal: the tile itself is the anchor
    if (anchorPlane != polyPlane) {                 
        out.push_back({ anchorPlane, tile, {}, 0 });
        return out;
    }

    // poly: gate extrusions whose far side is already on this net
    auto isTrans = [](CornerStitch* t) {
        return t && (t->getAttr() == L_NTRANS || t->getAttr() == L_PTRANS);
    };
    const unsigned int HASH_NET = getNetId("#");

    unordered_set<CornerStitch*> seen;
    deque<CornerStitch*> dq;
    dq.push_back(planeRoots[polyPlane]);
    seen.insert(planeRoots[polyPlane]);
    while (!dq.empty()) {
        CornerStitch* ext = dq.front(); dq.pop_front();
        if (!ext) continue;
        CornerStitch* nbrs[4] = { ext->left(), ext->right(), ext->bottom(), ext->top() };
        for (auto nb : nbrs) if (nb && seen.insert(nb).second) dq.push_back(nb);

        if (ext->getAttr() != tile->getAttr() || ext->getNet() != HASH_NET) continue;

        int Dir = 0; CornerStitch* trans = nullptr;
        if (isTrans(ext->bottom()))   { Dir = -1; trans = ext->bottom(); }
        else if (isTrans(ext->top())) { Dir =  1; trans = ext->top(); }
        if (Dir == 0 || Dir == avoidDir) continue;

        CornerStitch* farPoly = (Dir == 1) ? trans->top() : trans->bottom();
        if (!farPoly || !electricallyAdjacent(farPoly, tile)) continue;

        out.push_back({ polyPlane, ext, { ext, trans }, Dir });
    }
    return out;
}

bool placeViaStack(CornerStitch* planeRoots[3], CornerStitch* bloatedRoots[3],
                   unordered_map<int, vector<RectRecord>>& rectsByLayer,
                   ViaStack stack, unsigned int net, const ViaAnchor& anchor,
                   long& outX, long& outY, int maxSites = 8, long radius = 12)
{
    vector<pair<CornerStitch*, unsigned int>> oldNets;
    for (auto* t : anchor.claim) { oldNets.push_back({t, t->getNet()}); t->setNet(net); }
    auto restoreNets = [&]() { for (auto& [t, n] : oldNets) t->setNet(n); };

    CornerStitch* a = anchor.tile;
    auto sites = findViaSites(bloatedRoots, stack, net,
                              a->getllx(), a->getlly(), a->geturx(), a->getury(), maxSites, radius);

    for (auto& s : sites) {
        vector<pair<int,bool>> placed;
        if (!insertViaPads(planeRoots, s.x, s.y, net, stack, &placed)) continue;
        rebuildRectsByLayer(planeRoots, rectsByLayer);       // router rebuilds bloat from this

        // Touching / overlapping the anchor already? Otherwise bridge on the anchor's plane.
        CornerStitch* pad = findTileContaining(planeRoots[anchor.plane], s.x + 0.1, s.y + 0.1);
        if (!pad || !electricallyAdjacent(pad, a)) {
            RoutePair link = { a->getllx(), a->getlly(), s.x, s.y, net };
            if (!attemptRoutePair(link, anchor.plane, planeRoots, bloatedRoots, rectsByLayer, 0, false, false, true)) {
                cout << "Via bridge failed at (" << s.x << "," << s.y << "), trying next site\n";
                removeViaPads(planeRoots, s.x, s.y, stack, placed);
                rebuildRectsByLayer(planeRoots, rectsByLayer);
                continue;
            }
        }

        if (!carveViaContacts(planeRoots, s.x, s.y, stack)) {
            removeViaPads(planeRoots, s.x, s.y, stack, placed);
            rebuildRectsByLayer(planeRoots, rectsByLayer);
            continue;
        }
        rebuildRectsByLayer(planeRoots, rectsByLayer);
        outX = s.x; outY = s.y;
        return true;
    }
    restoreNets();
    return false;
}

struct PlacedVia { CornerStitch* li = nullptr; int dir = 0; };

PlacedVia placeVia(CornerStitch* planeRoots[3], CornerStitch* bloatedRoots[3],
                   unordered_map<int, vector<RectRecord>>& rectsByLayer,
                   ViaStack stack, int anchorPlane, CornerStitch* tile, int avoidDir = 0)
{
    if (!tile || tile->isSpace()) return {};
    const int liPlane = layerNameToPlane("li");
    const unsigned int net = tile->getNet();

    for (auto& anchor : findViaAnchors(planeRoots, anchorPlane, tile, avoidDir)) {
        long vx, vy;
        if (!placeViaStack(planeRoots, bloatedRoots, rectsByLayer, stack, net, anchor, vx, vy)) continue;
        return { findTileContaining(planeRoots[liPlane], vx + 0.1, vy + 0.1), anchor.dir };
    }
    return {};
}

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

    const ViaRule rule = viaRule(VIA_POLY_LI);
    const long S = rule.padW, H = rule.padH;
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

            CornerStitch* viaRoots[3] = { PolyplaneRoot, LiplaneRoot, nullptr };
            if (!insertViaPads(viaRoots, final_lx, final_ly, srcNet, VIA_POLY_LI)) continue;
            if (!carveViaContacts(viaRoots, final_lx, final_ly, VIA_POLY_LI)) {
                cout << "Contact brother setup failed\n";
                continue;                       // same as before: the pads stay in place
            }
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
    const int polyPlane = layerNameToPlane("polysilicon");
    PlacedVia v1 = placeVia(planeRoots, bloatedRoots, rectsByLayer, VIA_POLY_LI, polyPlane, srcTile);
    PlacedVia v2 = v1.li ? placeVia(planeRoots, bloatedRoots, rectsByLayer, VIA_POLY_LI, polyPlane, dstTile, -v1.dir) : PlacedVia{};

    if (!v1.li || !v2.li) {
        cout << "Switch: via placement failed\n";
        return false;
    }
    rebuildRectsByLayer(planeRoots, rectsByLayer);

    //
    RoutePair liRoute = {
        v1.li->getllx(), v1.li->getlly(),
        v2.li->getllx(), v2.li->getlly(),
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
    const ViaRule rule = viaRule(VIA_POLY_LI_M1);
    const long S = rule.padW, H = rule.padH;   // still 5 x 7, so freeOnAllPlanes needs no change

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
        if (!insertViaPads(planeRoots, found_lx, found_ly, net, VIA_POLY_LI_M1)) {
            cout << "  Via pad insertion failed for net " << net << "\n";
            continue;
        }

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

        if (!carveViaContacts(planeRoots, found_lx, found_ly, VIA_POLY_LI_M1))
            cout << "  Contact brother setup failed for net " << net << "\n";
        
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
 
    const ViaRule rule = viaRule(VIA_LI_M1);
    const long S = rule.padW, H = rule.padH;   // 5 x 5

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
 
    if (!insertViaPads(planeRoots, found_lx, found_ly, srcNet, VIA_LI_M1)) {
        cout << "placeViaOnMetal: via pad insertion failed for net " << srcNet << "\n";
        return nullptr;
    }

    // Connect the M1 pad back to srcTile if the position was outside it (unchanged block)
    {
        CornerStitch* m1At  = findTileContaining(planeRoots[m1Plane], found_lx + 0.1, found_ly + 0.1);
        CornerStitch* m1Src = findTileContaining(planeRoots[m1Plane], srcTile->getllx() + 0.1, srcTile->getlly() + 0.1);
        if (m1At && m1Src && !electricallyAdjacent(m1At, m1Src)) {
            RoutePair m1Bridge = { srcTile->getllx(), srcTile->getlly(), found_lx, found_ly, srcNet };
            attemptRoutePair(m1Bridge, m1Plane, planeRoots, bloatedRoots, rectsByLayer, 0, false, false, true);
        }
    }
 
    // FIX: use +1,+2 offsets to match placeInterlayerVias convention
    //      (+1,+1 left the mcon 1 unit short of center in H=5 tiles)
    long cx = found_lx + 1, cy = found_ly + 1;
    if (!carveViaContacts(planeRoots, found_lx, found_ly, VIA_LI_M1))
        cout << "placeViaOnMetal: contact brother setup failed for net " << srcNet << "\n";

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

    // PlacedVia v1 = placeVia(planeRoots, bloatedRoots, rectsByLayer, VIA_LI_M1, m1Plane, srcM1);
    // if (!v1.li) {
    //     cout << "SwitchRoutingMetal: src via placement failed for net " << rpair.net << "\n";
    //     return false;
    // }

    // PlacedVia v2 = placeVia(planeRoots, bloatedRoots, rectsByLayer, VIA_LI_M1, m1Plane, dstM1);
    // if (!v2.li) {
    //     cout << "SwitchRoutingMetal: dst via placement failed for net " << rpair.net << "\n";
    //     return false;
    // }

    // CornerStitch* srcLI = v1.li;     // so the rest of the function stays as it is
    // CornerStitch* dstLI = v2.li;
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