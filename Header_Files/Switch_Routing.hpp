// Switch_Routing.hpp
// Vias and contacts, and the routing hops built on them.
//
// Building blocks (bottom to top):
//   insertViaPads / removeViaPads / carveViaContacts : put pads down, take them out, cut contacts
//   findViaSites                                     : ranked list of legal pad spots near an anchor
//   findViaAnchors                                   : what a via can attach to (poly gate tab, M1 tile)
//   placeViaStack                                    : one anchor: try the sites, bridge, carve, undo on failure
//   placeVia (+ ViaOptions)                          : the one entry point every caller uses
//   undoVia                                          : take a via we created back out
//
// Callers:
//   attemptSwitchRouting : router fallback. When a direct route on poly or M1 fails or is too long,
//                          put a via at each end, route LI between them (poly->LI->poly, M1->LI->M1).
//   placeInterlayerVias  : once M1 is routed, build a poly-LI-M1 via stack per net; the M1 pad is
//                          a seed that the M1 router connects afterwards.
//   placeContactSquare   : puts a 5x5 M1 contact square on a diffusion tile (near the top for
//                          pdiff, near the bottom for ndiff). Separate job, no via involved.
//
// These functions call back into attemptRoutePair (Router.hpp), which in turn calls
// attemptSwitchRouting, so the two headers must be used together: include this file,
// never Router.hpp alone.
#pragma once
#include <bits/stdc++.h>
#include "Ports_Paths.hpp"
using namespace std;

// Cache of every filled tile as a RectRecord, grouped by layer (attr). Rebuilt by
// rebuildRectsByLayer after tiles change; attemptRoutePair uses it to rebuild the bloated planes.
inline unordered_map<int, vector<RectRecord>> rectsByLayer;

// Nets already known to be fully connected on a layer (attr -> net ids), so routeLayer can
// skip them. Clearing a net here forces it to be routed again (the old rip-up of blocking LI did this).
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
struct ViaSite { long x, y; long gap; long outside; long cdist; long tgap; };

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
                             int maxSites = 8, long radius = 12,
                             const vector<array<long,4>>* targets = nullptr) {
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
            long tgap = 0;                       // gap to the nearest "target" rect (e.g. the net's other M1)
            if (targets && !targets->empty()) {
                tgap = LONG_MAX;
                for (auto& t : *targets) {
                    long tx = max(0L, max(x - t[2], t[0] - (x + rule.padW)));
                    long ty = max(0L, max(y - t[3], t[1] - (y + rule.padH)));
                    tgap = min(tgap, tx + ty);
                }
            }
            all.push_back({x, y, gx + gy, outside, labs(cx2) + labs(cy2), tgap});
        }
    sort(all.begin(), all.end(), [](const ViaSite& a, const ViaSite& b) {
        long ca = a.gap + a.tgap, cb = b.gap + b.tgap;
        return tie(ca, a.outside, a.cdist, a.y, a.x) < tie(cb, b.outside, b.cdist, b.y, b.x);
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

// Rebuild the bloated planes from the real ones: refresh the rect cache, copy every rect in, then
// bloat each layer by its spacing rule. Call it before searching for via sites, because sites are
// checked against the bloated planes and they go stale as soon as anything is placed.
void rebuildBloatedPlanes(CornerStitch* planeRoots[3], CornerStitch* bloatedRoots[3],
                          unordered_map<int, vector<RectRecord>>& rectsByLayer) {
    rebuildRectsByLayer(planeRoots, rectsByLayer);
    for (int p = 0; p < 3; ++p) deleteRect(bloatedRoots[p], -20, -20, 200, 200);
    for (auto& kv : rectsByLayer)
        for (auto& r : kv.second)
            insertTileRect(bloatedRoots[r.plane], r.lx, r.ly, r.wx, r.wy, r.attr, r.net, r.virt);

    static const vector<string> allLayers = {"ndiff","pdiff","ntransistor","ptransistor","polysilicon","m1","li"};
    for (const auto& layer : allLayers) {
        int a = layerNameToAttr(layer);
        int plane = layerNameToPlane(layer);
        unsigned long b = layerBloat(layer);
        for (const auto& r : rectsByLayer[a]) {
            CornerStitch* t = findTileContaining(bloatedRoots[plane], r.lx + (long)r.wx * 0.5, r.ly + (long)r.wy * 0.5);
            if (!t) continue;
            if (!bloatByRect(t, b, b, b, b)) cout << "Bloating Failed " << plane << "\n";
        }
    }
}

// One placed (or reused) via: everything needed to undo it again.
struct PlacedVia {
    CornerStitch* li = nullptr;                          // LI pad tile (the thing LI routes to)
    int dir = 0;                                         // poly only: which side of the gate the tab was on
    bool fresh = false;                                  // true = we created it (so we may undo it); false = reused
    ViaStack stack = VIA_POLY_LI;
    long x = 0, y = 0;                                   // pad lower-left
    vector<pair<int,bool>> placed;                       // from insertViaPads: (plane, was-empty-before)
    vector<pair<CornerStitch*, unsigned int>> claimed;   // tiles we relabelled, with their old nets
    explicit operator bool() const { return li != nullptr; }
};

// What a caller wants from placeVia. Defaults are the plain behaviour.
struct ViaOptions {
    bool reuseExisting = false;   // reuse an LI via this net's M1 already has instead of placing a new one
    bool preferNearM1  = false;   // rank sites near the net's other M1 (the M1 pad is joined to it later)
    int  avoidDir      = 0;       // poly: skip gate tabs on this side (-1 below, +1 above)
    int  maxSites      = 8;       // how many ranked sites to try before giving up
    long radius        = 12;      // how far from the anchor to look
};

bool placeViaStack(CornerStitch* planeRoots[3], CornerStitch* bloatedRoots[3],
                   unordered_map<int, vector<RectRecord>>& rectsByLayer,
                   ViaStack stack, unsigned int net, const ViaAnchor& anchor,
                   PlacedVia& out, const ViaOptions& opt = ViaOptions())
{
    vector<pair<CornerStitch*, unsigned int>> oldNets;
    for (auto* t : anchor.claim) { oldNets.push_back({t, t->getNet()}); t->setNet(net); }
    auto restoreNets = [&]() { for (auto& [t, n] : oldNets) t->setNet(n); };

    CornerStitch* a = anchor.tile;
    // preferNearM1: the stack's M1 pad will have to be joined to the net's other M1 later, so rank
    // sites by how close they are to it as well as to the anchor.
    vector<array<long,4>> m1Targets;
    if (opt.preferNearM1)
        for (auto& r : rectsByLayer[L_M1])
            if (r.net == net) m1Targets.push_back({r.lx, r.ly, r.lx + (long)r.wx, r.ly + (long)r.wy});

    auto sites = findViaSites(bloatedRoots, stack, net,
                              a->getllx(), a->getlly(), a->geturx(), a->getury(), opt.maxSites, opt.radius,
                              m1Targets.empty() ? nullptr : &m1Targets);

    for (auto& s : sites) {
        vector<pair<int,bool>> placed;
        if (!insertViaPads(planeRoots, s.x, s.y, net, stack, &placed)) continue;
        viaRecords.push_back({stack, s.x, s.y, net});
        rebuildRectsByLayer(planeRoots, rectsByLayer);

        // Touching / overlapping the anchor already? Otherwise bridge on the anchor's plane.
        CornerStitch* pad = findTileContaining(planeRoots[anchor.plane], s.x + 0.1, s.y + 0.1);
        if (!pad || !electricallyAdjacent(pad, a)) {
            RoutePair link = { a->getllx(), a->getlly(), s.x, s.y, net };
            if (!attemptRoutePair(link, anchor.plane, planeRoots, bloatedRoots, rectsByLayer, 0, false, false, true, true)) {
                cout << "Via bridge failed at (" << s.x << "," << s.y << "), trying next site\n";
                viaRecords.pop_back();
                removeViaPads(planeRoots, s.x, s.y, stack, placed);
                rebuildRectsByLayer(planeRoots, rectsByLayer);
                continue;
            }
        }

        rebuildRectsByLayer(planeRoots, rectsByLayer);
        out.fresh = true; out.stack = stack; out.x = s.x; out.y = s.y;
        out.placed = placed; out.claimed = oldNets;
        return true;
    }
    restoreNets();
    return false;
}

// Does this net's M1 shape (every same-net tile reachable from `tile`) already sit over a via that
// reaches LI? If so return the LI pad tile of that via, so callers reuse it instead of stacking a new one.
CornerStitch* findExposedLI(CornerStitch* planeRoots[3], CornerStitch* tile) {
    const int liPlane = layerNameToPlane("li");
    const unsigned int net = tile->getNet();

    unordered_set<CornerStitch*> visited;
    deque<CornerStitch*> bfs;
    bfs.push_back(tile); visited.insert(tile);
    while (!bfs.empty()) {
        CornerStitch* cur = bfs.front(); bfs.pop_front();

        // Is this tile over the pad of a same-net via that has an LI layer?
        for (const auto& v : viaRecords) {
            if (v.net != net) continue;
            vector<unsigned int> layers = viaLayers(v.stack);
            if (find(layers.begin(), layers.end(), L_LI) == layers.end()) continue;
            ViaRule rule = viaRule(v.stack);
            if (cur->geturx() <= v.x || cur->getllx() >= v.x + rule.padW ||
                cur->getury() <= v.y || cur->getlly() >= v.y + rule.padH) continue;
            return findTileContaining(planeRoots[liPlane], v.x + 0.1, v.y + 0.1);
        }

        // Otherwise keep walking the net's shape.
        vector<CornerStitch*> nbrs;
        for (auto* t : rightNeighbors(cur))  nbrs.push_back(t);
        for (auto* t : leftNeighbors(cur))   nbrs.push_back(t);
        for (auto* t : topNeighbors(cur))    nbrs.push_back(t);
        for (auto* t : bottomNeighbors(cur)) nbrs.push_back(t);
        for (auto* nb : nbrs)
            if (nb && !visited.count(nb) && nb->getNet() == net && !nb->isSpace()) { visited.insert(nb); bfs.push_back(nb); }
    }
    return nullptr;
}

PlacedVia placeVia(CornerStitch* planeRoots[3], CornerStitch* bloatedRoots[3],
                   unordered_map<int, vector<RectRecord>>& rectsByLayer,
                   ViaStack stack, int anchorPlane, CornerStitch* tile,
                   const ViaOptions& opt = ViaOptions())
{
    if (!tile || tile->isSpace()) return {};
    const int liPlane = layerNameToPlane("li");
    const unsigned int net = tile->getNet();

    if (opt.reuseExisting)
        if (CornerStitch* li = findExposedLI(planeRoots, tile)) {
            cout << "placeVia: net " << net << " already exposed to LI\n";
            PlacedVia reused; reused.li = li; reused.stack = stack;   // fresh = false: nothing to undo
            return reused;
        }

    for (auto& anchor : findViaAnchors(planeRoots, anchorPlane, tile, opt.avoidDir)) {
        PlacedVia pv;
        if (!placeViaStack(planeRoots, bloatedRoots, rectsByLayer, stack, net, anchor, pv, opt)) continue;
        pv.li  = findTileContaining(planeRoots[liPlane], pv.x + 0.1, pv.y + 0.1);
        pv.dir = anchor.dir;
        return pv;
    }
    return {};
}

// Take a via we created back out. Reused vias are left alone.
void undoVia(CornerStitch* planeRoots[3], unordered_map<int, vector<RectRecord>>& rectsByLayer,
             const PlacedVia& v) {
    if (!v.li || !v.fresh) return;
    ViaRule rule = viaRule(v.stack);
    long cx = v.x + rule.offX(), cy = v.y + rule.offY();
    vector<unsigned int> layers = viaLayers(v.stack);

    for (auto it = viaRecords.begin(); it != viaRecords.end(); ++it)
        if (it->stack == v.stack && it->x == v.x && it->y == v.y) { viaRecords.erase(it); break; }
    //remove pads on planes we filled ourselves
    removeViaPads(planeRoots, v.x, v.y, v.stack, v.placed);
    //give relabelled tiles (gate tab + transistor) their old nets back
    for (auto& [t, n] : v.claimed) t->setNet(n);
    for (int p = 0; p < 3; ++p) if (planeRoots[p]) coalesce(planeRoots[p], 10);
    rebuildRectsByLayer(planeRoots, rectsByLayer);
}

void finalizeVias(CornerStitch* planeRoots[3]) {
    for (const auto& v : viaRecords)
        if (!carveViaContacts(planeRoots, v.x, v.y, v.stack))
            cout << "finalizeVias: contact carve failed at (" << v.x << "," << v.y << ") net " << v.net << "\n";
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

// Switch routing: hop to local interconnect when a direct route on the anchor layer failed or is
// too long. Put a via at each end (reusing one if the net already has it), route LI between them,
// and take back whatever we created if the hop does not work out.
//   anchorPlane = poly plane -> poly -> LI -> poly       anchorPlane = M1 plane -> M1 -> LI -> M1
bool attemptSwitchRouting(
    const RoutePair &rpair,
    int iter,
    int anchorPlane,
    CornerStitch* planeRoots[],
    CornerStitch* bloatedRoots[],
    unordered_map<int, vector<RectRecord>>& rectsByLayer
) {
    const int  polyPlane = layerNameToPlane("polysilicon");
    const int  liPlane   = layerNameToPlane("li");
    const bool onPoly    = (anchorPlane == polyPlane);
    const ViaStack stack = onPoly ? VIA_POLY_LI : VIA_LI_M1;

    if (rpair.src_x == rpair.dst_x && rpair.src_y == rpair.dst_y) return false;

    CornerStitch* srcTile = findTileContaining(planeRoots[anchorPlane], rpair.src_x, rpair.src_y);
    CornerStitch* dstTile = findTileContaining(planeRoots[anchorPlane], rpair.dst_x, rpair.dst_y);
    if (!srcTile || srcTile->isSpace() || !dstTile || dstTile->isSpace()) {
        cout << "Switch: src/dst not on the anchor layer for net " << rpair.net << "\n";
        return false;
    }

    // --- via at each end ---
    // metal: reuse an LI via the net already has. poly: no reuse (a used gate tab is gone anyway).
    ViaOptions opt;
    opt.reuseExisting = !onPoly;

    PlacedVia v1 = placeVia(planeRoots, bloatedRoots, rectsByLayer, stack, anchorPlane, srcTile, opt);
    if (!v1) { cout << "Switch: src via placement failed for net " << rpair.net << "\n"; return false; }

    // poly: keep the second end off the gate side the first end used
    opt.avoidDir = onPoly ? -v1.dir : 0;
    PlacedVia v2 = placeVia(planeRoots, bloatedRoots, rectsByLayer, stack, anchorPlane, dstTile, opt);
    if (!v2) {
        cout << "Switch: dst via placement failed for net " << rpair.net << "\n";
        undoVia(planeRoots, rectsByLayer, v1);
        return false;
    }

    // Both ends already sit on one LI shape (e.g. reused vias): the hop is already made.
    if (v1.li == v2.li || electricallyAdjacent(v1.li, v2.li)) {
        cout << "Switch: vias already connected for net " << rpair.net << "\n";
        return true;
    }

    // --- LI between the two pads ---
    // advance_routepair = true: ports come from the whole LI shape each pad belongs to, not just
    // the pad tile, so a reused via that already has LI wire on it is routed from that wire.
    RoutePair liRoute = { v1.li->getllx(), v1.li->getlly(), v2.li->getllx(), v2.li->getlly(), rpair.net };
    bool ok = attemptRoutePair(liRoute, liPlane, planeRoots, bloatedRoots, rectsByLayer, iter,
                               /*try_switch*/ false, /*bloat_src*/ false, /*advance_routepair*/ true);
    if (!ok) {
        cout << "Switch: LI routing failed for net " << rpair.net << "\n";
        undoVia(planeRoots, rectsByLayer, v2);
        undoVia(planeRoots, rectsByLayer, v1);
        return false;
    }

    cout << "Switch routing succeeded for net " << rpair.net << "\n";
    return true;
}

// Once M1 is routed: for each net that has both poly and M1, build a poly-LI-M1 via stack next to a
// gate extrusion. The M1 pad is left as a separate seed; the M1 router connects it afterwards.
void placeInterlayerVias(
    CornerStitch* planeRoots[3],
    CornerStitch* bloatedRoots[3],
    unordered_map<int, vector<RectRecord>>& rectsByLayer
) {
    const int polyPlane = layerNameToPlane("polysilicon");

    unordered_map<unsigned int, vector<CornerStitch*>> localPolyByNet, localM1ByNet;
    rebuildLayerByNet(L_POLY, planeRoots, localPolyByNet, rectsByLayer);
    rebuildLayerByNet(L_M1,   planeRoots, localM1ByNet,   rectsByLayer);

    for (auto& [net, polyTiles] : localPolyByNet) {
        if (net == getNetId("#")) continue;
        if (!localM1ByNet.count(net)) continue;

        cout << "Inter-layer via for net " << net << "\n";
        rebuildBloatedPlanes(planeRoots, bloatedRoots, rectsByLayer);

        ViaOptions opt;
        opt.preferNearM1 = true;      // the M1 pad is joined to the net's M1 later
        PlacedVia v = placeVia(planeRoots, bloatedRoots, rectsByLayer,
                               VIA_POLY_LI_M1, polyPlane, polyTiles[0], opt);
        if (!v) cout << "  No via stack placed for net " << net << "\n";
        else    cout << "  Via stack placed at (" << v.x << "," << v.y << ") for net " << net << "\n";
    }
    rebuildRectsByLayer(planeRoots, rectsByLayer);
}