// Layout_Files.hpp
// Reading and writing layout files.
//   Input  .rect : parseRectLine, loadRectsFromFile -> fills the planes and rectsByLayer
//   Output .rect : exportRect  -> routed layout, same format as the input
//   Output .sam  : exportTiles -> raw tile dump of one plane, read by Visualize_Tiles.py
// Depends on Layers.hpp, Corner_Stitch.hpp and Layout_Connectivity.hpp (for RectRecord).
#pragma once
#include <bits/stdc++.h>
#include "Layout_Connectivity.hpp"
using namespace std;

// Dumps every tile reachable from `anchor` as a list of Python-style dict literals
// (True/False, not true/false) because Visualize_Tiles.py reads it with ast.literal_eval.
// Infinite sentinel bounds are clamped to +-100 for plotting.
void exportTiles(CornerStitch* &anchor, const string& filename) {
    if (!anchor) { cout << "[]\n"; return; }

    const long VIS_MIN = -100;   // visualization clamp
    const long VIS_MAX =  100;

    unordered_set<CornerStitch*> seen;
    deque<CornerStitch*> dq;
    vector<CornerStitch*> tiles;

    dq.push_back(anchor);
    seen.insert(anchor);
    while (!dq.empty()) {
        CornerStitch* t = dq.front(); dq.pop_front();

        tiles.push_back(t);

        CornerStitch* nbrs[4] = { t->left(), t->right(), t->bottom(), t->top() };
        for (auto nb : nbrs) {
            if (!nb) continue;
            if (seen.insert(nb).second) dq.push_back(nb);
        }
    }
    ofstream fout(filename);
    if (!fout) return;

    fout << "[\n";
    for (size_t i = 0; i < tiles.size(); i++) {
        CornerStitch* t = tiles[i];

        long llx = t->getllx();
        long lly = t->getlly();
        long urx = t->geturx();
        long ury = t->getury();

        // ---- CLAMP INFINITE BOUNDS TO FINITE VALUES ----
        if (llx == MIN_VALUE) llx = VIS_MIN;
        if (lly == MIN_VALUE) lly = VIS_MIN;
        if (urx == MAX_VALUE) urx = VIS_MAX;
        if (ury == MAX_VALUE) ury = VIS_MAX;

        long wx = urx - llx;
        long wy = ury - lly;

        bool filled = !t->isSpace();

        fout << "  {"
             << "\"name\":\"T" << i << "\","
             << "\"llx\":" << llx << ","
             << "\"lly\":" << lly << ","
             << "\"wx\":" << wx << ","
             << "\"wy\":" << wy << ","
             << "\"filled\":" << (filled ? "True" : "False") << ","
             << "\"net\":" << t->getNet() << ","
             << "\"layer\":\"" << attrToLayerName(t->getAttr()) << "\""
             << "}";

        if (i + 1 < tiles.size()) fout << ",";
        fout << "\n";
    }
    fout << "]\n";
}

// Parses one line of a .rect file:  <kind> <net> <layer> x1 y1 x2 y2   (kind = rect|inrect|outrect)
// Anything after the 6th number (position labels like "right"/"center") is ignored.
// Returns false for comments (#), other kinds, malformed lines and zero-size rectangles.
// Output is the lower-left corner plus width/height.
bool parseRectLine(
    const string& line,
    string& inout,
    string& net,
    string& layer,
    long& lx, long& ly,
    unsigned long& wx, unsigned long& wy
) {
    if (line.empty() || line[0] == '#')
        return false;

    string kind;
    long x1, y1, x2, y2;

    stringstream ss(line);
    ss >> kind;

    if (kind != "rect" && kind != "inrect" && kind != "outrect")
        return false;
    inout = kind;
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

// Reads a .rect file. Each `rect` line is inserted into its plane in BOTH the real planes
// (planeRoots) and the bloated planes (bloatedRoots), and logged in rectsByLayer.
// Skipped: inrect/outrect lines, and layers with no plane. A failed insert (e.g. overlapping
// shapes) is reported and loading carries on.
// NOTE: because inrect/outrect are skipped above, ioMap only ever receives "rect".
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
        else if (inout == "inrect" || inout == "outrect")
            continue;
        ioMap[net] = inout;
        int plane = layerNameToPlane(layer);
        if (plane < 0) continue;

        unsigned int netId = getNetId(net);
        unsigned int attr = layerNameToAttr(layer);

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

// Writes the routed layout as a .rect file: a `bbox 0 0 60 60` line, then every filled
// tile of the three planes as `<kind> <net> <layer> llx lly urx ury`. M1/M2 tiles use the
// kind remembered in ioMap, everything else is "rect". Infinite sentinel bounds are
// clamped to the hard-coded 0..60 box.
void exportRect(CornerStitch* anchors[3], const string& filename) {
    const long VIS_MIN = 0;   // visualization clamp
    const long VIS_MAX = 60;
    ofstream fout(filename);
    if (!fout) return;
    fout << "bbox " << VIS_MIN << " " << VIS_MIN << " " << VIS_MAX << " " << VIS_MAX << "\n";
    for(int i=0; i<3; i++){
        if (!anchors[i]) continue;

        unordered_set<CornerStitch*> seen;
        deque<CornerStitch*> dq;
        vector<CornerStitch*> tiles;

        dq.push_back(anchors[i]);
        seen.insert(anchors[i]);
        while (!dq.empty()) {
            CornerStitch* t = dq.front(); dq.pop_front();
            if(!t->isSpace()) tiles.push_back(t);

            CornerStitch* nbrs[4] = { t->left(), t->right(), t->bottom(), t->top() };
            for (auto nb : nbrs) {
                if (!nb) continue;
                if (seen.insert(nb).second) dq.push_back(nb);
            }
        }

        for (size_t i = 0; i < tiles.size(); i++) {
            CornerStitch* t = tiles[i];
            if(t->isSpace()) continue;
            long llx = t->getllx();
            long lly = t->getlly();
            long urx = t->geturx();
            long ury = t->getury();

            // ---- CLAMP INFINITE BOUNDS TO FINITE VALUES ----
            if (llx == MIN_VALUE) llx = VIS_MIN;
            if (lly == MIN_VALUE) lly = VIS_MIN;
            if (urx == MAX_VALUE) urx = VIS_MAX;
            if (ury == MAX_VALUE) ury = VIS_MAX;

            string net = netList[t->getNet()];
            string inout = ((t->getAttr()==L_M2) || (t->getAttr()==L_M1)) ? ioMap[net] : "rect";

            bool filled = !t->isSpace();

            fout << inout << " "
                << net << " "
                << attrToLayerName(t->getAttr()) << " "
                << llx << " "
                << lly << " "
                << urx << " "
                << ury << " "
                << "\n";

        }
    }
}