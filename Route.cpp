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
    if (layer == "ndiff" || layer == "pdiff") return 2;
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

unordered_map<int, vector<RectRec>> rectsByLayer;

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

        
        if (!ok || !ok1) {
            cout << "Insert failed at line " << lineNo
                 << " (plane " << plane << "): "
                 << line << "\n";
        }
        rectsByLayer[attr].push_back({plane,layer,attr,netId,lx, ly, wx, wy});
    }
    vector<string> layers = {"ndiff","pdiff","ntransistor","ptransistor","polysilicon","m2"};

    for(const auto layer : layers){
        cout << "Layer: " << layer << " \n";
        for (const auto& r : rectsByLayer[layerToAttr(layer)]) {
            int plane = r.plane;

            // find the exact tile that was inserted
            CornerStitch* t = findTileContaining(
                bloatedRoots[plane],
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
            cout << "bl: " << bl << " br: " << br << " bb: " << bb << " bt: " << bt << "\n";
            long bloated_lx = r.lx - bl;
            long bloated_ly = r.ly - bb;
            long bloated_wx = r.wx + bl + br;
            long bloated_wy = r.wy + bb + bt;
            deleteTileAndCoalesce(bloatedRoots[plane],t);
            bool ok = insertTileRect(bloatedRoots[plane],bloated_lx,bloated_ly,bloated_wx,bloated_wy,r.attr,r.net);
            if (!ok) {
                cout << "Bloated insert failed for rect at ("
                    << r.lx << "," << r.ly << ")\n";
            }

        }
    }
    exportTiles(bloatedRoots[0], "plane0_bloated.sam", true);
    return 0;
}