#include <bits/stdc++.h>
#include "Corner_Stitch.cpp"
using namespace std;

int layerToPlane(const string& layer) {
    if (layer == "ndiff" || layer == "pdiff" || layer == "ndiffusion" || layer == "pdiffusion" ||
        layer == "ntransistor" || layer == "ptransistor")
        return 0; // diffusion, devices

    if (layer == "polysilicon")
        return 0    ; // poly
    
    if (layer == "m1")
        return 1;    

    if (layer == "m2")
        return 2; // metal

    return -1; 
}

unordered_map<string, string> ioMap;
unordered_map<string, unsigned int> netMap;
vector<string> netList = {"#"};
unsigned int nextNetId = 1;

unsigned int getNetId(const string& net) {
    if (net == "#") return 0;
    if (netMap.count(net)) return netMap[net];
    netMap[net] = nextNetId;
    netList.push_back(net);
    return nextNetId++;
}

bool parseRectLine(
    const string& line,
    string& inout,
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



unsigned int layerToAttr(const string& layer) {
    if (layer == "ndiff" || layer == "ndiffusion") return L_NDIFF;
    if (layer == "pdiff" || layer == "pdiffusion") return L_PDIFF;
    if (layer == "ntransistor")  return L_NTRANS;
    if (layer == "ptransistor")  return L_PTRANS;
    if (layer == "polysilicon")  return L_POLY;
    if (layer == "m1")           return L_M1;    
    if (layer == "m2")           return L_M2;
    return L_NONE;
}

long layerBloat(const string& layer) {
    if (layer == "ndiff" || layer == "pdiff" || layer == "ndiffusion" || layer == "pdiffusion") return 2;
    if (layer == "ntransistor" || layer == "ptransistor") return 0;
    if (layer == "polysilicon") return 3;
    if (layer == "m1") return 0;
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

enum Dir { LEFT, RIGHT, UP, DOWN };
struct Point {
    long x, y;
    bool operator==(const Point& other) const {
        return x == other.x &&
               y == other.y;    }
};

struct Port {
    long x, y;
    Dir normal;

    bool operator==(const Port& other) const {
        return x == other.x &&
               y == other.y &&
               normal == other.normal;
    }
};


struct RoutePair {
    long src_x, src_y;
    long dst_x, dst_y;
    unsigned int net;
};

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
                << attrToLayer(t->getAttr()) << " "
                << llx << " "
                << lly << " "
                << urx << " "
                << ury << " "
                << "\n";

        }
    }
}

vector<Port> getPorts(
    CornerStitch* t,              // NORMAL plane tile
    CornerStitch* bloatedRoot ,     // BLOATED plane root
    bool multiSample 
) {
    vector<Port> ports;
    vector<Port> result;

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
    
    if(multiSample){
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
    }
        

    for(auto port : ports){
        switch(port.normal){
            case LEFT:  if(findTileContaining(t,port.x-0.1,port.y)->isSpace()) result.push_back(port);
            case RIGHT: if(findTileContaining(t,port.x+0.1,port.y)->isSpace()) result.push_back(port);
            case UP:    if(findTileContaining(t,port.x,port.y+0.1)->isSpace()) result.push_back(port);
            case DOWN:  if(findTileContaining(t,port.x,port.y-0.1)->isSpace()) result.push_back(port);
            default: continue;
        }
    }

    return ports;
}

bool inferPreferHorizontal(const vector<vector<Point>>& pathPieces, bool defaultDir)
{
    if (pathPieces.empty()) return defaultDir;

    const auto& last = pathPieces.back();
    if (last.size() < 2) return defaultDir;

    const Point& a = last[last.size() - 2];
    const Point& b = last[last.size() - 1];

    if (a.y == b.y) return true;
    if (a.x == b.x) return false;

    return defaultDir;
}


vector<Point> pathInTile(CornerStitch* t, Point entry, Point exit, bool preferHorizontal)
{
    vector<Point> result;
    if (!t) return result;

    long x1 = entry.x, y1 = entry.y;
    long x2 = exit.x,  y2 = exit.y;

    result.push_back({x1, y1});

    // Trivial cases
    if (x1 == x2 && y1 == y2) return result;
    if (x1 == x2 || y1 == y2) {
        result.push_back({x2, y2});
        return result;
    }

    // Determine possible bend points
    Point hFirst = {x2, y1};  // horizontal → vertical
    Point vFirst = {x1, y2};  // vertical → horizontal

    auto insideTile = [&](Point p) {
        return p.x >= t->getllx() && p.x <= t->geturx() &&
               p.y >= t->getlly() && p.y <= t->getury();
    };

    // Prefer continuing direction if valid
    if (preferHorizontal && insideTile(hFirst)) {
        result.push_back(hFirst);
    }
    else if (!preferHorizontal && insideTile(vFirst)) {
        result.push_back(vFirst);
    }
    else if (insideTile(hFirst)) {
        result.push_back(hFirst);
    }
    else if (insideTile(vFirst)) {
        result.push_back(vFirst);
    }
    else {
        // Fallback: midpoint (should be rare)
        long cx = (x1 + x2) / 2;
        long cy = (y1 + y2) / 2;
        result.push_back({cx, y1});
        result.push_back({cx, y2});
    }

    result.push_back({x2, y2});
    return result;
}




bool WholeNetElectricallyConnected(const vector<CornerStitch*>& polys) {
    if (polys.empty()) return true;

    unordered_set<CornerStitch*> visited;
    deque<CornerStitch*> q;

    q.push_back(polys[0]);
    visited.insert(polys[0]);

    while (!q.empty()) {
        CornerStitch* cur = q.front();
        q.pop_front();

        for (CornerStitch* other : polys) {
            if (visited.count(other)) continue;
            if (!electricallyAdjacent(cur, other)) continue;

            visited.insert(other);
            q.push_back(other);
        }
    }

    return visited.size() == polys.size();
}

Point findClosestPoint(CornerStitch* t, Point dest){
    if(!t) return dest;
    if(t->containsPoint(dest.x,dest.y)) return dest;
    if(t->getllx()<=dest.x && dest.x<t->geturx()){
        if(llabs(dest.y-t->getlly()) > llabs(dest.y-t->getury())) return {dest.x,t->getury()};
        else return {dest.x,t->getlly()};
    }
    if(t->getlly()<=dest.y && dest.y<t->getury()){
        if(llabs(dest.x-t->getllx()) > llabs(dest.x-t->geturx())) return {t->geturx(),dest.y};
        else return {t->getllx(),dest.y};
    }
    long x,y;
    if(llabs(dest.y-t->getlly()) > llabs(dest.y-t->getury())) y = t->getury();
    else y = t->getlly();
    if(llabs(dest.x-t->getllx()) > llabs(dest.x-t->geturx())) x = t->geturx();
    else x = t->getllx();
    return {x,y};
}

void rebuildRectsByLayer(
    CornerStitch* roots[3],
    unordered_map<int, vector<RectRec>>& rectsByLayer
) {
    rectsByLayer.clear();
    for(int i=0; i<3; i++){
        unordered_set<CornerStitch*> seen;
        deque<CornerStitch*> dq;
        dq.push_back(roots[i]);
        seen.insert(roots[i]);

        while (!dq.empty()) {
            CornerStitch* t = dq.front(); dq.pop_front();
            if (!t) continue;

            if (!t->isSpace()) {
                long llx = t->getllx();
                long lly = t->getlly();
                long urx = t->geturx();
                long ury = t->getury();

                rectsByLayer[t->getAttr()].push_back({
                    i,                              // plane
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
    
}

void rebuildLayerByNet(
    int Layer, 
    CornerStitch* roots[3],
    unordered_map<unsigned int, vector<CornerStitch*>>& layerByNet,
    unordered_map<int, vector<RectRec>>& rectsByLayer)
{
    for (auto& [attr, rects] : rectsByLayer) {
        if (attr != Layer) continue;

        for (auto& r : rects) {
            CornerStitch* t = findTileContaining(
                roots[r.plane],
                r.lx + 0.1,
                r.ly + 0.1
            );
            if (t && !t->isSpace()) {
                layerByNet[t->getNet()].push_back(t);
            }
        }
    }
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

    const long SQ = 3;
    const long MIN_MARGIN = 1;

    long tileWidth  = rx - lx;
    long tileHeight = ry - ly;

    if (tileWidth < SQ + 2 * MIN_MARGIN) return false;
    if (tileHeight < SQ + 2 * MIN_MARGIN) return false;

    long MARGIN = (tileWidth - SQ) / 2;
    if (MARGIN < MIN_MARGIN) return false;

    if ((rx - lx) < SQ + 2*MARGIN) return false;
    if ((ry - ly) < SQ + 2*MARGIN) return false;

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