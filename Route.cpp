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
    if (layer == "ndiff" || layer == "pdiff") return 0;
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
        rects.push_back({
        plane,
        layer,
        attr,
        netId,
        lx, ly, wx, wy
        });
    }

    // CornerStitch* poly = findTileContaining(planeRoots[0],38,7);
    // if (poly->left())   {poly->left()->setAttr(L_PTR_LEFT); poly->left()->setSpace(0);}
    // if (poly->right())  {poly->right()->setAttr(L_PTR_RIGHT); poly->right()->setSpace(0);}
    // if (poly->top())    {poly->top()->setAttr(L_PTR_TOP); poly->top()->setSpace(0);}
    // if (poly->bottom()) {poly->bottom()->setAttr(L_PTR_BOTTOM); poly->bottom()->setSpace(0);}
    
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

        long bloated_lx = r.lx - bl;
        long bloated_ly = r.ly - bb;
        long bloated_wx = r.wx + bl + br;
        long bloated_wy = r.wy + bb + bt;

        bool ok = insertTileRect(
            bloatedRoots[plane],
            bloated_lx,
            bloated_ly,
            bloated_wx,
            bloated_wy,
            r.attr,
            r.net
        );

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
    

    return 0;
}