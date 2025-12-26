// Author: Stavan Mehta, Affaan Fakih
// Version: Pilot Initial (V0)

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
    if (layer == "ndiff" || layer == "pdiff" || layer == "ndiffusion" || layer == "pdiffusion") return 1;
    if (layer == "ntransistor" || layer == "ptransistor") return 1;
    if (layer == "polysilicon") return 1;
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
    vector <CornerStitch*> rootsVector ={bloatedRoots[0],bloatedRoots[1],bloatedRoots[2]};
    string line;
    int lineNo = 0;
    while (getline(fin, line)) {
        lineNo++;

        string net, layer, inout;
        long lx, ly, wx, wy;

        if (!parseRectLine(line, inout,net, layer, lx, ly, wx, wy))
            continue;
        ioMap[net] = inout;
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
    
    
    // poly = findTileContaining(bloatedRoots[0],34,8);
    // if (poly->left())   {poly->left()->setAttr(L_PTR_LEFT); poly->left()->setSpace(0);}
    // if (poly->right())  {poly->right()->setAttr(L_PTR_RIGHT); poly->right()->setSpace(0);}
    // if (poly->top())    {poly->top()->setAttr(L_PTR_TOP); poly->top()->setSpace(0);}
    // if (poly->bottom()) {poly->bottom()->setAttr(L_PTR_BOTTOM); poly->bottom()->setSpace(0);}
    
    
    vector<string> layers = {"ndiff","pdiff","ntransistor","ptransistor","polysilicon","m2"};

    for(const auto layer : layers){
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

            bool ok1 = bloatByRect(t,b,b,b,b); 

            if (!ok1) {
                cout << "Bloated insert failed for rect at ("
                    << r.lx << "," << r.ly << ")\n";
            }
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
    exportRect(bloatedRoots,"INVX8_bloat.rect");

    return 0;
}