// Author: Stavan Mehta, Affaan Fakih
// Version: Pilot Initial (V0)

#include <bits/stdc++.h>
#include "Modular_Route.cpp"
using namespace std;
int z=0;

int main(int argc, char** argv) {
    if (argc < 2) {
        cerr << "Usage: ./Route <file.rect> [layer1,layer2,...]\n";
        cerr << "Example: ./Route input.rect polysilicon,m1\n";
        return 1;
    }

    // Create plane roots
    CornerStitch* planeRoots[3];
    CornerStitch* bloatedRoots[3];
    for (int i = 0; i < 3; i++) {
        planeRoots[i] = new CornerStitch(-30, -30);
        bloatedRoots[i] = new CornerStitch(-30, -30);
    }

    string filename = argv[1];
    loadRectsFromFile(filename, planeRoots, bloatedRoots, rectsByLayer);

    cout << "\n=== PLANE " << 0 << " ===\n";
    exportTiles(planeRoots[0], "plane0.sam");
    cout << "\n=== PLANE " << 1 << " ===\n";
    exportTiles(planeRoots[1], "plane1.sam");
    cout << "\n=== PLANE " << 2 << " ===\n";
    exportTiles(planeRoots[2], "plane2.sam");

    // Choose which layers to route this run. Default = polysilicon
    vector<int> routingAttrs;
    if (argc >= 3) {
        string arg = argv[2];
        stringstream ss(arg);
        string token;
        while (getline(ss, token, ',')) {
            int a = layerToAttr(token);
            if (a != L_NONE) routingAttrs.push_back(a);
            else cerr << "Unknown layer name in arg: " << token << "\n";
        }
    } else {
        routingAttrs.push_back(L_POLY);
    }
    // routingAttrs.push_back(L_M1);
    int MAX_ITERS = 8;

    substrateByNet.clear();
    rebuildLayerByNet(L_NDIFF, planeRoots, substrateByNet, rectsByLayer);
    rebuildLayerByNet(L_PDIFF, planeRoots, substrateByNet, rectsByLayer);
    for (auto& kv : substrateByNet) {
        unsigned int net = kv.first;
        if (net == getNetId("#")) continue;

        for (CornerStitch* tile : kv.second) {
            if(tile->getAttr() == L_PDIFF){
                placeContactSquare(planeRoots[layerToPlane("ndiff")],planeRoots[layerToPlane("m1")],L_M1,0,tile,net);
            }
            else{
                placeContactSquare(planeRoots[layerToPlane("ndiff")],planeRoots[layerToPlane("m1")],L_M1,1,tile,net);
            }
        }
    }

    for (int layer : routingAttrs) {
        int iter = 0;

        while (true) {
            cout << "Layer " << layer << " Iteration: " << iter + 1 << "\n";
            if (iter++ > MAX_ITERS) {
                cout << "Max Iterations Reached for layer " << layer << "\n";
                break;
            }

            // Route ONLY this layer
            routeLayer(layer, planeRoots, bloatedRoots, rectsByLayer, iter);

            polysByNet.clear();
            metalsByNet.clear();
            lisByNet.clear();

            rebuildLayerByNet(L_POLY, planeRoots, polysByNet, rectsByLayer);
            rebuildLayerByNet(L_LI, planeRoots, lisByNet, rectsByLayer);
            rebuildLayerByNet(L_M1, planeRoots, metalsByNet, rectsByLayer);

            // Remove fully routed nets
            if (fullyRoutedNetsByAttr.count(L_POLY)) {
                for (auto netId : fullyRoutedNetsByAttr[L_POLY])
                    polysByNet.erase(netId);
            }
            if (fullyRoutedNetsByAttr.count(L_LI)) {
                for (auto netId : fullyRoutedNetsByAttr[L_LI])
                    lisByNet.erase(netId);
            }
            if (fullyRoutedNetsByAttr.count(L_M1)) {
                for (auto netId : fullyRoutedNetsByAttr[L_M1])
                    metalsByNet.erase(netId);
            }

            auto allDone = [](const unordered_map<unsigned int, vector<CornerStitch*>> &m) {
                for (const auto &kv : m) {
                    if (kv.first == getNetId("#")) continue;
                    if (kv.second.size() > 1) return false;
                }
                return true;
            };

            bool done = false;
            if (layer == L_POLY) done = allDone(polysByNet);
            else if (layer == L_M1) done = allDone(metalsByNet);

            if (done) {
                cout << "Layer " << attrToLayer(layer) << " fully routed\n";
                break;
            }
        }
    }

    exportTiles(planeRoots[0], "plane0_routed.sam");
    exportTiles(planeRoots[1], "plane1_routed.sam");
    exportTiles(planeRoots[2], "plane2_routed.sam");
    exportRect(planeRoots, "XNOR2X1_routed.rect");

    return 0;
}