// Author: Stavan Mehta, Affaan Fakih
// Version: Pilot Initial (V0)

#include <bits/stdc++.h>
#include "Modular_Route.cpp"
using namespace std;

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
    routingAttrs.push_back(L_M1);
    int MAX_ITERS = 8;

    substrateByNet.clear();
    rebuildLayerByNet(L_NDIFF, planeRoots, substrateByNet, rectsByLayer);
    rebuildLayerByNet(L_PDIFF, planeRoots, substrateByNet, rectsByLayer);

    for (auto& kv : substrateByNet) {
    unsigned int net = kv.first;
    if (net == getNetId("#")) continue;

    // Find the tile with a same-net top neighbor (the bottom tile of the L)
    // and use the shared X overlap between them to place the contact on the top tile
    for (CornerStitch* tile : kv.second) {

        CornerStitch* topTile = nullptr;
        for (auto* n : rightNeighbors(tile))
            if (n && !n->isSpace()  && n->getAttr() == tile->getAttr()) { topTile = n; break; }

        if (!topTile) {
            // Lone tile, place normally
            int side = (tile->getAttr() == L_PDIFF) ? 0 : 1;
            placeContactSquare(planeRoots[layerToPlane("ndiff")],
                               planeRoots[layerToPlane("m1")],
                               L_M1, side, tile, net);
            continue;
        }
        cout << "Here\n";
        // Shared X range between tile and its top neighbor
        long shared_lx = min(tile->getllx(), topTile->getllx());
        long shared_rx = max(tile->geturx(), topTile->geturx());

        if (shared_rx - shared_lx < 5) continue; // too narrow

        const long SQ = 5;
        long x_margin  = (shared_rx - shared_lx - SQ) / 2;
        long square_lx = shared_lx + x_margin;

        // Place on topTile's Y, centered vertically within it
        long y_margin;
        long square_ly;
        if((topTile->getury() - topTile->getlly()) / 2 < (tile->getury() - tile->getlly()) / 2){
            y_margin = (topTile->getury() - topTile->getlly() - SQ) / 2;
            square_ly = topTile->getlly() + y_margin;

        }
        else{
            y_margin  = (tile->getury() - tile->getlly() - SQ) / 2;
            square_ly = tile->getlly() + y_margin;
        }
        

        insertTileRect(planeRoots[layerToPlane("m1")],
                       square_lx, square_ly, SQ, SQ, L_M1, net);
        coalesce(planeRoots[layerToPlane("m1")], 10);
    }
}

    bool interlayerViasDone = false;  
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
            if(layer == L_M1 && done && !interlayerViasDone){
                cout << "\n=== Placing inter-layer vias (poly -> LI -> M1) ===\n";
                
                placeInterlayerVias(planeRoots, bloatedRoots, rectsByLayer);
                exportTiles(planeRoots[0], "plane0_routed.sam");
                exportTiles(planeRoots[1], "plane1_routed.sam");
                exportTiles(planeRoots[2], "plane2_routed.sam");
                // assert(false);

                fullyRoutedNetsByAttr[L_M1].clear();  // force re-check of M1 nets
                iter = 0;
                interlayerViasDone = true;
                polysByNet.clear();
                metalsByNet.clear();
                lisByNet.clear();

                rebuildLayerByNet(L_POLY, planeRoots, polysByNet, rectsByLayer);
                rebuildLayerByNet(L_LI, planeRoots, lisByNet, rectsByLayer);
                rebuildLayerByNet(L_M1, planeRoots, metalsByNet, rectsByLayer);

                continue;
            }
            if (done) {
                cout << "Layer " << attrToLayer(layer) << " fully routed\n";
                break;
            }
        }
    }
   
    // CornerStitch* t =  findTileContaining(planeRoots[0], 25,12);
    // placeViaOnMetal(planeRoots,t,bloatedRoots,rectsByLayer);
    // changeAttrRect(planeRoots[0],-20,-20,10,10,L_POLYCON);
    exportTiles(planeRoots[0], "plane0_routed.sam");
    exportTiles(planeRoots[1], "plane1_routed.sam");
    exportTiles(planeRoots[2], "plane2_routed.sam");
    exportRect(planeRoots, "XNOR2X1_routed.rect");

    return 0;
}