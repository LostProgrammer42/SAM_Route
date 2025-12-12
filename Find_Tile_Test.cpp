
int main() {
    // 1. Create ONE big tile from (0,0) to (100,100) using boundaries.
    CornerStitch* root = new CornerStitch(0, 0, 1, 0);  // space tile

    // 2. First horizontal split at y = 40
    CornerStitch* top1 = splitHorizontal(root, 40);

    // 3. Second horizontal split on the TOP part at y = 70
    CornerStitch* top2 = splitHorizontal(top1, 70);

    CornerStitch* top2_right = splitVertical(top2, 60);
    // 4. Export tiles (NO SKIPPING!)
    exportTiles(root);
    vector<pair<string, pair<long,long>>> tests = {
        {"(10, 10)", {10, 10}},   // should be T3
        {"(30, 75)", {30, 75}},   // should be T1
        {"(70, 20)", {70, 20}},   // should be T4
        {"(80, 80)", {80, 80}},   // should be T2
        {"(200, 200)", {200, 200}} // outside → nullptr
    };

    for (auto &t : tests) {
        string name = t.first;
        long x = t.second.first;
        long y = t.second.second;

        CornerStitch *res = findTileContaining(root, x, y);
        cout << "Point " << name << " → ";

        if (!res) {
            cout << "NULL (not found)\n";
        } else if (res == root) {
            cout << "Root\n";
        } else if (res == top1) {
            cout << "Top-1\n";
        } else if (res == top2) {
            cout << "Top-2-Left\n";
        } else if (res == top2_right) {
            cout << "Top-2-Right\n";
        } else {
            cout << "UNKNOWN TILE\n";
        }
    }
    cout << "\n--- Testing tilesInRect ---\n";
    long lx = 120;
    long ly = 20;
    long wx = 40;
    long wy = 100;

    vector<CornerStitch*> rectTiles = tilesInRect(root, lx, ly, wx, wy);

    cout << "Rectangle (120,20) w=40 h=100 intersects tiles: ";

    for (auto t : rectTiles) {
        if (t == root) cout << "Root ";
        else if (t == top1) cout << "Top-1 ";
        else if (t == top2) cout << "Top-2-Left ";
        else if (t == top2_right) cout << "Top-2-Right ";
        else cout << "Boundary ";
    }
    cout << "\n";
    return 0;
}