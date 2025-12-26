// Author: Stavan Mehta
// Version: Pilot Initial (V0)

#include <bits/stdc++.h>
using namespace std;

#define MIN_VALUE (numeric_limits<long>::min())
#define MAX_VALUE (numeric_limits<long>::max())

#define DEBUG_MODE false

enum LayerType {
    L_NONE = 0,
    L_NDIFF,
    L_PDIFF,
    L_NTRANS,
    L_PTRANS,
    L_POLY,
    L_M2
};
#define L_PTR_LEFT   50
#define L_PTR_RIGHT  51
#define L_PTR_TOP    52
#define L_PTR_BOTTOM 53


string attrToLayer(unsigned int attr) {
    switch (attr) {
        case L_NDIFF:   return "ndiff";
        case L_PDIFF:   return "pdiff";
        case L_NTRANS:  return "ntransistor";
        case L_PTRANS:  return "ptransistor";
        case L_POLY:    return "polysilicon";
        case L_M2:      return "m2";
        case L_PTR_LEFT: return "left";
        case L_PTR_RIGHT: return "right";
        case L_PTR_TOP: return "top";
        case L_PTR_BOTTOM: return "bottom";
        default:        return "none";
    }
}

class Rectangle {
private:
    long _llx = 0, _lly = 0;  //Lower Left Corner Coordinated
    unsigned long _wx = 0, _wy = 0; //Width and Height
public:
    //Constructors
    Rectangle() = default;
    Rectangle(long lx, long ly, unsigned long wx, unsigned long wy)
        : _llx(lx), _lly(ly), _wx(wx), _wy(wy) {}
    
    //Modify Rectangle
    void setRect(long lx, long ly, unsigned long wx, unsigned long wy) {
        _llx = lx; _lly = ly; _wx = wx; _wy = wy;
    }
    void clear() { _llx = _lly = 0; _wx = _wy = 0; }

    // Check if Empty
    bool empty() const { return _wx == 0 || _wy == 0; }

    //Get Corner Coordinates and Widths
    long llx() const { return _llx; }
    long lly() const { return _lly; }
    unsigned long wx() const { return _wx; }
    unsigned long wy() const { return _wy; }

    //Get Coordinates of Upper Right Corner
    long urx() const { return _llx + (long)_wx - 1; } 
    long ury() const { return _lly + (long)_wy - 1; }

    // Helper Functions
    bool inXRange(long x) const { return _llx <= x && x <= urx(); }
    bool inYRange(long y) const { return _lly <= y && y <= ury(); }
    bool contains(const Rectangle &r) const {
        if (empty() || r.empty()) return false;
        return _llx <= r._llx && _lly <= r._lly && urx() >= r.urx() && ury() >= r.ury();
    }
    void print(ostream &os=cout) const {
        os << "(" << _llx << "," << _lly << ") -> (" << urx() << "," << ury() << ")";
    }
};

class CornerStitch{
    public:
        struct CornerPointers
        {
            CornerStitch *x = nullptr, *y = nullptr; 
        };
        
    private:
        CornerPointers ll, ur;
        long llx, lly; // A Tile only has the Lower Left Corner
        
        //Flags for Space/Virtual Tile
        unsigned int space:1;
        unsigned int virt:1;
        
        //6 bits of Attribute 
        unsigned int attr:6;

        //6 bits of Net Type
        unsigned int net:6;
    public:
        //Constructors
        CornerStitch():
            llx(MIN_VALUE), lly(MIN_VALUE), space(1), virt(1), attr(0), net(0)
            {ll.x = ll.y = ur.x = ur.y = nullptr;}
        
        CornerStitch(long _llx, long _lly, unsigned int _space=1, unsigned int _virt=0, unsigned int _attr=0, unsigned int _net = 0):
            llx(_llx), lly(_lly), space(_space), virt(_virt), attr(_attr), net(_net)
            {ll.x = ll.y = ur.x = ur.y = nullptr;}
        
        ~CornerStitch() = default;

        // Get Coordinates and Widths
        long getllx() const { return llx; }
        long getlly() const { return lly; }
        long geturx() const { return ur.x ? ur.x->llx : MAX_VALUE; } // Check if UR.x Tile exists, if yes return it's lower left corner
        long getury() const { return ur.y ? ur.y->lly : MAX_VALUE; }
        unsigned int getNet() const {return net;}
        bool isSpace() const { return space; }
        unsigned int getAttr() const { return attr; }
        bool isVirt() const { return virt; }
        CornerStitch* left()   const { return ll.x; }
        CornerStitch* bottom() const { return ll.y; }
        CornerStitch* right()  const { return ur.x; }
        CornerStitch* top()    const { return ur.y; }

        void setSpace(unsigned int s) { space = s; }
        void setAttr(unsigned int a) { attr = a; }
        void setNet(unsigned int n) { net = n; }
        void setLeft(CornerStitch* L)   { ll.x = L; }
        void setBottom(CornerStitch* B) { ll.y = B; }
        void setRight(CornerStitch* R)  { ur.x = R; }
        void setTop(CornerStitch* T)    { ur.y = T; }
        

        bool containsPoint(float x, float y) const {
            return (getllx() <= x && x < geturx() && getlly() <= y && y < getury());
        }

        bool intersectsRect(float lx, float ly, float wx, float wy) const {
            if (geturx() <= lx) return false;
            if (lx + wx <= getllx()) return false;
            if (getury() <= ly) return false;
            if (ly + wy <= getlly()) return false;
            return true;
        }
};


// Finds the tile that contains point (x,y)
// anchor: any known tile (commonly a root/sentinel)
// Returns pointer to the containing CornerStitch, or nullptr if not found
CornerStitch* findTileContaining(CornerStitch *anchor, float x, float y) {
    if (!anchor) return nullptr;
    if (anchor->containsPoint(x,y)) return anchor;

    // Heuristic: walk left to get nearer horizontally (if possible)
    CornerStitch *cur = anchor;
    while (true) {
        CornerStitch *L = cur->left();
        if (!L) break;
        // only move left if that tile's llx is <= x and strictly less than cur's llx
        long Lllx = L->getllx();
        if (Lllx <= x && Lllx < cur->getllx()) cur = L;
        else break;
    }

    // Scan right along the row using right() pointers
    for (CornerStitch *row = cur; row; row = row->right()) {
        // check vertical span first
        if (row->getlly() <= y && y < row->getury()) {
            if (row->containsPoint(x,y)) return row;
        }
        // if we've passed x, stop this row scan
        if (row->getllx() > x) break;
    }
    unordered_set<CornerStitch*> seen;
    deque<CornerStitch*> dq;
    dq.push_back(anchor);
    seen.insert(anchor);

    while (!dq.empty()) {
        CornerStitch *t = dq.front(); dq.pop_front();
        if (t->containsPoint(x,y)) return t;

        CornerStitch *nbrs[4] = { t->left(), t->right(), t->bottom(), t->top() };
        for (auto nb : nbrs) {
            if (!nb) continue;
            if (seen.insert(nb).second) dq.push_back(nb);
        }
    }
    return nullptr; // not found (point outside tiled area or graph broken)
}

vector<CornerStitch*> tilesInRect(CornerStitch* anchor,
                                  float lx, float ly,
                                  float wx, float wy)
{
    vector<CornerStitch*> result;
    if (!anchor) return result;

    // Step 1: find a starting tile near the rect
    CornerStitch* start = findTileContaining(anchor, lx, ly);

    // If outside layout, start from anchor anyway
    if (!start) start = anchor;

    // BFS
    unordered_set<CornerStitch*> seen;
    deque<CornerStitch*> dq;

    dq.push_back(start);
    seen.insert(start);

    while (!dq.empty()) {
        CornerStitch* t = dq.front(); dq.pop_front();

        // If intersects → add to result
        if (t->intersectsRect(lx, ly, wx, wy)) {
            result.push_back(t);
        }

        // Explore neighbors
        CornerStitch* nbrs[4] = { t->left(), t->right(), t->bottom(), t->top() };
        for (auto nb : nbrs) {
            if (!nb) continue;
            if (seen.insert(nb).second) {
                dq.push_back(nb);
            }
        }
    }

    return result;
}


// Split tile t horizontally at splitY (splitY is the lly of the new top tile).
// Returns pointer to the new top tile on success, nullptr on failure.
CornerStitch* splitHorizontal(CornerStitch* t, long splitY) {
    if (!t) return nullptr;                     
    long a = t->getlly();
    long b = t->getury();                      
    if (!(a < splitY && splitY < b)) return nullptr;

    CornerStitch* top = new CornerStitch(t->getllx(), splitY, t->isSpace(), t->isVirt(), t->getAttr(), t->getNet());

    // === Fix pointers of top tile ===
    // Top tile inherits right, top pointers from t
    top->setTop(t->top());
    top->setRight(t->right());
    // Top tile has original tile as the bottom
    top->setBottom(t);
    // Find the new left pointer

    top->setLeft(findTileContaining(t,t->getllx() - 0.1 ,splitY+0.1));


    // === Fix pointers of original tile ===
    // Left, Bottom pointers remain same
    // Update the original tile's top pointer to point to the new top tile
    t->setTop(top);
    // Find the new right pointer
    t->setRight(findTileContaining(t,t->geturx() + 0.1 ,splitY-0.1));


    // Fix neighbors above the top tile if they used to point at t for their bottom links
    CornerStitch* above = top->top();
    if (above) {
        // walk left
        for (CornerStitch* cur = above; cur; cur = cur->left()) {
            if (cur->bottom() == t)
                cur->setBottom(top);
            else
                break;
        }
        // walk right
        for (CornerStitch* cur = above->right(); cur; cur = cur->right()) {
            if (cur->bottom() == t)
                cur->setBottom(top);
            else
                break;
        }
    }

    CornerStitch* right = top->right();
    if (right) {
        // walk down
        for (CornerStitch* cur = right; cur; cur = cur->bottom()) {
            if (cur->left() == t && cur->getlly() >= splitY)
                cur->setLeft(top);
            else
                break;
        }
        // walk up
        for (CornerStitch* cur = right; cur; cur = cur->top()) {
            if (cur->left() == t && cur->getlly() >= splitY)
                cur->setLeft(top);
            else
                break;
        }
        
    }   
    CornerStitch* left = top->left();
    if (left) {
        // walk up
        for (CornerStitch* cur = left; cur; cur = cur->top()) {
            if (cur->right() == t && cur->getury() > splitY)
                cur->setRight(top);
            else
                break;
        }
        // walk down
        for (CornerStitch* cur = left; cur; cur = cur->bottom()) {
            if (cur->right() == t && cur->getury() > splitY)
                cur->setRight(top);
            else
                break;
        }
    }
    
    
    return top;
}

// Split tile t vertically at splitX (splitX is the llx of the new right tile)
// Returns pointer to the new right tile on success, nullptr on failure
CornerStitch* splitVertical(CornerStitch* t, long splitX, bool debug = false) {
    if (!t) return nullptr;

    long llx = t->getllx();
    long urx = t->geturx();

    if (!(llx < splitX && splitX < urx)) return nullptr;

    // create new right piece (inherits flags/attrs/net from t)
    CornerStitch* rightTile = new CornerStitch(splitX, t->getlly(),
                                               t->isSpace(), t->isVirt(),
                                               t->getAttr(), t->getNet());

    // === Fix pointers of right tile ===
    // Right tile inherits right, top pointers from t
    rightTile->setTop(t->top());
    rightTile->setRight(t->right());
    // Right tile has original tile as the left pointer
    rightTile->setLeft(t);
    // Find the new bottom pointer
    rightTile->setBottom(findTileContaining(t,splitX + 0.1, t->getlly() - 0.1));

    // === Fix pointers of original tile ===
    // Left, Bottom pointers remain same
    // Update the original tile's right pointer to point to the new right tile
    t->setRight(rightTile);
    // Find the new top pointer
    t->setTop(findTileContaining(t,splitX-0.1 ,t->getury() + 0.1));


    // === Neighbour fixes ===

    CornerStitch* rNbr = rightTile->right();
    if (rNbr) {
        // walk down
        for (CornerStitch* cur = rNbr; cur; cur = cur->bottom()) {
            if (cur->left() == t)
                cur->setLeft(rightTile);
            else
                break;
        }
        // walk up
        for (CornerStitch* cur = rNbr->top(); cur; cur = cur->top()) {
            if (cur->left() == t){
                cur->setLeft(rightTile);
            }
            else
                break;
        }
    }
    // If bottom neighbor had its right pointer pointing to t, update it to rightTile
    CornerStitch* below = rightTile->bottom();
    if (below) {
        // walk left
        for (CornerStitch* cur = below; cur; cur = cur->left()) {
            if (cur->top() == t && cur->geturx() > splitX)
                cur->setTop(rightTile);
            else
                break;
        }
        // walk right
        for (CornerStitch* cur = below->right(); cur; cur = cur->right()) {
            if (cur->top() == t && cur->geturx() > splitX)
                cur->setTop(rightTile);
            else
                break;
        }
    }

    // If top neighbor had its left pointer pointing to t, update it to rightTile
    CornerStitch* above = rightTile->top();
    if (above) {
        // walk left
        for (CornerStitch* cur = above; cur; cur = cur->left()) {
            if (cur->bottom() == t && cur->getllx() >= splitX)
                cur->setBottom(rightTile);
            else
                break;
        }
        // walk right
        for (CornerStitch* cur = above->right(); cur; cur = cur->right()) {
            if (cur->bottom() == t && cur->getllx() >= splitX)
                cur->setBottom(rightTile);
            else
                break;
        }
    }
        
    return rightTile;
}

// For each tile crossing the vertical line x, split it (creating a right piece)
bool splitVerticalEdge(CornerStitch* anchor, long x, long y0, long y1, bool debug = false) {
    if (!anchor || !(y0 < y1)) return false;
    long y = y0;
    const int MAX_STEPS = 100000;
    int steps = 0;

    while (y < y1) {
        if (++steps > MAX_STEPS) return false;
        // find tile that contains (x-1, y) — tile immediately left of the split at this y
        CornerStitch* leftTile = findTileContaining(anchor, x - 1, y+0.1);
        if (!leftTile) return false;

        // If the tile does not cross the split line (its right <= x), advance to its top
        if (!(leftTile->getllx() < x && x < leftTile->geturx())) {
            long topY = leftTile->getury();
            if (topY <= y) return false; // stuck
            y = topY;
            continue;
        }

        // split the leftTile at x (creates right piece with llx = x)
        CornerStitch* rightPiece = splitVertical(leftTile, x, debug);
        if (!rightPiece) return false;
        
        // After splitting, both leftTile and rightPiece share same vertical span.
        // Advance y to the top of that span.
        y = leftTile->getury();
    }
    return true;
}

// For each tile crossing the horizontal line y, split it (creating top pieces)
bool splitHorizontalEdge(CornerStitch* anchor, long y, long x0, long x1) {
    if (!anchor || !(x0 < x1)) return false;
    long x = x0;
    const int MAX_STEPS = 100000;
    int steps = 0;

    while (x < x1) {
        if (++steps > MAX_STEPS) return false;
        // find tile containing (x, y-1) — tile immediately below the split at this x
        CornerStitch* belowTile = findTileContaining(anchor, x, y - 1);
        if (!belowTile) return false;

        // If the tile does not cross the split line (its top <= y), advance to its right edge
        if (!(belowTile->getlly() < y && y < belowTile->getury())) {
            long rightX = belowTile->geturx();
            if (rightX <= x) return false; // stuck
            x = rightX;
            continue;
        }

        // split belowTile horizontally at y (creates top piece with lly = y)
        CornerStitch* topPiece = splitHorizontal(belowTile, y);
        if (!topPiece) return false;
        
        // Advance x to the right boundary of the tile we just processed
        x = belowTile->geturx();
    }
    return true;
}

// Insert rectangle [lx..lx+wx) x [ly..ly+wy) by aligning edges (edge-walk) then marking tiles inside as filled.
bool insertTileRect(CornerStitch* anchor,
                             long lx, long ly, long wx, long wy,
                             unsigned int attr = 0, unsigned int net = 0)
{
    if (!anchor || wx <= 0 || wy <= 0) return false;
    long rx = lx + wx;
    long ry = ly + wy;

    //if any tile that fully lies inside is already filled -> reject
    auto intersect0 = tilesInRect(anchor, lx, ly, wx, wy);
    for (auto t : intersect0) if (t && !t->isSpace()) return false;

     // Split horizontal edges first (bottom and top) for horizontal span [lx, rx)
    if (!splitHorizontalEdge(anchor, ly, lx, rx)) return false;
    if (!splitHorizontalEdge(anchor, ry, lx, rx)) return false;

    // Then split vertical edges along x = lx and x = rx for full vertical span [ly, ry)
    if (!splitVerticalEdge(anchor, lx, ly, ry)) return false;
    if (!splitVerticalEdge(anchor, rx, ly, ry)) return false;
    // Now collect tiles that intersect the rectangle (they should be aligned to the rectangle grid)
    auto finalTiles = tilesInRect(anchor, lx, ly, wx, wy);

    // Mark tiles fully inside rectangle as occupied (space=0) and set attr/net.
    for (auto t : finalTiles) {
        if (!t) continue;
        if (t->getllx() >= lx && t->getlly() >= ly && t->geturx() <= rx && t->getury() <= ry) {
            t->setSpace(0);
            t->setAttr(attr);
            t->setNet(net);
        } else {
            // If any overlapping but not fully-contained tile is already occupied -> collision
            if (!t->isSpace()) return false;
        }
    }

    return true;
}

//===========DEBUGGING HELPERS=============
struct IncomingRef {
    CornerStitch* owner;
    string via;
};

vector<IncomingRef>
findIncomingPointers(CornerStitch* anchor, CornerStitch* target) {
    vector<IncomingRef> refs;
    if (!anchor || !target) return refs;

    unordered_set<CornerStitch*> seen;
    queue<CornerStitch*> q;

    q.push(anchor);
    seen.insert(anchor);

    while (!q.empty()) {
        CornerStitch* t = q.front();
        q.pop();
        if (!t) continue;

        if (t->top() == target)
            refs.push_back({t, "top"});
        if (t->bottom() == target)
            refs.push_back({t, "bottom"});
        if (t->left() == target)
            refs.push_back({t, "left"});
        if (t->right() == target)
            refs.push_back({t, "right"});

        CornerStitch* nbrs[4] = {
            t->left(),
            t->right(),
            t->top(),
            t->bottom()
        };

        for (CornerStitch* nb : nbrs) {
            if (!nb) continue;
            if (seen.insert(nb).second)
                q.push(nb);
        }
    }
    return refs;
}
void dumpIncomingPointers(CornerStitch* anchor, CornerStitch* target) {
    auto refs = findIncomingPointers(anchor, target);

    cout << "\n==== INCOMING POINTER CHECK ====\n";
    cout << "Target tile @ " << target << "\n";
    cout << "  llx=" << target->getllx()
         << " lly=" << target->getlly()
         << " wx=" << (target->geturx() - target->getllx())
         << " wy=" << (target->getury() - target->getlly())
         << " layer=" << attrToLayer(target->getAttr())
         << "\n";
    if (refs.empty()) {
        cout << "  NONE\n";
        return;
    }
    cout << "Found Incoming pointers (" << refs.size() << "):\n";
    for (auto& r : refs) {
        CornerStitch* t = r.owner;

        long llx = t->getllx();
        long lly = t->getlly();
        long wx  = t->geturx() - t->getllx();
        long wy  = t->getury() - t->getlly();

        cout << "  Tile @" << t
             << " via " << r.via << "\n"
             << "    llx=" << llx
             << " lly=" << lly
             << " wx=" << wx
             << " wy=" << wy
             << " layer=" << attrToLayer(t->getAttr())
             << "\n";
    }

    cout << "==== END CHECK ====\n";

}



//============ MERGING =================
//============ SIMPLE MERGE HELPERS ==================
bool mergeHorizontalOnce(CornerStitch* left) {
    if (!left) return false;

    CornerStitch* right = left->right();
    if (!right) return false;

    if (right->left() != left) return false;
    if (left->getAttr() != right->getAttr()) return false;
    if (left->isSpace() != right->isSpace()) return false;
    if (left->getlly() != right->getlly() ||
        left->getury() != right->getury()) return false;
    
    left->setRight(right->right());
    left->setTop(right->top());


    // ---- tiles ABOVE right ----
    for (CornerStitch* t = right->top(); t; t = t->left()) {
        if (t->bottom() == right)
            t->setBottom(left);
        else
            break;
    }
    for (CornerStitch* t = right->top()->right(); t; t = t->right()) {
        if (t->bottom() == right)
            t->setBottom(left);
        else
            break;
    }

    // ---- tiles BELOW right ----
    for (CornerStitch* t = right->bottom(); t; t = t->right()) {
        if (t->top() == right)
            t->setTop(left);
        else
            break;
    }
    for (CornerStitch* t = right->bottom()->left(); t; t = t->left()) {
        if (t->top() == right)
            t->setTop(left);
        else
            break;
    }

    // ---- tiles to the RIGHT of right ----
    for (CornerStitch* t = right->right(); t; t = t->bottom()) {
        if (t->left() == right)
            t->setLeft(left);
        else
            break;
    }
    // cout << "Deleting Right: " << right << "\n";
    // dumpIncomingPointers(left,right);
    delete right;
    return true;
}

bool mergeVerticalOnce(CornerStitch* bottom) {
    if (!bottom) return false;

    CornerStitch* top = bottom->top();
    if (!top) return false;
    if (top->bottom() != bottom) return false;
    if (bottom->getAttr() != top->getAttr()) return false;
    if (bottom->isSpace() != top->isSpace()) return false;
    if (bottom->getllx() != top->getllx() ||
        bottom->geturx() != top->geturx()) return false;

    // 1. Fix outgoing pointers
    bottom->setTop(top->top());
    bottom->setRight(top->right());
    // ---- tiles above top ----
    for (CornerStitch* t = top->top(); t; t = t->left()) {
        if (t->bottom() == top)
            t->setBottom(bottom);
        else
            break;
    }
    if (top->top()){
        for (CornerStitch* t = top->top()->right(); t; t = t->right()) {
            if (t->bottom() == top)
                t->setBottom(bottom);
            else
                break;
        }
    }

    // ---- tiles to left of top ----
    for (CornerStitch* t = top->left(); t; t = t->top()) {
        if (t->right() == top)
            t->setRight(bottom);
        else
            break;
    }

    // ---- tiles to right of top ----
    for (CornerStitch* t = top->right(); t; t = t->bottom()) {
        if (t->left() == top)
            t->setLeft(bottom);
        else
            break;
    }
    // cout << "Deleting Top: " << top << "\n";
    // dumpIncomingPointers(bottom, top);
    delete top;
    return true;
}

//================== MERGING TO MAXIMIZE HORIZONTAL SPAN ==========================
bool splitRightToMatchLeft(CornerStitch* anchor, CornerStitch* left, CornerStitch* right) {
    if (!anchor || !left || !right) return false;
    if (left->right() != right) return false;
    if (left->getAttr() != right->getAttr()) return false;
    else if (left->isSpace() != right->isSpace()) return false;

    long ly = left->getlly();
    long uy = left->getury();
    long x0 = right->getllx();
    long x1 = right->geturx();

    // If already identical vertical spans, nothing to do.
    if (ly == right->getlly() && uy == right->getury()) return true;

    // Split right column only where required
    if (right->getlly() < ly && ly < right->getury()) {
        if (!splitHorizontalEdge(anchor, ly, x0, x1)) return false;
    }
    if (right->getlly() < uy && uy < right->getury()) {
        if (!splitHorizontalEdge(anchor, uy, x0, x1)) return false;
    }
    return true;
}

bool alignAndMergeAdjacent(CornerStitch* anchor, CornerStitch* left, CornerStitch* right, bool debug = false) {
    if (!anchor || !left || !right) return false;
    if (left->right() != right) return false;
    if (left->getAttr() != right->getAttr()) return false;
    else if (left->isSpace() != right->isSpace()) return false;

    // ensure right column has a slice that matches left's vertical span
    if (!splitRightToMatchLeft(anchor, left, right)) return false;


    // probe a point inside the matching vertical span, but inside the right column
    long probeX = right->getllx() + 0.1;
    long probeY = left->getlly() + 0.1;
    CornerStitch* matching = findTileContaining(anchor, probeX, probeY);
    if (!matching) {
        // try nudging probe a bit to the right
        probeX = right->getllx() + max(1L, (right->geturx() - right->getllx())/4);
        matching = findTileContaining(anchor, probeX, probeY);
        if (!matching) return false;
    }
    
    // verify the found tile lives in right column and matches vertical span
    if (matching->getllx() != right->getllx() || matching->getlly() != left->getlly() || matching->getury() != left->getury())
        return false;



    // Now the tile immediately left of 'matching' should be a tile we can merge with.
    CornerStitch* leftOfMatching = matching->left();
    if (!leftOfMatching) return false;



    // If leftOfMatching exactly matches left's span and is space, do a single merge.
    if ((leftOfMatching->getAttr() == matching->getAttr()) && (leftOfMatching->isSpace() == matching->isSpace()) &&
        leftOfMatching->getlly() == matching->getlly() &&
        leftOfMatching->getury() == matching->getury()) {
        return mergeHorizontalOnce(leftOfMatching);
    }

    // Otherwise, if leftOfMatching is not directly the original 'left' but still a contiguous run,
    // attempt to walk leftwards until we find a candidate tile that is adjacent and has same span.
    CornerStitch* cur = leftOfMatching;
    while (cur && cur->right() != matching) {
        cur = cur->right();
        if (!cur) break;
        // safety guard: if we get past matching llx, bail
        if (cur->getllx() >= matching->getllx()) break;
    }
    if (!cur) return false;
    // cur->right() == matching means cur is immediately to left of matching
    if (cur->isSpace() && matching->isSpace() &&
        cur->getlly() == matching->getlly() && cur->getury() == matching->getury()) {
        return mergeHorizontalOnce(cur);
    }
    return false;
}


int passAlignAndMergeHorizontally(CornerStitch* anchor) {
    if (!anchor) return 0;
    unordered_set<CornerStitch*> seen;
    deque<CornerStitch*> dq;
    dq.push_back(anchor);
    seen.insert(anchor);
    int merges = 0;

    while (!dq.empty()) {
        CornerStitch* t = dq.front(); dq.pop_front();
        CornerStitch* r = t->right();
        if (r) {
            if (alignAndMergeAdjacent(anchor, t, r)) {
                ++merges;
                seen.clear();
                dq.clear();
                dq.push_back(anchor);
                seen.insert(anchor);
                continue;
            }
        }
        CornerStitch* nbrs[4] = { t->left(), t->right(), t->bottom(), t->top() };
        for (auto nb : nbrs) {
            if (!nb) continue;
            if (seen.insert(nb).second) dq.push_back(nb);
        }
    }
    return merges;
}

// One vertical pass: try mergeVerticalOnce on every tile reachable.
int passMergeVerticalSimple(CornerStitch* anchor) {
    if (!anchor) return 0;
    unordered_set<CornerStitch*> seen;
    deque<CornerStitch*> dq;
    dq.push_back(anchor);
    seen.insert(anchor);
    int merges = 0;
    
    while (!dq.empty()) {
        CornerStitch* t = dq.front(); dq.pop_front();
        if (mergeVerticalOnce(t)) {
            ++merges;
            seen.clear();
            dq.clear();
            dq.push_back(anchor);
            seen.insert(anchor);
            continue;
        }
        CornerStitch* nbrs[4] = { t->left(), t->right(), t->bottom(), t->top() };
        for (auto nb : nbrs) {
            if (!nb) continue;
            if (seen.insert(nb).second) dq.push_back(nb);
        }
    }
    return merges;
}

// Top-level coalesce used by deletion: horizontal-align+merge passes followed by normal vertical passes.
// Repeat until stable (or until maxRounds reached).
int coalesceAfterDeletion(CornerStitch* anchor, int maxRounds = 20) {
    if (!anchor) return 0;
    int total = 0;
    for (int r = 0; r < maxRounds; ++r) {
        int h = passAlignAndMergeHorizontally(anchor);
        int v = passMergeVerticalSimple(anchor);
        total += h + v;
        if (h == 0 && v == 0) break;
    }
    return total;
}

// Delete a single tile 't' (mark space) and then coalesce around anchor.
bool deleteTileAndCoalesce(CornerStitch* anchor, CornerStitch* t) {
    if (!anchor || !t) return false;
    t->setSpace(1); // mark as free
    t->setAttr(0);
    // try to merge/align until stable
    coalesceAfterDeletion(anchor, 50);
    return true;
}

bool deleteRectAndCoalesce(CornerStitch* anchor, long lx, long ly, long wx, long wy) {
    if (!anchor) return false;
    auto list = tilesInRect(anchor, lx, ly, wx, wy);
    if (list.empty()) return false;
    for (auto tt : list) {
        if (!tt) continue;
        if (tt->getllx() >= lx && tt->getlly() >= ly &&
            tt->geturx() <= lx + wx && tt->getury() <= ly + wy) {
            tt->setSpace(1);
            tt->setAttr(0);
        }
    }
    coalesceAfterDeletion(anchor, 50);
    return true;
}



bool moveTile(
    CornerStitch* root,
    CornerStitch* tile,
    long dx,
    long dy
) {
    if (!root || !tile) return false;
    if (tile->isSpace()) return false;  // only move filled tiles

   
    long old_lx = tile->getllx();
    long old_ly = tile->getlly();
    long wx = tile->geturx() - tile->getllx();
    long wy = tile->getury() - tile->getlly();

    long new_lx = old_lx + dx;
    long new_ly = old_ly + dy;

  
    auto destTiles = tilesInRect(root, new_lx, new_ly, wx, wy);
    for (auto t : destTiles) {
        // allow overlap with itself (important!)
        if (!t->isSpace() && t != tile)
            return false;
    }

    // 3. Delete old tile (edge-walk delete)
    if (!deleteRectAndCoalesce(root, old_lx, old_ly, wx, wy))
        return false;

    // 4. Insert at new location
    if (!insertTileRect(root, new_lx, new_ly, wx, wy,
                        tile->getAttr(), tile->getNet()))
        return false;

    return true;
}

bool bloatByRect(CornerStitch* &t, unsigned long bloat_right=0, unsigned long bloat_left=0, unsigned long bloat_bottom=0, unsigned long bloat_top=0){
    if(!t) return false;
    if(bloat_right == 0 && bloat_left == 0 && bloat_bottom == 0 && bloat_top == 0) return true;
    long urx=t->geturx(), ury=t->getury(), llx=t->getllx(), lly=t->getlly();
    Rectangle* probeR = new Rectangle(urx, lly - bloat_bottom, bloat_right, ury + bloat_top - lly + bloat_bottom);
    Rectangle* probeL = new Rectangle(llx - bloat_left, lly - bloat_bottom, bloat_left, ury + bloat_top - lly + bloat_bottom);
    Rectangle* probeT = new Rectangle(llx, ury, urx - llx, bloat_top);
    Rectangle* probeB = new Rectangle(llx, lly - bloat_bottom, urx - llx, bloat_top);

    for(auto probe : {probeR,probeL,probeT,probeB}){
        if (!splitHorizontalEdge(t, probe->lly(), probe->llx(), probe->llx() + probe->wx())) return false;
        if (!splitHorizontalEdge(t, probe->lly() + probe->wy(), probe->llx(), probe->llx() + probe->wx())) return false;
        if (!splitVerticalEdge(t, probe->llx(), probe->lly(), probe->lly() + probe->wy())) return false;
        if (!splitVerticalEdge(t, probe->llx() + probe->wx(), probe->lly(), probe->lly() + probe->wy())) return false;
        for(auto tile : tilesInRect(t,probe->llx(),probe->lly(),probe->wx(),probe->wy())){
            if(!tile || !tile->isSpace()) continue;
            tile->setSpace(0);
            tile->setAttr(t->getAttr());
            tile->setNet(t->getNet());
        }
    }
    // coalesceAfterDeletion(t,50);
    return true;
}

bool electricallyAdjacent(CornerStitch* a, CornerStitch* b) {
    if (!a || !b) return false;
    if (a->isSpace() || b->isSpace()) return false;
    if (a->getNet() != b->getNet()) return false;

    long ax0 = a->getllx();
    long ay0 = a->getlly();
    long ax1 = a->geturx();
    long ay1 = a->getury();

    long bx0 = b->getllx();
    long by0 = b->getlly();
    long bx1 = b->geturx();
    long by1 = b->getury();

    // Touch OR overlap in X
    bool xTouch = !(ax1 < bx0 || bx1 < ax0);
    // Touch OR overlap in Y
    bool yTouch = !(ay1 < by0 || by1 < ay0);

    if (xTouch && yTouch)
        return true;
    
    unordered_set<CornerStitch*> visited;
    deque<CornerStitch*> q;

    q.push_back(a);
    visited.insert(a);

    while (!q.empty()) {
        CornerStitch* cur = q.front();
        q.pop_front();

        if (cur == b)
            return true;

        CornerStitch* nbrs[4] = {
            cur->left(),
            cur->right(),
            cur->top(),
            cur->bottom()
        };

        for (CornerStitch* n : nbrs) {
            if (!n) continue;
            if (n->isSpace()) continue;
            if (n->getNet() != a->getNet()) continue;

            if (visited.insert(n).second) {
                q.push_back(n);
            }
        }
    }

    return false;
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


void exportTiles(CornerStitch* anchor, const string& filename, bool showInTerminal = false) {
    if (!anchor) { cout << "[]\n"; return; }

    const long VIS_MIN = -80;   // visualization clamp
    const long VIS_MAX =  80;

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
    if(showInTerminal)
        cout << "[\n";
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
        if(showInTerminal){
            cout << "  {"
                << "\"name\":\"T" << i << "\","
                << "\"llx\":" << llx << ","
                << "\"lly\":" << lly << ","
                << "\"wx\":" << wx << ","
                << "\"wy\":" << wy << ","
                << "\"filled\":" << (filled ? "True" : "False") << ","
                << "\"net\":" << t->getNet() << ","
                << "\"layer\":" << attrToLayer(t->getAttr())
                << "}";
        }
        fout << "  {"
             << "\"name\":\"T" << i << "\","
             << "\"llx\":" << llx << ","
             << "\"lly\":" << lly << ","
             << "\"wx\":" << wx << ","
             << "\"wy\":" << wy << ","
             << "\"filled\":" << (filled ? "True" : "False") << ","
             << "\"net\":" << t->getNet() << ","
             << "\"layer\":\"" << attrToLayer(t->getAttr()) << "\""
             << "}";

        if (i + 1 < tiles.size()){
            if(showInTerminal)
                cout << ",";
            fout << ",";
        }
        if(showInTerminal)
            cout << "\n";
        fout << "\n";
    }
    if(showInTerminal)
        cout << "]\n";
    fout << "]\n";
}