// Author: Stavan Mehta, Affaan Fakih
// Version: Pilot Initial (V0)

#include <bits/stdc++.h>
using namespace std;

#define MIN_VALUE (numeric_limits<long>::min())
#define MAX_VALUE (numeric_limits<long>::max())

#define DEBUG_MODE false
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

        void setVirt(unsigned int v) { virt = v; }
        void setSpace(unsigned int s) { space = s; if(s == 1) {attr = 0; net = 0;}}
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

vector<CornerStitch*> rightNeighbors(CornerStitch* t) {
    vector<CornerStitch*> result;
    if (!t) return result;

    long x = t->geturx() ;
    long y0 = t->getlly() ;
    unsigned long h  = t->getury() - y0;

    // 1-unit wide vertical slab to the right
    result = tilesInRect(t, x, y0, 1, h);
    return result;
}

vector<CornerStitch*> leftNeighbors(CornerStitch* t) {
    vector<CornerStitch*> result;
    if (!t) return result;

    long x = t->getllx() - 1;
    long y0 = t->getlly();
    unsigned long h  = t->getury() - y0;

    result = tilesInRect(t, x, y0, 1, h);
    return result;
}

vector<CornerStitch*> topNeighbors(CornerStitch* t) {
    vector<CornerStitch*> result;
    if (!t) return result;

    long y = t->getury();
    long x0 = t->getllx();
    unsigned long w  = t->geturx() - x0;

    result = tilesInRect(t, x0, y, w, 1);
    return result;
}

vector<CornerStitch*> bottomNeighbors(CornerStitch* t) {
    vector<CornerStitch*> result;
    if (!t) return result;

    long y = t->getlly() - 1;
    long x0 = t->getllx();
    unsigned long w  = t->geturx() - x0;

    result = tilesInRect(t, x0, y, w, 1);
    return result;
}


// Split tile t horizontally at splitY (splitY is the lly of the new top tile).
// Returns pointer to the new top tile on success, nullptr on failure.
bool splitHorizontal(CornerStitch* t, long splitY) {
    if (!t) return false;                     
    long a = t->getlly();
    long b = t->getury();                      
    if (!(a < splitY && splitY < b)) return false;

    vector<CornerStitch*> rightTiles = rightNeighbors(t);
    vector<CornerStitch*> leftTiles = leftNeighbors(t);
    vector<CornerStitch*> topTiles = topNeighbors(t);
    CornerStitch* top = new CornerStitch(t->getllx(), splitY, t->isSpace(), t->isVirt(), t->getAttr(), t->getNet());

    // === Fix pointers of top tile ===
    // Top tile inherits right, top pointers from t
    top->setTop(t->top());
    top->setRight(t->right());
    // Top tile has original tile as the bottom
    top->setBottom(t);
    // Find the new left pointer
    top->setLeft(findTileContaining(t,t->getllx() - 1 ,splitY));


    // === Fix pointers of original tile ===
    // Left, Bottom pointers remain same
    // Update the original tile's top pointer to point to the new top tile
    t->setTop(top);
    // Find the new right pointer
    t->setRight(findTileContaining(t,t->geturx() ,splitY-1));


    // Fix neighbors above the top tile if they used to point at t for their bottom links
    for (auto tile : topTiles){
        if(!tile) continue;
        if(tile->bottom() == t){
            tile->setBottom(top);
        }
    }
    for (auto tile : rightTiles){
        if(!tile) continue;
        if(tile->left() == t){
            if(tile->getlly() >= splitY) tile->setLeft(top);
        }
    }
    for (auto tile : leftTiles){
        if(!tile) continue;
        if(tile->right() == t){
            if(tile->getury() > splitY) tile->setRight(top);
        }
    }
    return true;
}

// Split tile t vertically at splitX (splitX is the llx of the new right tile)
// Returns pointer to the new right tile on success, nullptr on failure
bool splitVertical(CornerStitch* t, long splitX) {
    if (!t) return false;

    long llx = t->getllx();
    long urx = t->geturx();

    if (!(llx < splitX && splitX < urx)) return false;

    vector<CornerStitch*> rightTiles = rightNeighbors(t);
    vector<CornerStitch*> bottomTiles = bottomNeighbors(t);
    vector<CornerStitch*> topTiles = topNeighbors(t);

    // create new right piece (inherits flags/attrs/net from t)
    CornerStitch* rightTile = new CornerStitch(splitX, t->getlly(),
                                               t->isSpace(), t->isVirt(),
                                               t->getAttr(), t->getNet());

    // === Fix pointers of right tile ===
    // Right tile inherits right, top pointers from t
    rightTile->setBottom(findTileContaining(t,splitX, t->getlly() - 1));
    rightTile->setTop(t->top());
    rightTile->setRight(t->right());
    // Right tile has original tile as the left pointer
    rightTile->setLeft(t);
    // Find the new bottom pointer

    // === Fix pointers of original tile ===
    // Left, Bottom pointers remain same
    // Update the original tile's right pointer to point to the new right tile
    t->setTop(findTileContaining(t,splitX-1 ,t->getury()));
    t->setRight(rightTile);
  


    // === Neighbour fixes ===
    for (auto tile : rightTiles){
        if(!tile) continue;
        if(tile->left() == t){
            tile->setLeft(rightTile);
        }
    }

    for (auto tile : bottomTiles){
        if(!tile) continue;
        if(tile->top() == t){
            if(tile->geturx() > splitX) tile->setTop(rightTile);
        }
    }
    for (auto tile : topTiles){
        if(!tile) continue;
        if(tile->bottom() == t){
            if(tile->getllx() >= splitX) tile->setBottom(rightTile);
        }
    }
        
    return true;
}

// For each tile crossing the vertical line x, split it (creating a right piece)
bool splitVerticalEdge(CornerStitch* anchor, long x, long y0, long y1) {
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
        
        if (!splitVertical(leftTile, x)) return false;
        
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
        if (!splitHorizontal(belowTile, y)) return false;
        
        x = belowTile->geturx();
    }
    return true;
}

unordered_set<CornerStitch*> deleted;

//============ MERGING =================
//============ SIMPLE MERGE HELPERS ==================
bool mergeHorizontalOnce(CornerStitch* left,  CornerStitch* &anchor) {
    if (!left) return false;
    CornerStitch* right = left->right();
    if (!right) return false;
    if (left->getNet() != right->getNet()) return false;    
    if (left->getAttr() != right->getAttr()) return false;
    if (left->isSpace() != right->isSpace()) return false;
    if (left->getlly() != right->getlly() || left->getury() != right->getury()) return false;

    
    vector<CornerStitch*> rightTiles = rightNeighbors(right);
    vector<CornerStitch*> topTiles = topNeighbors(right);
    vector<CornerStitch*> bottomTiles = bottomNeighbors(right);

    left->setRight(right->right());
    left->setTop(right->top());


    for (auto nbr : rightTiles) {
        if(!nbr) continue;
        if (nbr->left() == right) nbr->setLeft(left);
    }
    for (auto nbr : topTiles) {
        if(!nbr) continue;
        if (nbr->bottom() == right) nbr->setBottom(left);
    }
    for (auto nbr : bottomTiles) {
        if(!nbr) continue;
        if (nbr->top() == right) nbr->setTop(left);
    }

    if(right == anchor) anchor = left;
    deleted.insert(right);

    delete right;
    return true;
}

bool mergeVerticalOnce(CornerStitch* bottom, CornerStitch* &anchor){
    if (!bottom) return false;
    CornerStitch* top = bottom->top();
    if (!top) return false;
    if (bottom->getNet() != top->getNet()) return false;    
    if (bottom->getAttr() != top->getAttr()) return false;
    if (bottom->isSpace() != top->isSpace()) return false;
    if (bottom->getllx() != top->getllx() || bottom->geturx() != top->geturx()) return false;

    
    
    vector<CornerStitch*> rightTiles = rightNeighbors(top);
    vector<CornerStitch*> topTiles = topNeighbors(top);
    vector<CornerStitch*> leftTiles = leftNeighbors(top);
    bottom->setRight(top->right());
    bottom->setTop(top->top());

    for (auto nbr : rightTiles) {
        if(!nbr) continue;
        if (nbr->left() == top) nbr->setLeft(bottom);
    }
    for (auto nbr : topTiles) {
        if(!nbr) continue;
        if (nbr->bottom() == top) nbr->setBottom(bottom);
    }
    for (auto nbr : leftTiles) {
        if(!nbr) continue;
        if (nbr->right() == top) nbr->setRight(bottom);
    }

    if(top == anchor) anchor = bottom;
    deleted.insert(top);
 
    delete top;
    return true;
}

//================== MERGING TO MAXIMIZE HORIZONTAL SPAN ==========================
bool splitRightToMatchLeft(CornerStitch* &anchor, CornerStitch* left, CornerStitch* right) {
    if (!anchor || !left || !right) return false;
    if (left->right() != right) return false;
    if (left->getNet() != right->getNet()) return false;    
    if (left->getAttr() != right->getAttr()) return false;
    if (left->isSpace() != right->isSpace()) return false;
    long ly = left->getlly();
    long uy = left->getury();
    long x0 = right->getllx();
    long x1 = right->geturx();

    // If already identical vertical spans, nothing to do.
    if (ly == right->getlly() && uy == right->getury()) return true;

    // Split right column only where required
    if (right->getlly() <= ly && ly <= right->getury()) {
        if (!splitHorizontalEdge(anchor, ly, x0, x1)) return false;
    }
    else return false;
    if (right->getlly() <= uy && uy <= right->getury()) {
        if (!splitHorizontalEdge(anchor, uy, x0, x1)) return false;
    }
    else return false;
    return true;
}

bool alignAndMergeAdjacent(CornerStitch* &anchor, CornerStitch* left, CornerStitch* right) {
    if (!anchor || !left || !right) return false;
    if (left->right() != right) return false;
    if (left->getNet() != right->getNet()) return false;    
    if (left->getAttr() != right->getAttr()) return false;
    if (left->isSpace() != right->isSpace()) return false;
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

    // If leftOfMatching exactly matches left's span, do a single merge.
    if (leftOfMatching->getlly() == matching->getlly() &&
        leftOfMatching->getury() == matching->getury()) {
        return mergeHorizontalOnce(leftOfMatching, anchor);
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
    if (cur->getlly() == matching->getlly() && cur->getury() == matching->getury()) {
        return mergeHorizontalOnce(cur, anchor);
    }
    return false;
}


int passAlignAndMergeHorizontally(CornerStitch* &anchor) {
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
                for(auto tile : dq) if(deleted.count(tile)) dq.erase(find(begin(dq),end(dq),tile));
                // after merge t may have expanded; try merging t again immediately
                dq.push_front(t);
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
int passMergeVerticalSimple(CornerStitch* &anchor) {
    if (!anchor) return 0;
    unordered_set<CornerStitch*> seen;
    deque<CornerStitch*> dq;
    dq.push_back(anchor);
    seen.insert(anchor);
    int merges = 0;

    while (!dq.empty()) {
        CornerStitch* t = dq.front(); dq.pop_front();
        if (mergeVerticalOnce(t, anchor)) {
            for(auto tile : dq) if(deleted.count(tile)) dq.erase(find(begin(dq),end(dq),tile));
            ++merges;
            dq.push_front(t);
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
int coalesce(CornerStitch* &anchor, int maxRounds = 20) {
    if (!anchor) return 0;
    int total = 0;
    for (int r = 0; r < maxRounds; ++r) {
        int v = passMergeVerticalSimple(anchor);
        int h = passAlignAndMergeHorizontally(anchor);
        total += h + v;
        if (h == 0 && v == 0) break;
    }

    deleted.clear();
    return total;
}

// Delete a single tile 't' (mark space) and then coalesce around anchor.
bool deleteTileAndCoalesce(CornerStitch* &anchor, CornerStitch* t) {
    if (!anchor || !t) return false;
    t->setSpace(1); // mark as free
    // try to merge/align until stable
    coalesce(anchor, 50);
    return true;
}

bool deleteRectAndCoalesce(CornerStitch* &anchor, long lx, long ly, long wx, long wy) {
    if (!anchor) return false;
    auto list = tilesInRect(anchor, lx, ly, wx, wy);
    if (list.empty()) return false;
    for (auto tt : list) {
        if (!tt) continue;
        if (tt->getllx() >= lx && tt->getlly() >= ly &&
            tt->geturx() <= lx + wx && tt->getury() <= ly + wy) {
            tt->setSpace(1);
        }
    }
    coalesce(anchor, 50);
    return true;
}

// Insert rectangle [lx..lx+wx) x [ly..ly+wy) by aligning edges (edge-walk) then marking tiles inside as filled.
bool insertTileRect(CornerStitch* &anchor,
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

    coalesce(anchor,50);
    return true;
}

bool bloatByRect(CornerStitch* &t, unsigned long drc_right=0, unsigned long drc_left=0, unsigned long drc_bottom=0, unsigned long drc_top=0){
    if(!t) return false;
    if(drc_right == 0 && drc_left == 0 && drc_bottom == 0 && drc_top == 0) return true;
    long urx=t->geturx(), ury=t->getury(), llx=t->getllx(), lly=t->getlly();
    Rectangle* probeR = new Rectangle(urx, lly - drc_bottom, drc_right, ury + drc_top - lly + drc_bottom);
    Rectangle* probeL = new Rectangle(llx - drc_left, lly - drc_bottom, drc_left, ury + drc_top - lly + drc_bottom);
    Rectangle* probeT = new Rectangle(llx, ury, urx - llx, drc_top);
    Rectangle* probeB = new Rectangle(llx, lly - drc_bottom, urx - llx, drc_top);

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
    coalesce(t,50);
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

enum LayerType {
    L_NONE = 0,
    L_NDIFF,
    L_PDIFF,
    L_NTRANS,
    L_PTRANS,
    L_POLY,
    L_M1,
    L_M2
};
#define L_PTR_LEFT   50
#define L_PTR_RIGHT  51
#define L_PTR_TOP    52
#define L_PTR_BOTTOM 53


string attrToLayer(unsigned int attr) {
    switch (attr) {
        case L_NDIFF:       return "ndiff";
        case L_PDIFF:       return "pdiff";
        case L_NTRANS:      return "ntransistor";
        case L_PTRANS:      return "ptransistor";
        case L_POLY:        return "polysilicon";
        case L_M1:          return "m1";
        case L_M2:          return "m2";
        case L_PTR_LEFT:    return "left";
        case L_PTR_RIGHT:   return "right";
        case L_PTR_TOP:     return "top";
        case L_PTR_BOTTOM:  return "bottom";
        default:            return "none";
    }
}


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
            cout << ",";
            fout << ",";}

        cout << "\n";
        fout << "\n";
    }
    cout << "]\n";
    fout << "]\n";
}

