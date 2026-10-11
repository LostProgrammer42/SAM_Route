#pragma once
#include <bits/stdc++.h>

enum DbgCat : unsigned {
    D_NONE    = 0,
    D_SUMMARY = 1u << 0,   // per-layer / per-net results: "fully routed", "Switch routing succeeded", final stats
    D_PAIRS   = 1u << 1,   // which route pairs are generated and tried
    D_PORTS   = 1u << 2,   // SRC / DST port lists
    D_SEARCH  = 1u << 3,   // bidirectional search: fails, path pieces, "Routed"
    D_VIA     = 1u << 4,   // via sites, anchors, placement, undo
    D_SWITCH  = 1u << 5,   // switch routing decisions
    D_CONN    = 1u << 6,   // connectivity checks ("Not routing Electrically Adjacent", ...)
    D_PLANES  = 1u << 7,   // plane dumps / exports
    D_ALL     = 0xFFFFFFFFu
};

inline unsigned dbgMask = D_ALL;

inline unsigned parseDbg(const std::string& s) {
    static const std::map<std::string, unsigned> names = {
        {"summary", D_SUMMARY}, {"pairs", D_PAIRS}, {"ports", D_PORTS}, {"search", D_SEARCH},
        {"via", D_VIA}, {"switch", D_SWITCH}, {"conn", D_CONN}, {"planes", D_PLANES},
        {"all", D_ALL}, {"none", D_NONE}};
    unsigned m = 0;
    std::stringstream ss(s); std::string tok;
    while (std::getline(ss, tok, ',')) {
        bool minus = !tok.empty() && tok[0] == '-';
        if (minus) tok.erase(0, 1);
        auto it = names.find(tok);
        if (it == names.end()) { std::cerr << "SAM_DEBUG: unknown category '" << tok << "'\n"; continue; }
        if (minus) m &= ~it->second; else m |= it->second;
    }
    return m;
}
inline void initDebug() { if (const char* e = std::getenv("SAM_DEBUG")) dbgMask = parseDbg(e); }

// Turns "cond ? (void)0 : stream << ..." into a valid expression statement (no if/else, so no dangling-else).
struct DbgVoid { void operator&(std::ostream&) const {} };

#ifdef SAM_NO_DEBUG
  #define DBG(cat) true ? (void)0 : DbgVoid() & std::cout
#else
  #define DBG(cat) !(dbgMask & (cat)) ? (void)0 : DbgVoid() & std::cout
#endif

// ---- Run statistics: how good is the routing, not just did it finish ----
struct RunStats {
    long length[3] = {0, 0, 0};   // routed centreline length per plane (0 poly, 1 LI, 2 M1)
    long bends[3]  = {0, 0, 0};   // direction changes per plane
    long routes[3] = {0, 0, 0};   // committed routes per plane
    long deadFronts = 0;          // searches that ran out of tiles
};
inline RunStats runStats;