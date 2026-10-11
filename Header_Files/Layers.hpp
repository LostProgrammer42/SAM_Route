// Layers.hpp
// Layer IDs, layer-name lookups, per-layer rule values, and the net-name <-> net-ID registry.
// Depends on nothing else in the project.
#pragma once
#include <bits/stdc++.h>
#include "Debug.hpp"

// ---------------- Layer IDs ("attr" stored in each tile) ----------------
// Must fit in 6 bits (CornerStitch::attr). L_NONE = 0 means "no layer".
enum LayerType {
    L_NONE = 0,
    L_NDIFF,
    L_PDIFF,
    L_NTRANS,
    L_PTRANS,
    L_POLY,
    L_LI,
    L_M1,
    L_M2,
    L_POLYCON,
    L_LIBRDRCON,
    L_MCON,

    // Debug-only markers. Tagging a tile with one of these makes the visualizer
    // highlight it as a left/right/top/bottom stitch neighbour. Not used by the router.
    L_PTR_LEFT   = 50,
    L_PTR_RIGHT  = 51,
    L_PTR_TOP    = 52,
    L_PTR_BOTTOM = 53
};

// Plane index of each layer. Plane 0: diffusion/transistors/poly, 1: li, 2: m1.
// Returns -1 for unknown names.
inline int layerNameToPlane(const std::string& layer) {
    if (layer == "ndiff" || layer == "pdiff" || layer == "ndiffusion" || layer == "pdiffusion" ||
        layer == "ntransistor" || layer == "ptransistor")
        return 0; // diffusion, transistors

    if (layer == "polysilicon")
        return 0; // poly

    if (layer == "li")
        return 1; // local interconnect

    if (layer == "m1" || layer == "mcon")
        return 2; // metal1

    if (layer == "m2")
        return 3; // metal2 (no plane is allocated for this yet)

    if (layer == "polycon")
        return 0;
    if (layer == "librdrcon")
        return 1;

    return -1;
}

inline unsigned int layerNameToAttr(const std::string& layer) {
    if (layer == "ndiff" || layer == "ndiffusion") return L_NDIFF;
    if (layer == "pdiff" || layer == "pdiffusion") return L_PDIFF;
    if (layer == "ntransistor")  return L_NTRANS;
    if (layer == "ptransistor")  return L_PTRANS;
    if (layer == "polysilicon")  return L_POLY;
    if (layer == "li")           return L_LI;
    if (layer == "m1")           return L_M1;
    if (layer == "m2")           return L_M2;
    if (layer == "polycon")      return L_POLYCON;
    if (layer == "librdrcon")    return L_LIBRDRCON;
    if (layer == "mcon")         return L_MCON;

    return L_NONE;
}

inline std::string attrToLayerName(unsigned int attr) {
    switch (attr) {
        case L_NDIFF:       return "ndiff";
        case L_PDIFF:       return "pdiff";
        case L_NTRANS:      return "ntransistor";
        case L_PTRANS:      return "ptransistor";
        case L_POLY:        return "polysilicon";
        case L_LI:          return "li";
        case L_M1:          return "m1";
        case L_M2:          return "m2";
        case L_POLYCON:     return "polycon";
        case L_LIBRDRCON:   return "librdrcon";
        case L_MCON:        return "mcon";
        case L_PTR_LEFT:    return "left";
        case L_PTR_RIGHT:   return "right";
        case L_PTR_TOP:     return "top";
        case L_PTR_BOTTOM:  return "bottom";
        default:            return "none";
    }
}

// Clearance (in tile units) to keep around shapes on each layer when bloating.
inline unsigned long layerBloat(const std::string& layer) {
    if (layer == "ndiff" || layer == "pdiff" || layer == "ndiffusion" || layer == "pdiffusion") return 2;
    if (layer == "ntransistor" || layer == "ptransistor") return 0;
    if (layer == "polysilicon") return 3;
    if (layer == "li") return 5;
    if (layer == "m1") return 3;
    if (layer == "m2") return 0;
    return 0;
}

// Wire width used when the router inserts a path segment on this layer.
inline unsigned long routeWidth(unsigned int attr) {
    if (attr == L_LI) return 3;
    return 2;
}

// ---------------- Nets ----------------
// Net 0 is reserved for "no net" ("#" in .rect files). IDs must fit in 6 bits (max 63).
constexpr unsigned int NET_NONE = 0;

inline std::unordered_map<std::string, std::string> ioMap;     // net name -> "rect"/"inrect"/"outrect"
inline std::unordered_map<std::string, unsigned int> netMap;    // net name -> id
inline std::vector<std::string> netList = {"#"};                // id -> net name
inline unsigned int nextNetId = 1;

// Returns the id for a net name, creating a new id the first time a name is seen.
inline unsigned int getNetId(const std::string& net) {
    if (net == "#") return NET_NONE;
    if (netMap.count(net)) return netMap[net];
    netMap[net] = nextNetId;
    netList.push_back(net);
    return nextNetId++;
}

// ---------------- Stuff for Contacts and via stacks ----------------
inline bool isContactAttr(unsigned int a) {
    return a == L_POLYCON || a == L_LIBRDRCON || a == L_MCON;
}

// The contact tile that sits on a given conductor layer (L_NONE if the layer has none).
inline unsigned int contactAttrOf(unsigned int layerAttr) {
    switch (layerAttr) {
        case L_POLY: return L_POLYCON;
        case L_LI:   return L_LIBRDRCON;
        case L_M1:   return L_MCON;
        default:     return L_NONE;
    }
}

// Plane index of a tile attr (-1 for markers/unknown).
inline int attrToPlane(unsigned int attr) {
    return layerNameToPlane(attrToLayerName(attr));
}

// Which conductor layers a via connects.
enum ViaStack { VIA_POLY_LI, VIA_LI_M1, VIA_POLY_LI_M1 };

// Bottom to top. Each layer in the stack gets a pad; each pad gets a contact.
inline std::vector<unsigned int> viaLayers(ViaStack s) {
    switch (s) {
        case VIA_POLY_LI:    return {L_POLY, L_LI};
        case VIA_LI_M1:      return {L_LI, L_M1};
        case VIA_POLY_LI_M1: return {L_POLY, L_LI, L_M1};
    }
    return {};
}

inline unsigned int baseLayerAttr(unsigned int a) {   // contact tile counts as the layer it sits on
    if (a == L_POLYCON)   return L_POLY;
    if (a == L_LIBRDRCON) return L_LI;
    if (a == L_MCON)      return L_M1;
    return a;
}

// Pad and contact sizes per stack. EDIT THESE for the technology.
// The contact is centred in the pad: offset = (pad - contact) / 2.
struct ViaRule {
    long padW, padH;       // pad rectangle on every plane of the stack
    long contactSize;      // square contact carved in the middle
    long offX() const { return (padW - contactSize) / 2; }
    long offY() const { return (padH - contactSize) / 2; }
};

inline ViaRule viaRule(ViaStack s) {
    switch (s) {
        case VIA_POLY_LI:    return {5, 7, 3};
        case VIA_LI_M1:      return {5, 5, 3};
        case VIA_POLY_LI_M1: return {5, 7, 3};
    }
    return {5, 5, 3};
}