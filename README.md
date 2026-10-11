<h1 align="center">SAM_Route</h1>

<p align="center">
  <b>A detailed router for VLSI standard cells, built on the corner-stitch data structure.</b><br>
  <sub>Version 2 &nbsp;·&nbsp; C++17 &nbsp;·&nbsp; sky130 cells &nbsp;·&nbsp; header-only core</sub>
</p>

<p align="center">
  <img alt="C++17" src="https://img.shields.io/badge/C%2B%2B-17-00599C?logo=c%2B%2B&logoColor=white">
  <img alt="Technology" src="https://img.shields.io/badge/tech-sky130-4c8bf5">
  <img alt="Layers" src="https://img.shields.io/badge/layers-poly%20%7C%20LI%20%7C%20M1-8a63d2">
  <img alt="Status" src="https://img.shields.io/badge/status-V2%20in%20progress-orange">
</p>

---

## What is this?

SAM_Route takes a placed standard cell (a `.rect` file with diffusion, transistors, poly and pre-placed pins),
works out which tiles of each net are not yet connected, and draws the missing wires on **polysilicon**,
**local interconnect (LI)** and **metal 1**, adding vias between them where needed.

Everything lives in three **corner-stitch planes**, so the "is this spot free?" and "who is my neighbour?"
questions that routing asks all the time are cheap tile walks instead of scans of a grid.

| Plane | Holds |
|:-----:|-------|
| 0 | diffusion, transistors, polysilicon (and the poly contacts) |
| 1 | local interconnect (LI) (and the LI contacts) |
| 2 | metal 1 (and the M1 contacts) |

---

## What's new in V2

V2 keeps the routing core of V1 and reworks **how vias are placed, tracked and finally drawn**. Switch routing
(the "hop to LI when a direct route fails" fallback) was rebuilt around one via engine instead of several near-duplicate functions.

### At a glance

| | V1 | V2 |
|---|----|----|
| Via placement | `placeViaOnPoly` and `placeViaOnMetal`, two separate functions | **One** `placeVia` entry point with a `ViaOptions` struct |
| Switch routing | `attemptSwitchRouting` (poly) and `attemptSwitchRoutingMetal` (M1) | **One** `attemptSwitchRouting` used by poly, M1 and the final interlayer pass |
| Via shapes | Sizes inside the placement code | **Lookup table** (`ViaStack` + `ViaRule` in `Layers.hpp`), edit one place per technology |
| Via sites | Per-function site search | **Ranked candidates** from `findViaSites` (see below) |
| Contacts | Carved immediately when a via is placed | **Carved once, at the very end** (`finalizeVias`) |
| Connectivity across layers | Followed pointers between contact tiles ("brothers") | Follows a **via registry** of plain coordinates |
| Failed via | Handled separately in each function | **Undone cleanly** (`undoVia`): pads removed, relabelled tiles restored |
| Existing via nearby | Metal switch searched for existing LI | **Reused** when the net already sits over a via that reaches LI (`findExposedLI`, via the registry) |
| Console output | Plain `cout` everywhere | **Switchable categories** (`SAM_DEBUG=...`), or compiled out entirely |
| Quality numbers | Only "did it finish" | **`STATS` line** per run: wire length, bends, vias, dead fronts |
| Port choice | Nearest pair by Manhattan distance | Also weighs **which way each port faces** |
| Corner contact | Counted as electrically connected | **Not** counted: a real overlap is required |

### 1. One via engine

Every caller (switch routing on poly, switch routing on M1, the interlayer pass) now goes through the same stack of building blocks:

```text
insertViaPads / removeViaPads / carveViaContacts   put pads down, take them out, cut contacts
findViaSites                                       ranked list of legal pad spots near an anchor
findViaAnchors                                     what a via can attach to (poly gate tab, M1 tile)
placeViaStack                                      try the sites for one anchor, bridge, undo on failure
placeVia (+ ViaOptions)                            the single entry point
undoVia                                            take a via we created back out
```

The three via stacks are `VIA_POLY_LI`, `VIA_LI_M1` and `VIA_POLY_LI_M1`. Pad and contact sizes sit in
`viaRule()` in `Layers.hpp`:

```cpp
case VIA_POLY_LI:    return {5, 7, 3};   // pad W, pad H, contact size
case VIA_LI_M1:      return {5, 5, 3};
case VIA_POLY_LI_M1: return {5, 7, 3};
```

### 2. Contacts are cut at the end

In V1 a via carved its small contact squares the moment it was placed. Those little squares fragmented the surrounding pads, so the router saw
many small tiles and the pad edges did not behave like normal wire sources and destinations.

In V2 a via is first only **recorded** (stack, position, net) and its **full pad rectangle** is filled on every layer of the stack.
Routing therefore sees large, fully merged rectangles. Once all routing is finished, `finalizeVias()` goes through the record and cuts every contact square
in one pass, at the same positions as before.

```mermaid
flowchart LR
    A[place via] --> B[fill full pads on each layer]
    B --> C[record in via registry]
    C --> D[route as usual]
    D --> E{all layers routed?}
    E -- no --> A
    E -- yes --> F[finalizeVias: cut contacts]
    F --> G[export]
```

### 3. A via registry instead of contact pointers

Layers used to be joined electrically through pointers between contact tiles, which could dangle whenever tiles were merged or deleted.
V2 stores each via as a `ViaRecord { stack, x, y, net }`. A via's pad covers every layer of its stack, so `electricallyAdjacent` just
crosses layers wherever a same-net tile overlaps a recorded pad. `findExposedLI` and `undoVia` use the same registry.

### 4. Smarter via sites

`findViaSites` ranks candidate positions instead of taking the first free one, using the bloated planes (so spacing is respected):

- the **gap** to neighbouring shapes, so vias do not squeeze into tight spots
- how much **new metal** the pad adds beyond the anchor, so a site overlapping the anchor's own metal wins
- distance from the anchor, then position as a stable tie-break
- for the interlayer pass (`preferNearM1`), the gap to the net's existing M1 is added to the ranking too

Sites are checked lazily against the bloated planes, and the search is capped by `maxSites` and `radius` in `ViaOptions`.

### 5. Poly anchors and gate tabs

For poly, the via attaches to a **gate tab**: a poly stub sitting above or below a transistor. `findViaAnchors` finds these,
including which side of the gate each one is on, so the second via of a pair can be put on the *other* side (`avoidDir`).
Gate tabs are used once, which makes the poly hop naturally repeat-safe.

### 6. Debug output you can switch on and off

All progress output goes through one macro, `DBG(category) << ...`, defined in `Header_Files/Debug.hpp`. Each message belongs to a
category, and you choose which categories print when you run the program, with no recompiling:

| Category | Covers |
|----------|--------|
| `summary` | Per-layer and per-net results ("fully routed", "Switch routing succeeded", iteration counts) and the final `STATS` line |
| `pairs` | Which route pairs are generated and tried |
| `ports` | The SRC and DST port lists |
| `search` | The bidirectional search: failures, `PathPiece` lines, "Routed", retries |
| `via` | Via sites, placement, bridge failures, reuse |
| `switch` | Switch routing decisions |
| `conn` | Connectivity checks ("Not routing Electrically Adjacent Tiles", ...) |
| `planes` | Plane dumps |
| `all` / `none` | Everything / nothing |

```bash
SAM_DEBUG=via ./Route cell.rect polysilicon,m1               # only via output
SAM_DEBUG=summary,switch ./Route cell.rect polysilicon,m1    # results plus switch decisions
SAM_DEBUG=all,-search ./Route cell.rect polysilicon,m1       # everything except the search noise
SAM_DEBUG=none ./Route cell.rect polysilicon,m1              # silent
g++ -std=c++17 -DSAM_NO_DEBUG SAM_Route.cpp -o Route         # every DBG line compiled out
```

- Names are comma separated; a leading `-` removes a category from the list so far. Unknown names print a warning and are ignored.
- With `SAM_DEBUG` unset everything prints, so output is the same as before the system existed.
- A message whose category is off is skipped completely, arguments included, so disabled output costs almost nothing.
- Error messages that must never be hidden (bad input line, failed contact carve) stay as plain `cout`.
- To add a category: one line in the `DbgCat` enum and one entry in the name table in `parseDbg`.

### 7. A one-line quality summary

Finishing is not the same as routing well, so every run can print one `STATS` line (category `summary`):

```text
STATS cell=_0_0syn_0_0MUX2X1 len(poly/li/m1)=44/322/184 bends=33 vias(poly-li/li-m1/stack)=0/6/1 deadFronts=9
```

| Field | Meaning |
|-------|---------|
| `len(poly/li/m1)` | Total routed centreline length per layer |
| `bends` | Direction changes over all routed paths |
| `vias(...)` | Vias placed, by stack type |
| `deadFronts` | Searches that ran out of tiles (the "fail" lines) |

```bash
SAM_DEBUG=summary ./Route Test_Cells/_0_0syn_0_0MUX2X1.rect polysilicon,m1 | grep STATS
```

Routes that were committed and later undone (for example a failed bridge) still count, so the numbers run slightly high.
Use these lines to compare two versions of the router cell by cell instead of judging by eye.

### 8. Small router improvements

- **Ports are chosen with direction in mind.** The port-pair loop used to take the nearest pair and ignore which way each port faces.
  A port that faces away from the other end has to U-turn, so it gets a penalty of 2 x (wire width + spacing); a port that points straight
  at the other end gets a small bonus (`facingCost` in `Ports_Paths.hpp`).
- **Ports with no room are dropped once**, before pairing, using the same push-out check the search uses (`portPushOut`).
  Same routes as before, a little less work. If every port is dropped the search still runs, so the metal-switch fallback is not skipped.
- **Corner contact is no longer a connection.** `geomTouch` used to treat two tiles that share only a corner point as touching, so two wires
  meeting diagonally were taken for connected. A real overlap along one edge is now required.

### 9. Structure

The code is split by job:

| File | Role |
|------|------|
| `SAM_Route.cpp` | `main`: load, route poly, route M1, interlayer vias, re-route M1, finalize, export |
| `Header_Files/Corner_Stitch.hpp` | The tile structure, insertion, deletion, neighbour walks, bloating |
| `Header_Files/Layers.hpp` | Layer IDs, rules, via stacks and sizes, net-name registry |
| `Header_Files/Debug.hpp` | Output categories (`DBG`), `SAM_DEBUG` parsing, run statistics |
| `Header_Files/Layout_Files.hpp` | Reading `.rect` files and exporting results |
| `Header_Files/Layout_Connectivity.hpp` | `electricallyAdjacent`, via registry, per-layer/net caches |
| `Header_Files/Ports_Paths.hpp` | Port selection and path shaping |
| `Header_Files/Router.hpp` | Route pairs, `attemptRoutePair` (bidirectional tile search), `routeLayer` |
| `Header_Files/Switch_Routing.hpp` | The via engine, switch routing, interlayer vias |

> `Router.hpp` and `Switch_Routing.hpp` call into each other, so include them together.

---

## How a run works

```mermaid
flowchart TD
    L[load .rect into 3 planes] --> P[route polysilicon]
    P --> M[route M1]
    M -->|direct route fails or is too long| S[switch routing:<br/>via, route on LI, via]
    S --> M
    M --> I[interlayer vias:<br/>poly to LI to M1 per net]
    I --> R[re-route M1 to join the new pads]
    R --> F[finalizeVias: cut contacts]
    F --> X[export .rect and .sam]
```

Each layer is routed for several iterations. Iteration 1 pairs tiles that line up in X, iteration 2 in Y, and later ones take nearest-neighbour
pairs, so a stuck pair gets retried differently.

---

## Quick start

```bash
# build
g++ -std=c++17 -O2 SAM_Route.cpp -o Route

# route one cell: poly first, then M1
./Route Test_Cells/_0_0syn_0_0MUX2X1.rect polysilicon,m1

# draw the result
python3 Visualize_Tiles.py

# quiet run with a one-line quality summary
SAM_DEBUG=summary ./Route Test_Cells/_0_0syn_0_0MUX2X1.rect polysilicon,m1 | grep STATS
```

Outputs:

- `Output_Rects/Routed.rect`: the routed layout, contacts included
- `SAM_Files/plane{0,1,2}_routed.sam`: the three tile planes
- `layout.pdf`: a picture of the planes (via `Visualize_Tiles.py`)

Helper scripts are in `Scripts/`:

| Script | Does |
|--------|------|
| `Run_Flow.sh` | build, route one cell, visualise |
| `Test_All.sh` | build, route every cell in `Test_Cells/`, produce a PDF per cell |
| `Compare_All.sh` | regression check: `save` stores results for every cell, `check` re-runs and reports any difference |

---

## Results so far

Run over the 38 cells in `Test_Cells/`. "Fail lines" (dead fronts) counts failed searches printed by the router. Many are retried and succeed,
so it is a measure of search effort, not of final failures.

**The via rework** (the version just before it, V1 plus early fixes, against V2 at that point):

| | Before | After |
|---|:---:|:---:|
| Cells that hit the 40 s timeout | 3 | **0** |
| Cells where M1 did not finish | 3 | **1** |
| Total fail lines | 11,159 | **341** |
| Cells with zero fail lines | 22 | **24** |
| Wall time for all 38 cells | ~132 s | **~12 s** |

**Router improvements since** (measured with the `STATS` line, totals over all 38 cells):

| | Via rework | Now |
|---|:---:|:---:|
| Total wire length (poly / LI / M1) | 7314 (967 / 1743 / 4604) | **7218** (884 / 1815 / 4519) |
| Vias | 107 | **103** |
| Bends | 510 | 512 |
| Dead fronts | 341 | 367 |
| M1 unfinished | 1 | 1 |

The length and via gains come from the port-direction cost (mostly `MUX2X1`, 550 down to 397 length). Dead fronts rose a little with it
(341 to 368); the corner-contact fix then adds about 10 units of wire, because the router now routes pairs it used to treat as already connected.
The one cell where M1 still does not finish is `g0n1n2naa_01ox0` (see below).

### Things we tried that did not pay off

Recorded so they are not tried again by accident:

- **Tie-break rules for equal-cost neighbours** (keep direction, line up with target, prefer wider tile): no change. Exact ties happened only 95 times across all runs.
- **A bend penalty in the main neighbour cost:** worse. From a penalty of 4 upward dead fronts rose from about 370 to over 3000 and more cells failed to finish.
- **Raising the 50-alternation cap on the search to 500:** no change. The cap is never reached.
- **Ordering route pairs by the real gap between tiles instead of corner distance:** the pair order changed in 14 cells but the layouts came out identical.
- **Not stopping the path insertion at the first electrical contact:** no measurable change.

---

## Known limitations

- **No rip-up yet.** A net routed early can wall off a later net. In `g0n1n2naa_01ox0`, net 1's interlayer via sits below the cell,
  so its M1 wire runs through the middle and net 3 cannot cross it. V1's metal switch could rip up blocking LI; the unified V2 version does not.
- **Greedy search.** `attemptRoutePair` advances the two fronts by smallest Manhattan distance and breaks ties by a fixed
  direction order. It does not compare complete candidate paths.
- **Bloat is not refreshed after each via**, so the second via of a pair may be tested against slightly stale spacing.
- **Paths are not smoothed.** Jogs, staircases and detours remain. Across all cells 65 of 351 routes contain a short jog (a 1-2 unit sideways step
  between two parallel runs). It comes from the exit point chosen when the walk enters the next tile, not from the horizontal/vertical preference.

## Roadmap

- [ ] Remove the leftover contact-pointer code now that connectivity comes from the via registry
- [ ] Rebuild bloated planes after each placed via
- [ ] Rip-up and re-route, with per-route records so one route can be removed cleanly
- [ ] Best-first search instead of the greedy walk
- [ ] Path smoothing: remove the short jogs, merge collinear pieces, add a via-count penalty
- [ ] Retry a failed port pair once with a different first move, so only failures change
- [ ] Optional cell bounding box instead of the fixed 0..60 clamp on export

---

## Changelog

### V2 (in progress)
- Unified via placement (`placeVia`) and switch routing (`attemptSwitchRouting`)
- Via shapes moved to a lookup table in `Layers.hpp`
- Ranked via-site search; reuse of existing exposed LI; clean undo of failed vias
- Deferred contact carving (`finalizeVias`) and a via registry for cross-layer connectivity
- Guard against switch routing re-entering itself on the M1 bridge route
- Header comments and file layout reorganised by job
- Switchable debug output (`SAM_DEBUG`, `Debug.hpp`) and a compile-out mode (`-DSAM_NO_DEBUG`)
- `STATS` run summary: wire length, bends, vias, dead fronts
- Port-pair choice weighs port direction (`facingCost`); ports with no room are filtered once (`portPushOut`)
- `geomTouch` no longer counts a shared corner point as a connection

### V1
- Initial modular version: corner-stitch planes, bidirectional tile router, poly/M1 switch routing through LI, interlayer via stacks

---

## Authors

Stavan Mehta, Affaan Fakih