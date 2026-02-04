import ast
from matplotlib import pyplot as plt
from matplotlib.patches import Rectangle

# ---------- CONFIG ----------
plane_files = [
    ("plane0_routed.sam", 0)
    # ("plane1.sam", 1)
    # ("plane2.sam", 2),
]

Tile_boundaries = True
Tile_names = False
# ---------- COLOR MAP ----------
def layer_color(layer):
    layer = layer.lower()
    if layer == "none":
        return "white"
    if layer == "ndiff":
        return "green"
    if layer == "pdiff":
        return "blue"
    if layer == "ntransistor":
        return "#006400"   # dark green
    if layer == "ptransistor":
        return "#00008B"   # dark blue
    if layer == "polysilicon":
        return "red"
    if layer.startswith("m"):
        return "purple"
    if layer == "left":
        return "orange"
    if layer == "right":
        return "cyan"
    if layer == "top":
        return "pink"
    if layer == "bottom":
        return "black"
    return "gray"

# ---------- LOAD ALL TILES ----------
all_tiles = []

for fname, plane in plane_files:
    with open(fname) as f:
        tiles = ast.literal_eval(f.read())
        for t in tiles:
            t["plane"] = plane
            all_tiles.append(t)

# ---------- PLOT ----------
fig, ax = plt.subplots(figsize=(8, 8))

PLANE_STYLE = {
    0: dict(alpha=0.60),
    1: dict(alpha=0.45),
    2: dict(alpha=0.30),
}
for t in all_tiles:
    color = layer_color(t["layer"])
    filled = t["filled"]
    style = PLANE_STYLE[t["plane"]]
    rect = Rectangle(
        (t["llx"], t["lly"]),
        t["wx"],
        t["wy"],
        linewidth=0.5 if filled else 1,
        edgecolor="black" if Tile_boundaries else (color if filled else "none"),
        facecolor=color if filled else "none",
        alpha= style["alpha"] if filled else 1.0,
        zorder=10 + t["plane"]   # higher plane on top
    )
    ax.add_patch(rect)
    if Tile_names:
        cx = t["llx"] + t["wx"] / 2
        cy = t["lly"] + t["wy"] / 2
        ax.text(
            cx, cy, t["name"],
            ha="center", va="center",
            fontsize=7,
            color="black",
            zorder=30
        )
    # draw net id (only if filled)
    if filled and t["net"] != 0:
        cx = t["llx"] + t["wx"] / 2
        cy = t["lly"] + t["wy"] / 2
        ax.text(cx, cy, str(t["net"]),
                ha="center", va="center",
                fontsize=8, color="black",
                zorder=20)

# ---------- AXES ----------
ax.set_aspect("equal", adjustable="box")
ax.set_xlabel("x")
ax.set_ylabel("y")
ax.set_title("Corner Stitch Layout (3 planes)")
ax.grid(True, linestyle=":", linewidth=0.5)

# auto bounds
xs = [t["llx"] for t in all_tiles] + [t["llx"] + t["wx"] for t in all_tiles]
ys = [t["lly"] for t in all_tiles] + [t["lly"] + t["wy"] for t in all_tiles]
ax.set_xlim(min(xs) - 5, max(xs) + 5)
ax.set_ylim(min(ys) - 5, max(ys) + 5)

plt.show()