# ---------- USAGE ----------
# Run without an argument:
#   python plot.py
#   -> saves the output as "layout.pdf" in the current directory
#
# Run with an output name:
#   python plot.py <name>
#   -> saves the output as "Visuals/<name>.pdf"
#
# Example:
#   python plot.py routed_layout
#   -> Visuals/routed_layout.pdf
#
# The "Visuals" directory is created automatically if it does not exist.
# ----------------------------------------------------------------------

import ast
from matplotlib import pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.patches import Patch
import sys
import os

# ---------- CONFIG ----------
plane_files = [
    ("SAM_Files/plane0_routed.sam", 0),
    ("SAM_Files/plane1_routed.sam", 1),
    ("SAM_Files/plane2_routed.sam", 2)
]

Tile_boundaries = False
Tile_names = False
Tile_nets = True
Tile_edge_color = True
Zoom_to_fit = True
Legend = False

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
    if layer.startswith("m") and layer != "mcon":
        return "purple"
    if layer == "left":
        return "orange"
    if layer == "right":
        return "cyan"
    if layer == "top":
        return "pink"
    if layer == "bottom":
        return "black"
    if layer == "li":
        return "#ECBA6F"
    if layer == "polycon":
        return "#12A7BB"
    if layer == "mcon":
        return "#6D2483"
    return "gray"

def layer_hatch(layer):
    layer = layer.lower()
    if layer == "polycon" or layer == "mcon":
        return "xxxx"
    return None  # no hatch
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
    hatch = layer_hatch(t["layer"])
    filled = t["filled"]
    style = PLANE_STYLE[t["plane"]]
    rect = Rectangle(
        (t["llx"], t["lly"]),
        t["wx"],
        t["wy"],
        linewidth=0.5 if filled else 1,
        edgecolor="black" if Tile_boundaries else (color if (filled and Tile_edge_color) else "none"),
        facecolor=color if filled else "none",
        hatch=hatch if filled else None,
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
    if filled and t["net"] != 0 and Tile_nets:
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
# ax.grid(True, linestyle=":", linewidth=0.5)

if Zoom_to_fit: 
    valid_tiles = [
        t for t in all_tiles
        if t["filled"] and t["layer"].lower() != "none"
    ]

    # compute bounds only from valid tiles
    xs = [t["llx"] for t in valid_tiles] + [t["llx"] + t["wx"] for t in valid_tiles]
    ys = [t["lly"] for t in valid_tiles] + [t["lly"] + t["wy"] for t in valid_tiles]

    ax.set_xlim(min(xs) - 5, max(xs) + 5)
    ax.set_ylim(min(ys) - 5, max(ys) + 5)
else: 
    xs = [t["llx"] for t in all_tiles] + [t["llx"] + t["wx"] for t in all_tiles]
    ys = [t["lly"] for t in all_tiles] + [t["lly"] + t["wy"] for t in all_tiles]
    ax.set_xlim(min(xs) - 5, max(xs) + 5)
    ax.set_ylim(min(ys) - 5, max(ys) + 5)

if Legend:
    present_layers = []

    # collect only visible layers
    for t in all_tiles:
        layer = t["layer"].lower()
        if t["filled"] and layer not in present_layers:
            present_layers.append(layer)

    legend_elements = []

    for layer in present_layers:

        # use plane-0 alpha as default
        alpha = PLANE_STYLE[0]["alpha"]

        legend_elements.append(
            Patch(
                facecolor=layer_color(layer),
                hatch=layer_hatch(layer),
                alpha=alpha,
                edgecolor=(0,0,0,0.3) if hatch else (0,0,0,0),
                linewidth=0,
                label=layer
            )
        )

    ax.legend(
        handles=legend_elements,
        title="Layers",
        loc="upper left",
        bbox_to_anchor=(1.02, 1),
        fontsize=11,
        title_fontsize=13,

        # increase patch size
        handlelength=2.2,
        handleheight=1.3,
        handletextpad=0.8,
        borderpad=1.2,
        labelspacing=0.8,

        frameon=True
    )

# ---------- SAVE PDF ----------
if len(sys.argv) > 1:
    output_name = sys.argv[1]
    output_path = os.path.join("Visuals", output_name + ".pdf")
    os.makedirs("Visuals", exist_ok=True)
else:
    output_path = "layout.pdf"

plt.savefig(
    output_path,
    bbox_inches="tight",
    facecolor="white",
    format="pdf"
)

plt.show()

