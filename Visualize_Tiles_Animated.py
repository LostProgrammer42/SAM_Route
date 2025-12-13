import time
from matplotlib import pyplot as plt
from matplotlib.patches import Rectangle

def draw_tiles(ax, tiles):
    ax.clear()
    ax.set_aspect('equal', adjustable='box')
    ax.grid(True, linestyle=':', linewidth=0.5)
    ax.set_xlabel('x')
    ax.set_ylabel('y')

    # Compute consistent bounds
    xs, ys = [], []
    for t in tiles:
        xs += [t["llx"], t["llx"] + t["wx"]]
        ys += [t["lly"], t["lly"] + t["wy"]]

    ax.set_xlim(min(xs)-5, max(xs)+5)
    ax.set_ylim(min(ys)-5, max(ys)+5)

    for t in tiles:
        rect = Rectangle(
            (t["llx"], t["lly"]),
            t["wx"], t["wy"],
            fill=False,
            linewidth=3 if t.get("filled", False) else 1
        )
        ax.add_patch(rect)

        # Draw label
        cx = t["llx"] + t["wx"]/2
        cy = t["lly"] + t["wy"]/2
        label = t.get("name", "")
        ax.text(cx, cy, label, ha="center", va="center", fontsize=9)

    plt.draw()


def jump_cut_loop(tiles_a, tiles_b, delay=1.0):
    fig, ax = plt.subplots(figsize=(6,6))
    fig.suptitle("Jump-Cut Loop")

    state = 0  # 0 = show A, 1 = show B

    while True:
        if state == 0:
            draw_tiles(ax, tiles_a)
            state = 1
        else:
            draw_tiles(ax, tiles_b)
            state = 0

        # Keep GUI responsive
        t0 = time.time()
        while time.time() - t0 < delay:
            plt.pause(0.05)

false = False
true = True


# -----------------------
# Example usage
# -----------------------

tiles_a = [
  {"name":"T0","llx":0,"lly":0,"wx":500,"wy":30,"filled":false},
  {"name":"T1","llx":0,"lly":30,"wx":30,"wy":40,"filled":false},
  {"name":"T2","llx":30,"lly":30,"wx":40,"wy":40,"filled":true},
  {"name":"T3","llx":0,"lly":70,"wx":500,"wy":430,"filled":false},
  {"name":"T4","llx":70,"lly":30,"wx":430,"wy":10,"filled":false},
  {"name":"T5","llx":70,"lly":40,"wx":20,"wy":30,"filled":false},
  {"name":"T6","llx":90,"lly":40,"wx":40,"wy":30,"filled":true},
  {"name":"T7","llx":130,"lly":40,"wx":370,"wy":30,"filled":false}
]



tiles_b = [
  {"name":"T0","llx":0,"lly":0,"wx":500,"wy":30,"filled":false},
  {"name":"T1","llx":0,"lly":30,"wx":30,"wy":40,"filled":false},
  {"name":"T2","llx":30,"lly":30,"wx":40,"wy":40,"filled":true},
  {"name":"T3","llx":0,"lly":70,"wx":190,"wy":10,"filled":false},
  {"name":"T4","llx":70,"lly":30,"wx":430,"wy":20,"filled":false},
  {"name":"T5","llx":190,"lly":70,"wx":40,"wy":10,"filled":true},
  {"name":"T6","llx":0,"lly":80,"wx":500,"wy":420,"filled":false},
  {"name":"T7","llx":70,"lly":50,"wx":120,"wy":20,"filled":false},
  {"name":"T8","llx":230,"lly":70,"wx":270,"wy":10,"filled":false},
  {"name":"T9","llx":190,"lly":50,"wx":40,"wy":20,"filled":true},
  {"name":"T10","llx":230,"lly":50,"wx":270,"wy":20,"filled":false}
]



jump_cut_loop(tiles_a, tiles_b, delay=1.2)
