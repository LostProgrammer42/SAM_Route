from matplotlib import pyplot as plt
from matplotlib.patches import Rectangle

false = False
true = True


tiles = [
  {"name":"T0","llx":0,"lly":0,"wx":500,"wy":30,"filled":false},
  {"name":"T1","llx":0,"lly":30,"wx":30,"wy":40,"filled":false},
  {"name":"T2","llx":30,"lly":30,"wx":40,"wy":40,"filled":true},
  {"name":"T3","llx":0,"lly":70,"wx":500,"wy":430,"filled":false},
  {"name":"T4","llx":70,"lly":30,"wx":430,"wy":40,"filled":false}
]


fig, ax = plt.subplots(figsize=(6,6))

for t in tiles:
    rect = Rectangle((t["llx"], t["lly"]), t["wx"], t["wy"],
                     linewidth=3 if t["filled"] else 1,
                     fill=False)
    ax.add_patch(rect)
    cx = t["llx"] + t["wx"]/2.0
    cy = t["lly"] + t["wy"]/2.0
    label = t["name"] + ("\n(filled)" if t["filled"] else "")
    ax.text(cx, cy, label, ha="center", va="center", fontsize=10)

ax.set_xlim(-5, 505)
ax.set_ylim(-5, 505)
ax.set_aspect('equal', adjustable='box')
ax.set_title("Corner-Stitched Tiles: center inserted into a big empty tile")
ax.set_xlabel("x")
ax.set_ylabel("y")
plt.grid(True, linestyle=':', linewidth=0.5)
plt.show()
