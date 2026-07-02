#!/usr/bin/env python3

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math


# ---------------------------------------------------------
# File Readers
# ---------------------------------------------------------

def read_polygon_file(filename):
    """
    Reads a file of lat,lon points.
    Blank lines separate polygons.
    Returns list of polygons (each is list of (lat, lon)).
    """
    polygons = []
    current = []

    with open('/home/rpi31/Documents/rutas/'+filename, "r") as f:
        for line in f:
            line = line.strip()

            if not line:
                if current:
                    polygons.append(current)
                    current = []
                continue

            gps = line.split(" ")
            lat = float(gps[0])
            lon = float(gps[1])
            if len(gps)>=3: ber = float(gps[2])
            
            current.append((lat, lon))

    if current:
        polygons.append(current)

    return polygons


def read_path_file(filename):
    """
    Reads a continuous path file of lat,lon points.
    """
    points = []
    with open(filename, "r") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            # ~ lat, lon, ber = map(float, line.split(" "))
            gps = line.split(" ")
            lat = float(gps[0])
            lon = float(gps[1])
            if len(gps)>=3: ber = float(gps[2])
            
            points.append((lat, lon))

    return points

def find_duplicates(point_list):
    dups = list(set([ele for ele in point_list if point_list.count(ele) > 1]))
    # ~ print(dups)
    return dups


# ---------------------------------------------------------
# Load data
# ---------------------------------------------------------

boundary_polys = read_polygon_file("mowing_boundary.txt")
nogo_polys = read_polygon_file("mowing_nogo.txt")
path_points = read_path_file("mowing_path.txt")

# Convert to plotting coordinates (lon = x, lat = y)
boundary_xy = [[(lon, lat) for lat, lon in poly] for poly in boundary_polys]
nogo_xy = [[(lon, lat) for lat, lon in poly] for poly in nogo_polys]

dups_xy = find_duplicates(path_points)
dx = [lon for lat, lon in dups_xy]
dy = [lat for lat, lon in dups_xy]

px = [lon for lat, lon in path_points]
py = [lat for lat, lon in path_points]


# ---------------------------------------------------------
# Setup plot
# ---------------------------------------------------------

fig, ax = plt.subplots(figsize=(8, 8))

# Plot boundary
for poly in boundary_xy:
    xs, ys = zip(*(poly + [poly[0]]))  # close loop
    ax.plot(xs, ys, color="black", linewidth=2, label="Boundary")

# Plot no-go areas
for poly in nogo_xy:
    xs, ys = zip(*(poly + [poly[0]]))
    ax.plot(xs, ys, color="gray", linestyle="--", label="No-Go")

# Any duplicates
ax.plot(dx, dy, 'o', markersize=6)

# Animated elements
path_line, = ax.plot([], [], color="red", linewidth=2, label="Path")

# Set limits
all_x = px + [x for poly in boundary_xy for x, _ in poly]
all_y = py + [y for poly in boundary_xy for _, y in poly]

ax.set_xlim(min(all_x), max(all_x))
ax.set_ylim(min(all_y), max(all_y))

ax.set_aspect("equal")
ax.set_title("Mowing Path Animation")
ax.set_xlabel("Longitude")
ax.set_ylabel("Latitude")


# Remove duplicate legend entries
handles, labels = ax.get_legend_handles_labels()
unique = dict(zip(labels, handles))
ax.legend(unique.values(), unique.keys())


# ---------------------------------------------------------
# Animation functions
# ---------------------------------------------------------
def aniInit():
    path_line.set_data([],[])
    return (path_line,)
def frameUpdate(path_lineFrame):
    path_line.set_data(px[:path_lineFrame], py[:path_lineFrame])
    return (path_line,)

ani = animation.FuncAnimation(fig, frameUpdate, frames = len(px), init_func = aniInit, interval = 2, blit = True)
# ---------------------------------------------------------
# Show plot
# ---------------------------------------------------------

plt.show()
