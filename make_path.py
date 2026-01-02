#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  make_path.py
#  
#  Copyright 2025  <rpi31@rpi31>
#  
#  Credit: mostly Microsoft Copilot
#  
# Given boundary path, interior "no-go" or "hole" path(s)
# Returns continuous interior path in North-South direction


from shapely.geometry import Polygon, LineString, MultiLineString, Point, mapping

from shapely.affinity import translate

from shapely.prepared import prep

from pyproj import Transformer

import numpy as np

import math

import matplotlib.pyplot as plt

 

# ---------------------------------------------------------

# PARAMETERS in meters unless otherwise noted

# ---------------------------------------------------------

IMPLEMENT_WIDTH = 0.5 #cut per pass

TURN_RADIUS_PREFERRED = 1.2

TURN_RADIUS_MIN = 0.4

VEHICLE_WIDTH = 0.67 #"fit thru door" width

SAFETY_MARGIN = 0.4

TOTAL_MARGIN = SAFETY_MARGIN + VEHICLE_WIDTH / 2.0

 

# ---------------------------------------------------------

# Helper: read GPS coordinates from file

# ---------------------------------------------------------

def read_gps_file(filename):

    coords = []

    with open(filename, "r", encoding='utf-8-sig') as f:

        for line in f:

            line = line.strip()

            if not line:

                coords.append(None)  # blank line = separator for holes

                continue
            
            lat, lon = map(float, line.split(" "))

            coords.append((lat, lon))

    return coords

 

# ---------------------------------------------------------

# 1. Load boundary and holes from files

# ---------------------------------------------------------

boundary_raw = read_gps_file("mowing_boundary.txt")

nogo_raw = read_gps_file("mowing_nogo.txt")

 

# Extract boundary polygon (ignore None separators)

boundary_gps = [p for p in boundary_raw if p is not None]

 

# Extract holes (list of lists)

holes_gps = []

current_hole = []

for p in nogo_raw:

    if p is None:

        if current_hole:

            holes_gps.append(current_hole)

            current_hole = []

    else:

        current_hole.append(p)

if current_hole:

    holes_gps.append(current_hole)

 
# ---------------------------------------------------------

# 2. GPS → Local ENU conversion

# ---------------------------------------------------------

lat0, lon0 = boundary_gps[0]

 

transformer = Transformer.from_crs(

    "epsg:4326",

    f"+proj=tmerc +lat_0={lat0} +lon_0={lon0} +k=1 +x_0=0 +y_0=0",

    always_xy=True

)

 

def gps_to_local(lat, lon):

    x, y = transformer.transform(lon, lat)

    return (x, y)

 

boundary_local = [gps_to_local(lat, lon) for lat, lon in boundary_gps]

holes_local = [[gps_to_local(lat, lon) for (lat, lon) in hole] for hole in holes_gps]

 
# ---------------------------------------------------------

# 3. Build lawn polygon with holes

# ---------------------------------------------------------

lawn = Polygon(boundary_local, holes = holes_local)

boundary_poly = Polygon(boundary_local)

# Shrink polygon for safety
lawn_safe = lawn.buffer(-TOTAL_MARGIN)

if lawn_safe.is_empty:

    raise ValueError("Safety margin too large; lawn_safe polygon disappeared.")

# Extract holes as individual polygons for explicit checks
hole_polygons = [Polygon(hole) for hole in holes_local]


# Make prepared polygons for later use
prepared_lawn = prep(lawn)

prepared_lawn_safe = prep(lawn_safe)

prepared_boundary = prep(boundary_poly)
 

# ---------------------------------------------------------

# 4. Define A and B from North-South bias OR Principal Component Analysis

# ---------------------------------------------------------

def AB_points_PCA(lawn_polygon):
    """
    Given a Shapely Polygon that may include holes,
    compute the long axis using PCA and return A and B points
    defining the AB line.
    """

    # ---------------------------------------------------------
    # 4.1. Collect ALL boundary points (exterior + holes)
    # ---------------------------------------------------------
    # Exterior boundary
    all_points = [np.array(lawn_polygon.exterior.coords)]

    # Add each hole boundary
    for interior in lawn_polygon.interiors:
        all_points.append(np.array(interior.coords))

    # Combine into one Nx2 array
    boundary = np.vstack(all_points)

    # ---------------------------------------------------------
    # 4.2. Center the data (subtract centroid)
    # ---------------------------------------------------------
    centroid = boundary.mean(axis=0)
    centered = boundary - centroid

    # ---------------------------------------------------------
    # 4.3. PCA: covariance → eigenvectors
    # ---------------------------------------------------------
    cov = np.cov(centered.T)
    eigenvalues, eigenvectors = np.linalg.eig(cov)

    # Principal component = eigenvector with largest eigenvalue
    idx = np.argmax(eigenvalues)
    principal_axis = eigenvectors[:, idx]

    # Normalize direction
    principal_axis = principal_axis / np.linalg.norm(principal_axis)

    # ---------------------------------------------------------
    # 4.4. Project all points onto the principal axis
    # ---------------------------------------------------------
    projections = centered @ principal_axis

    # Endpoints of the long axis
    min_proj = projections.min()
    max_proj = projections.max()

    A = centroid + min_proj * principal_axis
    B = centroid + max_proj * principal_axis

    return tuple(A), tuple(B)

def AB_points_NSbias(boundary_local):
    '''
    Given tuple of ENU coordinates as outer shell limits
    Define A and B cooridinates based on top/bottom 25% latitude bands
    '''

    # Sort all boundary points by latitude (north = larger y)
    sorted_by_lat = sorted(boundary_local, key=lambda p: p[1], reverse=True)

    n = len(sorted_by_lat)
    quartile_count = max(1, n // 4)

    # Top & 25% northern points
    top_band = sorted_by_lat[:quartile_count]
    bottom_band = sorted_by_lat[-quartile_count:]

    # A = most eastern point in the top 25%
    A = max(top_band, key=lambda p: p[0])

    # B = most eastern point in the bottom 25%
    B = max(bottom_band, key=lambda p: p[0])

    return tuple(A), tuple(B)


if False:
    A, B = AB_points_PCA(lawn)
else:
    A, B = AB_points_NSbias(boundary_local)


ab_line = LineString([A, B])


dx = B[0] - A[0]
dy = B[1] - A[1]

length = math.hypot(dx, dy)

if length == 0:

    raise ValueError("A and B must be distinct.")

 
nx = -dy / length

ny = dx / length


# ---------------------------------------------------------

# 5. Compute number of passes

# ---------------------------------------------------------

boundary_points = np.array(boundary_local)

ax, ay = A

px = boundary_points[:, 0]

py = boundary_points[:, 1]

 

cross = np.abs((px - ax) * dy - (py - ay) * dx)

distances = cross / length

max_dist = float(distances.max())

 

num_lines_each_side = math.ceil(max_dist / IMPLEMENT_WIDTH)

 

minx, miny, maxx, maxy = lawn.bounds

 

# ---------------------------------------------------------

# 6a. Generate offset passes

# ---------------------------------------------------------

# Type handler

def stringTypeAppend(myList, myLine):
    '''
    Given list/array-like variable to append either
     a LineString or MultiLineString to.
    Returns myList with new value
    '''
    
    if myLine is None:
        return myList
    
    if myLine.geom_type == "LineString":
        
        myList.append(myLine)

    elif myLine.geom_type == "MultiLineString":
        
        myList.extend(list(mapping(myLine).get('coordinates')))
    
    return myList

parallel_lines = []

# Generate all lines

for i in range(-num_lines_each_side, num_lines_each_side + 1):

    offset_x = nx * IMPLEMENT_WIDTH * i

    offset_y = ny * IMPLEMENT_WIDTH * i

    parallel = translate(ab_line, xoff=offset_x, yoff=offset_y)

 

    lminx, lminy, lmaxx, lmaxy = parallel.bounds

    if lmaxx < minx or lminx > maxx or lmaxy < miny or lminy > maxy:

        continue

    if not prepared_boundary.intersects(parallel):

        continue
    
    parallel_lines = stringTypeAppend(parallel_lines, parallel)
        
 
# ---------------------------------------------------------

# 6b. Sort lines boustrophedon-style

# ---------------------------------------------------------

def avg_x(line):
    
    line=LineString(line)
    
    xs, _ = line.xy

    return sum(xs) / len(xs)
    #return min(xs)

 
parallel_lines.sort(key=avg_x)


alternating_lines = []

for idx, ln in enumerate(parallel_lines):
    
    ln=LineString(ln)
    
    coords = list(ln.coords)

    if idx % 2 == 1:

        coords.reverse()
        
        fwd = True
    
    else:
        
        fwd = False

    alternating_lines.append([coords, fwd])


# ---------------------------------------------------------
#
# 7a. Clip lines to holes
#
# ---------------------------------------------------------

clipped_passes = []
plot_fwd = []
plot_rev = []

for fl in alternating_lines:

    full_line = LineString(fl[0])
    
    clipped = full_line.intersection(lawn_safe)
        
    if clipped.is_empty:
        
        continue
    
    clipped_passes = stringTypeAppend(clipped_passes, clipped)
        
        
    #for plotting later
    if fl[1]:
        plot_rev = stringTypeAppend(plot_rev, clipped)
    else:
        plot_fwd = stringTypeAppend(plot_fwd, clipped)


plot_rev = [LineString(y) for y in plot_rev]
plot_fwd = [LineString(z) for z in plot_fwd]


# ---------------------------------------------------------
#        
# 7b. Sequence clipped, alternated passes in order
#
# ---------------------------------------------------------

#Create point cloud of line endpoints

c_starts_full = []
c_ends_full = []

for cp in clipped_passes:
    cp = LineString(cp)
    cxy_0, cxy_1 = list(cp.coords)
    c_starts_full.append(cxy_0)
    c_ends_full.append(cxy_1)


#Determine nearest next start point based on available start points

c_starts_temp = np.array(c_starts_full)
c_ends_temp = np.array(c_ends_full)
c_passes_temp = clipped_passes
min_index = 0

ordered_passes = []

for idx, to_point in enumerate(c_starts_temp):
    
    #get rid of where we're starting since we cannot start there again
    c_starts_temp = np.delete(c_starts_temp, min_index, 0)
    
    #handle first values
    if idx == 0:
        ordered_passes.append(LineString(c_passes_temp[0]))
        xy_end = np.array(c_ends_temp[0])
    else:
        #assign endpoint found in previous loop
        xy_end = np.array(c_ends_temp[min_index])
        #add closest line as found in the previous loop
        chosen_line = LineString(c_passes_temp[min_index])
        ordered_passes.append(chosen_line)
    
    #remove line to keep all arrays the same length
    c_passes_temp.pop(min_index) # = np.delete(c_passes_temp, min_index, 0)
    
    #remove current endpoint since we're about to use it up
    c_ends_temp = np.delete(c_ends_temp, min_index, 0)
    
    #get all distances from current endpoint to all remaining start points
    if len(c_ends_temp)>=1:
        dist_cloud = np.linalg.norm(c_starts_temp - xy_end, axis=1)
        min_index = np.argmin(dist_cloud)
    else:
        break
    

# ---------------------------------------------------------

# 8. Arc generator

# ---------------------------------------------------------

def tangent_arc_between_passes(p1, p2, v1, v2, lawn, hole_polygons, num_points=6):
    '''
    Create a tangent circular arc between two passes with automatically
    computed radius limited by minimum radius and preferred radius.

    p1 = end point of previous pass (x,y)
    p2 = start point of next pass (x,y)
    v1 = direction vector of previous pass (not necessarily unit)
    v2 = direction vector of next pass (not necessarily unit)
    lawn = full lawn polygon (with holes)
    hole_polygons = list of hole polygons
    num_points = number of coordinates to define the "arc"
     
    '''
    #convert 2D to 3D for numpy
    p1 = np.array((*p1,0))
    p2 = np.array((*p2,0))
    v1 = np.array((*v1,0))
    v2 = np.array((*v2,0))
    # ~ print(f'from {p1} to {p2}')
    # ~ print(f'    dir {v1} to {v2}')
    
    #Make direction vectors into unit vectors
    def unitize(vector):
        norm = np.linalg.norm(vector) #credit: squash.io
        return vector/norm
    v1 = np.array(unitize(v1))
    v2 = np.array(unitize(v2))
    # ~ print(f'    unit {v1} to {v2}')
    
    #=====
    # Distances and Angles
    #=====
    
    #Angle of turn
    vdot = np.dot(v1, v2)
    # ~ print(f'    vdot {vdot}')
    turn_angle = math.acos(np.clip(vdot, -1.0, 1.0)) #credit: math.ttu.edu
    if turn_angle < math.pi/2: #short turn bypass
        seg = LineString([tuple(p1)[:-1], tuple(p2)[:-1]])
        if lawn.contains(seg) and all(not seg.intersects(h) for h in hole_polygons):
            # ~ print('== segmented')
            return seg
    # ~ print(f'    at angle {turn_angle} rad')
    
    #Vector from p1 to p2
    d = np.subtract(p2, p1)
    # ~ print(f'    separation {d}')
    
    #Minimum distances from points to other direction vector
    m1 = np.cross(-d, v2) #credit: math.lsa.umich.edu
    # ~ print(f'    m1 cross = {m1}')
    m1 = np.linalg.norm(m1) #dist from p1 to v2
    # ~ print(f'    p1 to v2 = {m1}')
    m2 = np.linalg.norm(np.cross(d, v1))
    # ~ print(f'    p2 to v1 = {m2}')
    #Chosen radius of turn
    R = max(TURN_RADIUS_MIN, min(m1/2, m2/2, TURN_RADIUS_PREFERRED))
    # ~ print(f'    Turn radius is {round(R,2)}m')
    
    #=====
    # Helper functions
    #=====
    
    def orth_comp_center(v_angled, v_along, p_from, radius):
        '''
        Define arc center point as perpendicular by radius distance 
         (on the side of v_angled) to given point of v_along
        Return orthogonal unit vector
        
        v_angled (d) = vector offset or at an angle to the pass vector
        v_along (v1/v2) = pass vector containing the given point
        p_from (p1/p2) = point on v_along
        radius (R) = target distance between arc center and p_from
        '''
        # ~ print('orth_comp_center given:')
        # ~ print(f'    v_angled = {v_angled}')
        # ~ print(f'    v_along = {v_along}')
        # ~ print(f'    p_from = {p_from}')
        # ~ print(f'    radius = {radius}')
        
        #Projected vector of Angled onto Along
        v_ang_len = np.hypot(v_angled[0],v_angled[1])
        v_along_unit = unitize(v_angled)
        v_dot = np.dot(v_along,v_angled)
        proj_ang_on_along = v_along_unit*(v_dot/v_ang_len) #credit: math.lsa.umich.edu
        
        #Orthogonal as difference of component from full vector
        orth_ang_on_along = v_angled - proj_ang_on_along
        dir_from_p = unitize(orth_ang_on_along)
        arc_center = p_from + R*dir_from_p
        
        # ~ print('--orth_comp_center made: ')
        # ~ print(f'    dir_from_p = {dir_from_p}')
        # ~ print(f'    arc_center = {arc_center}')
        return dir_from_p, arc_center
    
    def rotated_point(orig_dir, from_point, angle, radius):
        '''
        Return end point (2D) where vector with magnitude "radius",
          angled away from start unit vector "orig_dir" by "angle" radians
          starts from and about point "from_point"
        orig_dir and from_point = [x, y]
        '''
        # ~ print('rotated_point given:')
        # ~ print(f'    orig_dir = {orig_dir}')
        # ~ print(f'    from_point = {from_point}')
        # ~ print(f'    angle = {angle}')
        # ~ print(f'    radius = {radius}')
        
        #make unit vector for sure
        orig_dir = unitize(orig_dir)
        
        dx = orig_dir[0]*math.cos(angle) - orig_dir[1]*math.sin(angle)
        dy = orig_dir[0]*math.sin(angle) - orig_dir[1]*math.cos(angle)
        dir_to_new = np.array([dx, dy, 0]) #credit: physicsdigest.blog
        
        new_point = from_point + radius*dir_to_new
        new_point = new_point[:-1]
        
        # ~ print(f'--rotated_point = {new_point}')
        return new_point #[x,y]
    
    #=====
    # Arc point generation
    #=====
    
    #Define the center and points of the arc
    # using logic if p2 is "in the distance" from p1
    if np.dot(d,v1)>0: #arc is defined by p2
        # ~ print('Arc is connected to destination')
        #orth_v = from line-point to arc center
        orth_v, arc_ctr = orth_comp_center(-d, v2, p2, R)
        
        #Find start point rotated an angle away from center point
        phi = math.pi - turn_angle
        arc_start = (*rotated_point(orth_v, arc_ctr, phi, R),0)
        
        new_v = np.subtract(arc_start, arc_ctr)
        
        #Use all arc points except the last one (p2)
        arc_points = [rotated_point(new_v, arc_ctr, theta, R) for theta in np.linspace(0, -turn_angle, num_points)]
        arc = LineString(arc_points[:-1])
        
    else: #arc starts right away at p1
        # ~ print('Arc is connected to start')
        orth_v, arc_ctr = orth_comp_center(d, v1, p1, R)
        
        #Determine if turn should be left/right
        turn_sign = np.sign(np.cross(orth_v, v2)[2])
        
        #Use all arc points except the first one (p1)
        arc_points = [rotated_point(-orth_v, arc_ctr, theta, R) for theta in np.linspace(0, turn_sign*turn_angle, num_points)]
        arc = LineString(arc_points[1:])
        
    # ~ print(f'== {arc}')
    
    #=====
    # Double-check fitment
    #=====
    
    if not lawn.contains(arc):
        # ~ print('== out of bounds')
        return None
    if any(arc.intersects(h) for h in hole_polygons):
        # ~ print('== fell in a hole')
        return None
    
    # ~ print('')
    return arc
    
    '''
    """
    BY COPILOT
    Create a tangent circular arc between two passes with automatically
    computed radius. No preferred or minimum radius is used.

    p1 = end point of previous pass (x,y)
    p2 = start point of next pass (x,y)
    v1 = direction vector of previous pass (not necessarily unit)
    v2 = direction vector of next pass (not necessarily unit)
    lawn = full lawn polygon (with holes)
    hole_polygons = list of hole polygons
    """

    p1 = np.array(p1, float)
    p2 = np.array(p2, float)
    print(f'from {p1} to {p2}')

    # Normalize direction vectors
    v1 = np.array(v1, float)
    v2 = np.array(v2, float)
    v1 /= np.linalg.norm(v1)
    v2 /= np.linalg.norm(v2)
    print(f'    dir {v1} to {v2}')

    # Angle between directions
    dot = np.clip(np.dot(v1, v2), -1.0, 1.0)
    theta = math.acos(dot)
    print(f'    at angle {theta} rad')

    # If nearly collinear, no arc needed
    if theta < 1e-6:
        seg = LineString([tuple(p1), tuple(p2)])
        if lawn.contains(seg) and all(not seg.intersects(h) for h in hole_polygons):
            print('== segmented')
            return seg
        print('== colinear')
        return None

    # ---------------------------------------------------------
    # Compute the unique tangent arc radius
    # ---------------------------------------------------------
    # The tangent arc must touch both lines at equal distance d from p1 and p2.
    # The distance between tangent points is:
    #     chord_length = ||p2 - p1||
    # And the radius is:
    #     R = chord_length / (2 * sin(theta/2))
    #
    # This is the ONLY radius that produces a tangent arc between two lines.
    # ---------------------------------------------------------

    chord = p2 - p1
    chord_length = np.linalg.norm(chord)
    print(f'    chord length {chord_length}')

    # If chord is zero, no arc possible
    if chord_length < 1e-9:
        print('== same point')
        return None

    R = chord_length / (2.0 * math.sin(theta / 2.0))
    print(f'    Turn radius is {round(R,2)}m')

    # Distance from endpoints to tangent points
    d = R * math.atan(theta / 2.0)
    print(f'    end-tan distance {d}')

    # Tangent points
    t1 = p1 + v1 * d
    t2 = p2 - v2 * d
    print(f'Now tangent points {t1} to {t2}')

    # ---------------------------------------------------------
    # Compute arc center
    # ---------------------------------------------------------
    n1 = np.array([-v1[1], v1[0]])
    n2 = np.array([-v2[1], v2[0]])

    A = np.vstack([n1, -n2]).T
    b = (t2 - t1)
    print(f'    linalg given {A} and {b}')
    
    try:
        lam = np.linalg.solve(A, b)
        print(f'    linear algebra solution {lam}')
    except np.linalg.LinAlgError:
        print('!!! linalg.solve failed')
        return None

    C = t1 + n1 * lam[0]
    print(f'    arc center {C}')

    # ---------------------------------------------------------
    # Build the arc
    # ---------------------------------------------------------
    vC1 = t1 - C
    vC2 = t2 - C
    a1 = math.atan2(vC1[1], vC1[0])
    a2 = math.atan2(vC2[1], vC2[0])

    # Shortest sweep direction
    diff = (a2 - a1 + math.pi) % (2 * math.pi) - math.pi
    angles = [a1 + diff * t for t in np.linspace(0, 1, num_points)]
    # ~ print(f'    angles {angles}')
    
    arc_pts = [C + np.array([math.cos(a), math.sin(a)]) * R for a in angles]
    arc = LineString(arc_pts)
    print(f'== path {arc}')
    
    # ---------------------------------------------------------
    # Validate arc: must stay inside lawn and avoid holes
    # ---------------------------------------------------------
    if not lawn.contains(arc):
        print('out of bounds')
        return None
    if any(arc.intersects(h) for h in hole_polygons):
        print('fell in a hole')
        return None

    return arc
    '''

# ---------------------------------------------------------

# 9. Build continuous path

# ---------------------------------------------------------

continuous_coords = []

 
for i, ln in enumerate(ordered_passes):
    coords = list(ln.coords)

    if i == 0:
        continuous_coords.extend(coords)
    else:
        prev_end = continuous_coords[-1]
        curr_start = coords[0]

        # Direction of previous and next passes (for tangency)
        prev_line = ordered_passes[i - 1]
        prev_dir = np.array(prev_line.coords[-1]) - np.array(prev_line.coords[-2])
        next_dir = np.array(ln.coords[1]) - np.array(ln.coords[0])

        arc = tangent_arc_between_passes(
            prev_end,
            curr_start,
            prev_dir,
            next_dir,
            lawn,
            hole_polygons
        )

        if arc is not None:
            continuous_coords.extend(list(arc.coords))
            # ~ print(f'arc got {arc}')

        if coords[0] == continuous_coords[-1]:
            continuous_coords.extend(coords[1:])
            # ~ print(f'coords were {coords} | using {coords[1:]}')
        else:
            continuous_coords.extend(coords)
            # ~ print(f'adding {coords}')
        # ~ print('')
 

# ---------------------------------------------------------

# 10. Export continuous path to file

# ---------------------------------------------------------

transformer_inv = Transformer.from_crs(

    f"+proj=tmerc +lat_0={lat0} +lon_0={lon0} +k=1 +x_0=0 +y_0=0",

    "epsg:4326",

    always_xy=True

)

 

with open("mowing_path.txt", "w") as f:
    
    for x, y in continuous_coords:
        
        lon, lat = transformer_inv.transform(x, y)

        f.write(f"{lat} {lon}\n")

 

print("Exported mowing_path.txt with", len(continuous_coords), "points")

 

# ---------------------------------------------------------

# 11. Plot the result

# ---------------------------------------------------------

fig, ax = plt.subplots(figsize=(8, 8))

 

# Lawn boundary

lx, ly = lawn.exterior.xy

ax.plot(lx, ly, color="black", linewidth=2, label="Lawn boundary")

 

# Holes

for hole in holes_local:

    hx, hy = zip(*hole)

    ax.plot(hx, hy, color="orange", linewidth=2, label="No-go zone")

 

# Safe boundary

sx, sy = lawn_safe.exterior.xy

ax.plot(sx, sy, color="green", linestyle="--", label="Safe boundary")

 

# Passes

# ~ for ln in ordered_passes:
    # ~ x, y = ln.xy
    # ~ ax.plot(x, y, color="blue")
    
for ln in plot_fwd:
    x, y = ln.xy
    ax.plot(x, y, color="blue")

for ln in plot_rev:
    x, y = ln.xy
    ax.plot(x, y, color="red")
 

# Continuous path

cx = [p[0] for p in continuous_coords]

cy = [p[1] for p in continuous_coords]

ax.plot(cx, cy, color="red", linewidth=1.5, label="Continuous path")  #hide to see which passes are reversed

 

ax.set_aspect("equal")

ax.legend()

plt.title("Mowing Path with Holes + Turning Arcs")

plt.show()
