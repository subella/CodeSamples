import gmsh
import sys
import numpy as np
import time

# Constants
# Mesh element size
LC = 0.1
# Inner radius
R1 = 1
# Inner radius of outer rim
R2 = 3
# Outer radius of outer rim
R3 = 4
# Bounds on number of points used to parameterize spline
MIN_NUM_CONTROL_POINTS = 2
MAX_NUM_CONTROL_POINTS = 10
# Bounds on width of spokes (note: this is actually width of free space not spoke)
MIN_SPOKE_WIDTH = 0.1
MAX_SPOKE_WIDTH = 1.9
# Bounds on number of spokes
MIN_NUM_SPOKES = 4
MAX_NUM_SPOKES = 10
# Safety margin for radius and angle of control points
ETA_R = 0.2
ETA_TH = 0.03

factory = gmsh.model.geo

def create_bspline(points):
    bspline = factory.addBSpline(points)
    return [bspline]

def create_bezier(points):
    bezier = factory.addBezier(points)
    return [bezier]

def create_spline(points):
    spline = factory.addSpline(points)
    return [spline]

def create_lines(points):
    lines = []
    for i in range(len(points) - 1):
        lines.append(factory.addLine(points[i], points[i+1]))
    return lines

def create_arc_endpoints(start_pt, mid_pt, end_pt):
    pt1 = factory.addPoint(start_pt[0], start_pt[1], 0)
    pt2 = factory.addPoint(mid_pt[0], mid_pt[1], 0)
    pt3 = factory.addPoint(end_pt[0], end_pt[1], 0)
    return pt1, pt2, pt3

def create_arc(start_pt, end_pt, center=(0,0)):
    pt1, _, pt2 = create_arc_endpoints(start_pt, end_pt)
    pt_center = factory.addPoint(center[0], center[1], 0)
    c = factory.addCircleArc(pt1, pt_center, pt2)
    return pt1, pt2, c

def create_circle(r, center=(0,0)):
    pt1 = factory.addPoint(center[0], center[1] + r, 0)
    pt2 = factory.addPoint(center[0], center[1] - r, 0)
    pt_center = factory.addPoint(center[0], center[1], 0)
    c1 = factory.addCircleArc(pt1, pt_center, pt2)
    c2 = factory.addCircleArc(pt2, pt_center, pt1)
    cl = factory.addCurveLoop([c1, c2])
    return cl

def create_segment_spacings(num_spokes, spoke_width):
    spacings = []
    for i in range(0, 2 * num_spokes):
        # Spoke segment
        if i % 2 != 0:
            length = ((i / num_spokes) * np.pi - spacings[-1]) * spoke_width + spacings[-1]
        else:
            length = (i / num_spokes) * np.pi
        spacings.append(length)
    return spacings

def create_point_field(pts, r, th):
    # Mirror points in local ref frame (at origin)
    M = np.array([[1, 0],[0, -1]])
    mirrored_pts = pts.dot(M)
    all_pts = np.vstack((pts, mirrored_pts))
    all_pts[:,1] += th

    # Split array back into orig and mirrored sub-arrays
    left_pts = np.vsplit(all_pts, 2)[0]
    right_pts = np.vsplit(all_pts, 2)[1]

    left_pts_gmsh = []
    right_pts_gmsh = []
    for pt in left_pts:
        left_pts_gmsh.append(factory.addPoint(pt[0]*np.cos(pt[1]), pt[0]*np.sin(pt[1]), 0))
    for pt in right_pts:
        right_pts_gmsh.append(factory.addPoint(pt[0]*np.cos(pt[1]), pt[0]*np.sin(pt[1]), 0))
    return left_pts_gmsh, right_pts_gmsh

def get_max_bounds(num_spokes):
    return get_bounds(num_spokes, MAX_SPOKE_WIDTH)

def get_bounds(num_spokes, spoke_width):
    # Add buffer on r to prevent self loops
    low = [R1 + ETA_R, ETA_TH]
    high = [R2 - ETA_R, (np.pi / num_spokes) * spoke_width / 2 - ETA_TH]
    return low, high

def sort_control_points(points):
    # Sort by ascending r
    sorted_points = points[points[:, 0].argsort()]
    return sorted_points

def gen_random_point_field(num_points, 
                           num_spokes,
                           spoke_width):
    low, high = get_bounds(num_spokes, spoke_width)
    points = np.random.uniform(low=low, high=high, size=(num_points,2))
    return points

def create_mesh(control_points,
                num_spokes,
                spoke_width,
                parameterization="bspline",
                mesh_resolution="variable",
                filename="mesh.mesh",
                show_gui=False,
                verbose=False):

    gmsh.initialize()

    # Change settings so error is thrown if meshing fails
    #gmsh.option.setNumber('General.AbortOnExit', 2)

    if not verbose:
        gmsh.option.setNumber('General.Terminal', 0)

    control_points = sort_control_points(control_points)

    # Create segment spacings
    segment_spacing = create_segment_spacings(num_spokes, spoke_width)

    cutouts = []
    for i in range(0, len(segment_spacing) - 1, 2):
        # Start and end angles of the arc
        start_th = segment_spacing[i]
        end_th = segment_spacing[i+1]
        mid_th = start_th + (end_th - start_th) / 2

        # Create inner segment arc
        inner_start_loc = (R1 * np.cos(start_th), R1 * np.sin(start_th))
        inner_mid_loc = (R1 * np.cos(mid_th), R1 * np.sin(mid_th))
        inner_end_loc = (R1 * np.cos(end_th), R1 * np.sin(end_th))
        inner_start_pt, inner_mid_pt, inner_end_pt = create_arc_endpoints(inner_start_loc, inner_mid_loc, inner_end_loc)

        # Create outer segment arc
        outer_start_loc = (R2 * np.cos(start_th), R2 * np.sin(start_th))
        outer_mid_loc = (R2 * np.cos(mid_th), R2 * np.sin(mid_th))
        outer_end_loc = (R2 * np.cos(end_th), R2 * np.sin(end_th))
        outer_start_pt, outer_mid_pt, outer_end_pt = create_arc_endpoints(outer_start_loc, outer_mid_loc, outer_end_loc)

        # Create spokes between inner and outer arcs
        r = R1 + (R2 - R1) / 2
        th = start_th + (end_th - start_th) / 2
        left_pts, right_pts = create_point_field(control_points, r, th)

        ordered_points_arr = [inner_start_pt] + right_pts + [outer_start_pt] +\
                             [outer_mid_pt] + [outer_end_pt] + left_pts[::-1] +\
                             [inner_end_pt] + [inner_mid_pt] + [inner_start_pt]

        # From point field, create spoke according to parameterization.
        if parameterization == "line":
            curve = create_lines(ordered_points_arr)

        elif parameterization == "spline":
            curve = create_spline(ordered_points_arr)

        elif parameterization == "bezier":
            curve = create_bezier(ordered_points_arr)

        elif parameterization == "bspline":
            curve = create_bspline(ordered_points_arr)

        else:
            print("Invalid parameterization! Valid options are: 'line', 'spline',\
                    'bezier', 'bspline'")
            return

        cutout = factory.addCurveLoop(curve)
        cutouts.append(cutout) 

    # Generate outer rim
    outer_rim_loop = create_circle(R3)

    # Create planes
    surface = factory.addPlaneSurface([outer_rim_loop] + cutouts)
    gmsh.model.addPhysicalGroup(2, [surface])

    # Refine mesh
    if mesh_resolution == "constant":
        gmsh.option.setNumber("Mesh.MeshSizeFactor", LC);

    elif mesh_resolution == "variable":
        gmsh.model.mesh.field.add("Distance", 1)
        gmsh.model.mesh.field.setNumbers(1, "CurvesList", cutouts)
        gmsh.model.mesh.field.setNumber(1, "NumPointsPerCurve", 100)
        gmsh.model.mesh.field.add("Threshold", 2)
        gmsh.model.mesh.field.setNumber(2, "InField", 1)
        gmsh.model.mesh.field.setNumber(2, "SizeMin", LC / 1.5)
        gmsh.model.mesh.field.setNumber(2, "SizeMax", 3 * LC)
        gmsh.model.mesh.field.setNumber(2, "DistMin", 0.1)
        gmsh.model.mesh.field.setNumber(2, "DistMax", 0.5)
        gmsh.model.mesh.field.setAsBackgroundMesh(2)

    # Write mesh
    factory.synchronize()
    try:
        gmsh.model.mesh.generate(2)
        success = True
    except:
        success = False
    gmsh.write(filename)
    if show_gui:
         # Launch the GUI to see the results:
         gmsh.fltk.run()
    gmsh.finalize()

    return success

def create_random_mesh(num_control_points_bounds=(MIN_NUM_CONTROL_POINTS, 
                                                  MAX_NUM_CONTROL_POINTS),
                       num_spokes_bounds=(MIN_NUM_SPOKES, 
                                          MAX_NUM_SPOKES),
                       spoke_width_bounds=(MIN_SPOKE_WIDTH,
                                           MAX_SPOKE_WIDTH)):

    # Generate random values for free parameters.
    num_control_points = np.random.randint(num_control_points_bounds[0],
                                           num_control_points_bounds[1])

    num_spokes = np.random.randint(num_spokes_bounds[0],
                                   num_spokes_bounds[1])

    spoke_width = np.random.uniform(spoke_width_bounds[0],
                                    spoke_width_bounds[1])

    # Create control points
    control_points = gen_random_point_field(num_control_points, 
                                            num_spokes,
                                            spoke_width)

    # Create mesh
    create_mesh(control_points,
                num_spokes,
                spoke_width,
                parameterization="bspline",
                mesh_resolution="variable",
                show_gui=True)

def time_100():
    start_time = time.time()
    for i in range(100):
        create_random_mesh()
    end_time = time.time()
    print((end_time - start_time) / 100)


if __name__=="__main__":
    start = time.time()
    create_random_mesh()
    end = time.time()
    print("Total time", end - start)
