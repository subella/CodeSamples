# ---
# jupyter:
#   jupytext:
#     text_representation:
#       extension: .py
#       format_name: light
#       format_version: '1.5'
#       jupytext_version: 1.13.7
#   kernelspec:
#     display_name: Python 3
#     language: python
#     name: python3
# ---

# # Teaser Evaluation Setup

import json
import seaborn as sns
from scipy.spatial.transform import Rotation as Rotation
import pandas as pd

# Create some helper functions for coordinate frame conversion and Unity to ENU conversion:

# +
# Converts Unity's left-handed coordinate system to ENU
# by swapping the y and z axes.
UNITY_TO_ENU = np.array([[1,0,0,0],
                         [0,0,1,0],
                         [0,1,0,0],
                         [0,0,0,1]])

CAMERA_TO_ENU = np.array([[1,0,0,0],
                         [0,0,1,0],
                         [0,-1,0,0],
                         [0,0,0,1]])


# +
def transform_kpts(src, transformation):
    """
    Transforms array of positions by the transformation.

    Parameters
    ----------
    src : 3,x np array
        Positions in the original frame.
    transformation : 4,4 np array
        Transformation matrix.
        
    Returns
    -------
    3,x np array
        Positions in the transformed frame.

    """
    ones = np.ones(src.shape[1])
    src_homo_T = np.vstack((src, ones)).T
    dst = (src_homo_T.dot(transformation.T)).T[:3]
    return dst

def kpts_unity_to_enu_np(kpts_unity):
    """
    Converts a Unity keypoints dict to numpy array in ENU.

    Parameters
    ----------
    unity_keypoints : dict
        Contains keys "x", "y", "z" for positions
        in Unity's left-handed frame.

    Returns
    -------
    3x20 np array
        Array of keypoint positions.
        Rows are x,y,z.

    """
    kpts_np = np.zeros((3, len(kpts_unity)))
    for kpt_id in range(len(kpts_unity)):
        kpt = kpts_unity[kpt_id]
        kpt_np = pos_unity_to_np(kpt)
        kpts_np[:,kpt_id] = kpt_np.reshape((3,))
    kpts_enu = transform_kpts(kpts_np, UNITY_TO_ENU)
    return kpts_enu

def pos_unity_to_np(pos_unity):
    """
    Converts a Unity position dict to numpy array.

    Parameters
    ----------
    pos_unity : dict
        Contains keys "x", "y", "z" for position
        in Unity's left-handed frame.

    Returns
    -------
    3, np array
        Position vector in np format.

    """
    pos_np = np.array([pos_unity["x"], pos_unity["y"], pos_unity["z"]])
    return pos_np

def quat_unity_to_np(quat_unity):
    """
    Converts a Unity quaternion dict to numpy array.

    Parameters
    ----------
    quat_unity : dict
        Contains keys "x", "y", "z", "w" for quaternion
        in Unity's left-handed frame.

    Returns
    -------
    4, np array
        Quaternion in np format.

    """
    quat_np = np.array([quat_unity["x"], quat_unity["y"], quat_unity["z"], quat_unity["w"]])
    return quat_np

def quat_to_rot_matrix(quat):
    """
    Converts a quaternion to rotation matrix.

    Parameters
    ----------
    quat : 4, np array
        Quaternion in numpy format x, y, z, w.

    Returns
    -------
    3,3 np array
        Rotation matrix.

    """
    r = Rotation.from_quat(quat)
    rot_np = r.as_matrix()
    return rot_np
    
def make_transformation_matrix(rot, trans):
    """
    Creates a transformation matrix.

    Parameters
    ----------
    rot : 3,3 np array
        Rotation matrix.
    tras : 3, np array
        Translation vector.

    Returns
    -------
    4,4 np array
        Transformation matrix.

    """
    transformation = np.empty((4,4))
    transformation[:3, :3] = rot
    transformation[:3, 3] = trans
    transformation[3, :] = [0, 0, 0, 1]
    return transformation
        
def unity_to_enu_transformation(pos_unity, quat_unity):
    """
    Creates a transformation matrix from Unity to ENU.

    Parameters
    ----------
    pos_unity : dict
        Contains keys "x", "y", "z" for position
        in Unity's left-handed frame.
    quat_unity : dict
        Contains keys "x", "y", "z", "w" for quaternion
        in Unity's left-handed frame.

    Returns
    -------
    4,4 np array
        Transformation matrix.

    """
    
    quat_np = quat_unity_to_np(quat_unity)
    rot_matrix = quat_to_rot_matrix(quat_np)
    if pos_unity is not None:
        pos = pos_unity_to_np(pos_unity)
    else:
        pos = np.zeros((3,))
    
    transformation = make_transformation_matrix(rot_matrix, pos)
    unity_to_enu = UNITY_TO_ENU.dot(transformation).dot(UNITY_TO_ENU)
    return unity_to_enu


# -

# Evaluation and Visualization code:

# +
def plot_teaser(src, dst, est_dst=None, dst_gt_pose=None):
    """
    Plots keypoints for teaser visualization.

    Parameters
    ----------
    src : 3,x np array
        Keypoints in the target frame.
    dst : 3,x np array
        Ground truth keypoints wrt the camera frame.
    est_dst: 3,x np array
        dst keypoints computed using Teaser's pose 
        estimate and transforming each src keypoint.
    dst_gt_pose : 3,x np array
        dst keypoints computed using the ground truth
        pose and transforming each src keypoint.

    """
    # %matplotlib inline
    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(src[0], src[1], src[2], c='green', label='src')
    ax.scatter(dst[0], dst[1], dst[2], c='red', label='dst')
    vals = np.hstack((src, dst))
    if est_dst is not None:
        vals = np.hstack((vals, est_dst))
        ax.scatter(est_dst[0], est_dst[1], est_dst[2], c='purple', label='est_dst')
    if dst_gt_pose is not None:
        vals = np.hstack((vals, dst_gt_pose))
        ax.scatter(dst_gt_pose[0], dst_gt_pose[1], dst_gt_pose[2], c='orange', label='transformed_dst')
    ax.legend()
    ax.set_box_aspect((np.ptp(vals[0]), np.ptp(vals[1]), np.ptp(vals[2])))
    plt.show()
    
def plot_samples(est_dst_kpts_arr, gt_dst_kpts_arr, est_transformation_arr, num_samples=3):
    for i in range(num_samples):
        est_dst_transformed = transform_kpts(src, est_transformation_arr[i])
        plot_teaser(src, gt_dst_kpts_arr[i], est_dst_kpts_arr[i], est_dst_transformed)
        
def evaluate_teaser(est_transformation_arr, gt_transformation_arr, data_name="Data"):
    """
    Computes error metrics on Teaser's performance.

    Parameters
    ----------
    est_transformation_arr : x,4,4 np array
        Estimated positions for each image.
    gt_transformation_arr : x,4,4 np array
        Ground truth positions for each image.

    """
    est_rot_arr = est_transformation_arr[:,:3,:3]
    gt_rot_arr = gt_transformation_arr[:,:3,:3]
    est_pos_arr = est_transformation_arr[:,:3,3]
    gt_pos_arr = gt_transformation_arr[:,:3,3]

    # Rotation error represents the angle between estimated and ground truth matrices:
    # theta = arccos( (tr(R) - 1) / 2), R = Rd'R
    R = np.matmul(gt_rot_arr.transpose((0,2,1)), est_rot_arr)

    cos_theta = np.clip((np.trace(R, axis1=1, axis2=2) - 1)/2, -1, 1)
    # rot_err_norms ia a (n,) array with each entry the rotation error formula above.
    rot_error_norms = np.arccos(cos_theta)
    rot_error_avg = np.mean(rot_error_norms)
    
    # pos_error_norms is a (n,) array with each entry the error norm for the
    # corresponding image.
    pos_error_norms = np.linalg.norm(est_pos_arr - gt_pos_arr, axis=1)
    # pos_error_avg averages all of the error norms for each image.
    pos_error_avg = np.mean(pos_error_norms)
    
    pos_error_norms = pd.DataFrame(pos_error_norms.T, columns = [data_name])
    rot_error_norms = pd.DataFrame(rot_error_norms.T, columns = [data_name])
    
    return pos_error_norms, rot_error_norms
    
def plot_errors(pos_error_norms, rot_error_norms, pos_max=None, rot_max=None):
    fig, ax = plt.subplots(nrows=1, ncols=2, figsize=(15,7))
    if pos_max is not None:
        ax[0].set_xlim([0, pos_max])
    sns.histplot(pos_error_norms, ax=ax[0]).set_title("Position Errors (mm)")
#     sns.histplot(pos_error_norms, ax=ax[1]).set_title("Position Errors (m)")
    
#     fig, ax = plt.subplots(nrows=1, ncols=1, figsize=(15,7))
    if rot_max is not None:
        ax[1].set_xlim(0, rot_max)
    sns.histplot(rot_error_norms, ax=ax[1]).set_title("Rotation Errors (rad)")
#     sns.histplot(rot_error_norms, ax=ax[1]).set_title("Rotation Errors (rad)")
    
def plot_errors_distance(pos_error_norms, rot_error_norms, gt_transformation_arr, column):
    distances = np.linalg.norm(gt_transformation_arr[:,:3,3], axis=1)
    distances = pd.DataFrame(distances, columns = ["Distance"])
    pos_error_norms = pd.concat([pos_error_norms, distances], axis=1)
    rot_error_norms = pd.concat([rot_error_norms, distances], axis=1)
    fig, ax = plt.subplots(nrows=1, ncols=2, figsize=(15,7))
    sns.regplot("Distance", column, data=pos_error_norms, ax=ax[0]).set_title("{} Position Errors (mm)".format(column))
    sns.regplot("Distance", column, data=rot_error_norms, ax=ax[1]).set_title("{} Rotation Errors (rad)".format(column))
    ax[0].set(xlabel='Distance', ylabel='Error')
    ax[1].set(xlabel='Distance', ylabel='Error')


# -

# Teaser specific utils:

# +
def estimate_pose(solver, src, dst):
    # Populate the parameters    
    solver.solve(src, dst)
    solution = solver.getSolution()
    return solution.translation.copy(), solution.rotation.copy()

def get_src_kpts(data):
    # Positions of the keypoints in the CAD frame.
    src_kpts = data["keypointFrame"]["localKeypoints"]
    src_kpts_wrt_cam = kpts_unity_to_enu_np(src_kpts)
    return src_kpts_wrt_cam

def get_est_tar_wrt_cam_tf(solver, src_kpts_wrt_cam, dst_kpts_wrt_cam):
    """
        Returns the transform of target wrt cam using teaser 
        with ground truth keypoints
    """

    # By construction the transform src wrt world is the identity.
    gt_src_wrt_cam_tf = np.eye(4)

    # Target's tf relative to the keypoints CAD frame.
    est_tar_wrt_src_pos, est_tar_wrt_src_rot = estimate_pose(solver, src_kpts_wrt_cam, dst_kpts_wrt_cam)
    est_tar_wrt_src_tf = make_transformation_matrix(est_tar_wrt_src_rot, est_tar_wrt_src_pos)

    est_tar_wrt_cam_tf = np.matmul(est_tar_wrt_src_tf, gt_src_wrt_cam_tf)
    return est_tar_wrt_cam_tf

def get_gt_tar_wrt_cam_tf(annotation):
    """
        Computes ground truth transformation matrix of target wrt camera.
    """
    gt_tar_wrt_cam_pos_unity = annotation["localPosition"]
    gt_tar_wrt_cam_rot_unity = annotation["localRotation"]
    gt_tar_wrt_cam_tf = unity_to_enu_transformation(gt_tar_wrt_cam_pos_unity, gt_tar_wrt_cam_rot_unity)
    return gt_tar_wrt_cam_tf

def get_gt_tar_wrt_cam_tf_hardware(annotation):
    gt_tar_wrt_cam_pos = annotation["localPosition"].copy()
    gt_tar_wrt_cam_rot = annotation["localRotation"].copy()
    gt_tar_wrt_cam_rot.append(gt_tar_wrt_cam_rot.pop(0))
    gt_tar_wrt_cam_rot = quat_to_rot_matrix(gt_tar_wrt_cam_rot)
    gt_tar_wrt_cam_tf = make_transformation_matrix(gt_tar_wrt_cam_rot, gt_tar_wrt_cam_pos)
    return gt_tar_wrt_cam_tf


# -

# #  Ground Truth Keypoint Detections

# +
def get_all_ids(visibility):
    """
    Placeholder function that does not filter out any 
    invisible keypoints
    """
    best_ids = np.indices(visibility.shape)[0]
    return best_ids
    
def get_best_ids(visibility):
    """
    Returns an index array containing the visible keypoints.
    """
    best_ids = np.argpartition(visibility, -15)[-15:]
    return best_ids
    
def get_gt_kpts(annotation):
    """
    Returns the true ground truth world coordinates.
    """
    gt_kpts_wrt_cam_unity = annotation["groundTruthKeypoints"]
    dst_kpts_wrt_cam = kpts_unity_to_enu_np(gt_kpts_wrt_cam_unity)
    
    # TODO: Visibility information is only store in the "keypoints" entry.
    gt_kpts_wrt_cam_pix_unity = annotation["keypoints"]
    gt_kpts_wrt_cam_pix = kpts_pix_unity_to_np(gt_kpts_wrt_cam_pix_unity)
    visibility = gt_kpts_wrt_cam_pix[:,2]
    dst_kpts_wrt_cam = np.vstack((dst_kpts_wrt_cam, visibility.T))
    
    return dst_kpts_wrt_cam

def get_gt_kpts_hardware(annotation):
    dst_kpts_wrt_cam = np.array(annotation["groundTruthKeypoints"]).T
    print (dst_kpts_wrt_cam)
    return dst_kpts_wrt_cam


# -

# # Depth Keypoint Detections

# +
def pixels_to_meters(px, py, z, K):
    """
        Convert pixel coordinates + depth to world coordinates.
    """
    fx = K[0,0]
    fy = K[1,1]
    cx = K[0,2]
    cy = K[1,2]
    x = (px - cx) * z / fx
    y = (py - cy) * z / fy
    pos = np.array([x, y, z])
    return pos

def kpts_pix_unity_to_np(kpts, filter_visible=True):
    """
        Convert a list of kpts in the form 
        [px1, py1, v1, ..., pxn, pyn, vn]
        to:
        [[px1,py1],
         ...
         [pxn, pyn]]
    """
    kpts_np = np.array(kpts)
    kpts_np = kpts_np.reshape((len(kpts)//3, 3))
    return kpts_np

def get_depth_kpts(annotation, kpts_wrt_cam_pix):
    """
        Returns keypoints in meters using depth info and
        ground truth pixel coordinates.
    """
    depth_image_name = teaser_folder + annotation["depth_file_name"]
    depth_image_rgba = cv2.imread(depth_image_name,cv2.IMREAD_UNCHANGED)
    depth_image_bgra = cv2.cvtColor(depth_image_rgba, cv2.COLOR_RGBA2BGRA)
    
    # Converts BGRA to float array.
    coeffs = np.array([1.0, 1/255., 1/(255.**2), 1/(255.**3)])
    depth_image = np.dot(depth_image_bgra, coeffs).astype('float32')  
    depth_image /= 255.0
    
    # Multiply by far render distance to get values in meters. (TODO: hardcoded)
    depth_image *= 50

    px = np.clip(kpts_wrt_cam_pix[:,0].astype(int), 0, 719)
    py = np.clip(kpts_wrt_cam_pix[:,1].astype(int), 0 , 479)
    z = depth_image[py,px]

    # Hardcoded camera matrix from tesse.
    K = np.array([[415.69219381653056, 0.0, 360.0],
                  [0.0, 415.69219381653056, 240.0],
                  [0.0, 0.0, 1.0]])

    # Get the 3D coordinate for each keypoint 
    est_kpts_wrt_cam = pixels_to_meters(px, py, z, K)
    
    # Convert camera coordinate frame to ENU
    est_kpts_wrt_cam_enu = transform_kpts(est_kpts_wrt_cam, CAMERA_TO_ENU)
    
    # Append visibility info to array.
    est_kpts_wrt_cam_enu = np.vstack((est_kpts_wrt_cam_enu, kpts_wrt_cam_pix[:,2].T))
    return est_kpts_wrt_cam_enu

def get_depth_kpts_hardware(annotation, kpts_wrt_cam_pix):
    """
        Returns keypoints in meters using depth info and
        ground truth pixel coordinates.
    """
    depth_image_name = teaser_folder + annotation["depth_file_name"]
    depth_image = cv2.imread(depth_image_name,cv2.IMREAD_UNCHANGED)   


    px = np.clip(kpts_wrt_cam_pix[:,0].astype(int), 0, 1279)
    py = np.clip(kpts_wrt_cam_pix[:,1].astype(int), 0 , 719)
    z = depth_image[py,px]

    # Hardcoded camera matrix from tesse.
    K = np.array([[629.1040649414062, 0.0, 637.203369140625], 
                  [0.0, 628.583251953125, 380.56463623046875], 
                  [0.0, 0.0, 1.0]])

    # Get the 3D coordinate for each keypoint 
    est_kpts_wrt_cam = pixels_to_meters(px, py, z, K)
    
    # Convert camera coordinate frame to ENU
#     est_kpts_wrt_cam_enu = transform_kpts(est_kpts_wrt_cam, CAMERA_TO_ENU)
    
    # Append visibility info to array.
    est_kpts_wrt_cam_enu = np.vstack((est_kpts_wrt_cam, kpts_wrt_cam_pix[:,2].T))
    return est_kpts_wrt_cam_enu

def get_depth_gt_kpts(annotation):
    gt_kpts_wrt_cam_pix_unity = annotation["keypoints"]
    # The ground truth keypoints' pixels (px, py) wrt to the camera frame using
    # a camera coordinate system. (z forward, x right, y down)
    gt_kpts_wrt_cam_pix = kpts_pix_unity_to_np(gt_kpts_wrt_cam_pix_unity)
    return get_depth_kpts(annotation, gt_kpts_wrt_cam_pix)

def get_depth_gt_kpts_hardware(annotation):
    gt_kpts_wrt_cam_pix_unity = annotation["keypoints"]
    # The ground truth keypoints' pixels (px, py) wrt to the camera frame using
    # a camera coordinate system. (z forward, x right, y down)
    gt_kpts_wrt_cam_pix = kpts_pix_unity_to_np(gt_kpts_wrt_cam_pix_unity)
    return get_depth_kpts_hardware(annotation, gt_kpts_wrt_cam_pix)


# -

# # Neural Net + Depth Keypoint Predictions

# +
def get_nn_visible_ids(visibility):
    visible_ids = np.where(visibility > 0.0)[0]
    return visible_ids

def get_depth_nn_kpts(annotation):
    rgb_image_name = teaser_folder + annotation["file_name"]
    bgr_image = cv2.imread(rgb_image_name)
    rgb_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
    
    model = model_no_detectron
    est_kpts_wrt_cam_pix = model(rgb_image)
    
    if est_kpts_wrt_cam_pix is None:
        return None
    
    est_kpts_wrt_cam_pix = est_kpts_wrt_cam_pix.cpu().detach().numpy()[:,:3]
    return get_depth_kpts(annotation, est_kpts_wrt_cam_pix)

def get_depth_nn_kpts_hardware(annotation):
    rgb_image_name = teaser_folder + annotation["depth_file_name"]
    rgb_image_name = rgb_image_name.replace("depth", "rgb")
    rgb_image = cv2.imread(rgb_image_name)
    
#     model = model_no_detectron
    est_kpts_wrt_cam_pix = model(rgb_image)
    
    if est_kpts_wrt_cam_pix is None:
        return None
    
    est_kpts_wrt_cam_pix = est_kpts_wrt_cam_pix.cpu().detach().numpy()[:,:3]
    return get_depth_kpts_hardware(annotation, est_kpts_wrt_cam_pix)    


# -

# # Teaser Evaluation

def compute_test_set(data, solver_params, get_est_kpts, filter_best=False):

    annotations = data["annotations"]
    # TODO: Make consistent between unity and hardware.
#     gt_src_kpts_wrt_cam = get_src_kpts(data)
    gt_src_kpts_wrt_cam = np.array(data['keypointFrame']['localKeypoints']).T
    
    gt_transformation_arr = np.zeros((len(annotations),4,4))
    est_transformation_arr = np.zeros((len(annotations),4,4))
    gt_dst_kpts_arr = np.zeros((len(annotations), 
                                gt_src_kpts_wrt_cam.shape[0],
                                gt_src_kpts_wrt_cam.shape[1]))
    est_dst_kpts_arr = np.zeros((len(annotations), 
                                gt_src_kpts_wrt_cam.shape[0],
                                gt_src_kpts_wrt_cam.shape[1]))
    failures = 0
    
    for annotation_id, annotation in enumerate(annotations):
        solver = teaserpp_python.RobustRegistrationSolver(solver_params)

        gt_dst_kpts_wrt_cam = get_gt_kpts_hardware(annotation)
        est_dst_kpts_wrt_cam = get_est_kpts(annotation)
        
        if est_dst_kpts_wrt_cam is None:
            print("Error getting keypoints")
            failures += 1
            continue
        
        if filter_best:
            est_best_ids = get_best_ids(est_dst_kpts_wrt_cam[3])
            # TODO: IT's probably better to just leave keypoints as (n,4) and transpose
            # for final step.
            src = (gt_src_kpts_wrt_cam.T[est_best_ids].T)[:3]
            dst = (est_dst_kpts_wrt_cam.T[est_best_ids].T)[:3]
        else:
            src = gt_src_kpts_wrt_cam[:3]
            dst = est_dst_kpts_wrt_cam[:3]
        
        
        if (src.shape[1] <= 3):
            print("Too little keypoints!")
            failures += 1
            continue
        
        gt_tar_wrt_cam_tf = get_gt_tar_wrt_cam_tf_hardware(annotation)
        est_tar_wrt_cam_tf = get_est_tar_wrt_cam_tf(solver, 
                                                    src, 
                                                    dst)
        
        print("GT", gt_tar_wrt_cam_tf)
        print("EST", est_tar_wrt_cam_tf)
        
        gt_transformation_arr[annotation_id] = gt_tar_wrt_cam_tf
        est_transformation_arr[annotation_id] = est_tar_wrt_cam_tf
        
        gt_dst_kpts_arr[annotation_id] = gt_dst_kpts_wrt_cam[:3]
        est_dst_kpts_arr[annotation_id] = est_dst_kpts_wrt_cam[:3]
        
        plot_teaser(src, dst, gt_dst_kpts_wrt_cam[:3], transform_kpts(src, gt_tar_wrt_cam_tf))
    
    print("{} out of {} images failed".format(failures, len(annotations)))
    return gt_transformation_arr, est_transformation_arr, gt_dst_kpts_arr, est_dst_kpts_arr

# Plot the first annotated keypoint frame for a sanity check:

# +
teaser_folder = "/home/subella/KeypointDetectorData/DataSets/dataset_home_subella_Downloads_2022-01-12-10-27-58.mat/Validation/"

# Open the data folder and parse the metadata into a python dict.
with open(teaser_folder + "metadata.json") as json_file:
    data = json.load(json_file)

# +
src_kpts = data["keypointFrame"]["localKeypoints"]
dst_kpts = data["annotations"][0]["groundTruthKeypoints"]

# src = kpts_unity_to_enu_np(src_kpts)
# dst = kpts_unity_to_enu_np(dst_kpts)

src = np.array(src_kpts)
dst = np.array(dst_kpts)

plot_teaser(src.T, dst.T)
# -

# Initialize the solver:

# +
import teaserpp_python

# Initialize Teaser Solver
solver_params = teaserpp_python.RobustRegistrationSolver.Params()
solver_params.cbar2 = 1
solver_params.noise_bound = 5
solver_params.estimate_scaling = False
solver_params.rotation_estimation_algorithm = (
    teaserpp_python.RobustRegistrationSolver.ROTATION_ESTIMATION_ALGORITHM.GNC_TLS
)
solver_params.rotation_gnc_factor = 1.4
solver_params.rotation_max_iterations = 100
solver_params.rotation_cost_threshold = 1e-6

print("TEASER++ Parameters are:", solver_params)


# +
def evaluate_comparison(data, solver_params):
    gt_transformation_arr, est_transformation_arr,\
    gt_dst_kpts_arr, est_dst_kpts_arr = compute_test_set(data, solver_params, get_gt_kpts_hardware)
    gt_pos_error_norms, gt_rot_error_norms = evaluate_teaser(est_transformation_arr, 
                                                             gt_transformation_arr, 
                                                             data_name="Ground Truth")
    
    
#     gt_transformation_arr, est_transformation_arr,\
#     gt_dst_kpts_arr, est_dst_kpts_arr = compute_test_set(data, solver_params, get_gt_kpts, filter_best=True)
#     gt_pos_error_norms_filt, gt_rot_error_norms_filt = evaluate_teaser(est_transformation_arr,
#                                                                        gt_transformation_arr,
#                                                                        data_name="Ground Truth Filt")

    gt_transformation_arr, est_transformation_arr,\
    gt_dst_kpts_arr, est_dst_kpts_arr = compute_test_set(data, solver_params, get_depth_gt_kpts_hardware)
    depth_pos_error_norms, depth_rot_error_norms = evaluate_teaser(est_transformation_arr, 
                                                                   gt_transformation_arr,
                                                                   data_name="Depth")

#     gt_transformation_arr, est_transformation_arr,\
#     gt_dst_kpts_arr, est_dst_kpts_arr = compute_test_set(data, solver_params, get_depth_gt_kpts, filter_best=True)
#     depth_pos_error_norms_filt, depth_rot_error_norms_filt = evaluate_teaser(est_transformation_arr, 
#                                                                              gt_transformation_arr,
#                                                                              data_name="Depth Filt")

    gt_transformation_arr, est_transformation_arr,\
    gt_dst_kpts_arr, est_dst_kpts_arr = compute_test_set(data, solver_params, get_depth_nn_kpts_hardware)
    nn_pos_error_norms, nn_rot_error_norms = evaluate_teaser(est_transformation_arr, 
                                                             gt_transformation_arr,
                                                             data_name="Detector")

#     gt_transformation_arr, est_transformation_arr,\
#     gt_dst_kpts_arr, est_dst_kpts_arr = compute_test_set(data, solver_params, get_depth_nn_kpts, filter_best=True)
#     nn_pos_error_norms_filt, nn_rot_error_norms_filt = evaluate_teaser(est_transformation_arr,
#                                                                        gt_transformation_arr,
#                                                                        data_name="Detector Filt")

#     pos_error_norms = pd.concat([gt_pos_error_norms, gt_pos_error_norms_filt,
#                                  depth_pos_error_norms, depth_pos_error_norms_filt,
#                                  nn_pos_error_norms, nn_pos_error_norms_filt], axis=1)
    
#     rot_error_norms = pd.concat([gt_rot_error_norms, gt_pos_error_norms_filt,
#                                  depth_rot_error_norms, depth_rot_error_norms_filt,
#                                  nn_rot_error_norms, nn_rot_error_norms_filt], axis=1)

    pos_error_norms = pd.concat([gt_pos_error_norms, depth_pos_error_norms, nn_pos_error_norms], axis=1)
    
    rot_error_norms = pd.concat([gt_rot_error_norms, depth_rot_error_norms, nn_rot_error_norms], axis=1)

    return pos_error_norms, rot_error_norms, gt_transformation_arr

def plot_comparison(pos_error_norms, rot_error_norms):
    # Plotting
    plot_errors(pos_error_norms[["Depth", "Depth Filt"]], 
                rot_error_norms[["Depth", "Depth Filt"]],
                pos_max=0.05, rot_max=0.5)
    
    plot_errors(pos_error_norms[["Detector", "Detector Filt"]], 
                rot_error_norms[["Detector", "Detector Filt"]],
                pos_max=None, rot_max=None)
    
    plot_errors(pos_error_norms[["Ground Truth", "Depth", "Detector"]], 
                rot_error_norms[["Ground Truth", "Depth", "Detector"]],
                pos_max=0.05, rot_max=0.2)


# -

# # Easy Test Set

# +
teaser_folder = "/home/subella/KeypointDetectorData/DataSets/dataset_home_subella_Downloads_2022-01-12-10-27-58.mat/Validation/"

# Open the data folder and parse the metadata into a python dict.
with open(teaser_folder + "metadata.json") as json_file:
    data = json.load(json_file)
# -

easy_pos_error_norms, easy_rot_error_norms, gt_transformation_arr = evaluate_comparison(data, solver_params)

print(easy_pos_error_norms[["Depth"]].std())
print(easy_pos_error_norms[["Detector"]].std())

# +
# plot_comparison(pos_error_norms, rot_error_norms)
plot_errors(easy_pos_error_norms[["Ground Truth"]], 
            easy_rot_error_norms[["Ground Truth"]],
            pos_max=300, rot_max=0.3)

plot_errors(easy_pos_error_norms[["Depth"]], 
            easy_rot_error_norms[["Depth"]],
            pos_max=300, rot_max=0.3)

plot_errors(easy_pos_error_norms[["Detector"]], 
            easy_rot_error_norms[["Detector"]],
            pos_max=300, rot_max=0.3)

# plot_errors(easy_pos_error_norms[["Detector", "Detector Filt"]], 
#             easy_rot_error_norms[["Detector", "Detector Filt"]],
#             pos_max=0.1, rot_max=0.5)

plot_errors(easy_pos_error_norms[["Ground Truth", "Depth", "Detector"]], 
            easy_rot_error_norms[["Ground Truth", "Depth", "Detector"]],
            pos_max=300, rot_max=0.3)
# -

plot_errors_distance(easy_pos_error_norms, easy_rot_error_norms, gt_transformation_arr, "Ground Truth")
plot_errors_distance(easy_pos_error_norms, easy_rot_error_norms, gt_transformation_arr, "Depth")
plot_errors_distance(easy_pos_error_norms, easy_rot_error_norms, gt_transformation_arr, "Detector")

# # Hard Test Set

# +
teaser_folder = "/home/subella/TestSetHard/Validation/"

# Open the data folder and parse the metadata into a python dict.
with open(teaser_folder + "metadata.json") as json_file:
    data = json.load(json_file)
# -

hard_pos_error_norms, hard_rot_error_norms = evaluate_comparison(data, solver_params)

# +
plot_errors(hard_pos_error_norms[["Depth", "Depth Filt"]], 
            hard_rot_error_norms[["Depth", "Depth Filt"]],
            pos_max=0.1, rot_max=0.5)

plot_errors(hard_pos_error_norms[["Detector", "Detector Filt"]], 
            hard_rot_error_norms[["Detector", "Detector Filt"]],
            pos_max=None, rot_max=None)

plot_errors(hard_pos_error_norms[["Ground Truth", "Depth", "Detector"]], 
            hard_rot_error_norms[["Ground Truth", "Depth", "Detector"]],
            pos_max=0.7, rot_max=0.5)


# -

# # Teaser Ground Truth with Stereo Depth

# Helper functions to calculate focal lengths:

# +
def vfov_from_hfov(hfov, width, height):
    """ Returns horiziontal FOV based on provided vertical FOV and dimensions.

        Based on (this source)[http://paulbourke.net/miscellaneous/lens].

        Args:
            hfov: Horizontal FOV in degrees.
            width: width of image, in pixels.
            height: height of image, in pixels.

        Returns:
            A float representing the vertical FOV of the image in degrees.
    """
    return np.rad2deg(2.0 * np.arctan(np.tan(np.deg2rad(hfov) / 2.0) * height / width))


def hfov_from_vfov(vfov, width, height):
    """ Returns vertical FOV based on provided horizontal FOV and dimensions.

        Based on (this source)[http://paulbourke.net/miscellaneous/lens].

        Args:
            vfov: Vertical FOV in degrees.
            width: width of image, in pixels.
            height: height of image, in pixels.

        Returns:
            A float representing the horizontal FOV of the image in degrees.
    """
    return np.rad2deg(2.0 * np.arctan(np.tan(np.deg2rad(vfov) / 2.0) * width / height))


def fx_from_hfov(hfov, width):
    """ Returns horizontal focal length based on provided horizontal FOV and
        width.

        Based on (this source)[http://paulbourke.net/miscellaneous/lens].

        Args:
            hfov: Horizontal FOV in degrees.
            width: width of the image, in pixels.

        Returns:
            A float representing the horizontal focal length of the image in
            pixels.
    """
    return (width / 2.0) / np.tan(np.deg2rad(hfov) / 2.0)


def fy_from_vfov(vfov, height):
    """ Returns vertical focal length based on provided vertical FOV and
        height.

        Based on (this source)[http://paulbourke.net/miscellaneous/lens].

        Args:
            vfov: Vertical FOV in degrees.
            height: height of the image, in pixels.

        Returns:
            A float representing the vertical focal length of the image in
            pixels.
    """
    return (height / 2.0) / np.tan(np.deg2rad(vfov) / 2.0)


# -

# Lets plot a disparity map for an rgb pair to sanity check:

# +
def stereo_to_3D(left_image, right_image):
    # Get camera params from JSON metadata.
    stereo_baseline = data["cameraParams"]["stereoBaseline"]
    unity_vertical_fov = data["cameraParams"]["fieldOfView"]
    width = data["images"][0]["width"]
    height = data["images"][0]["height"]

    print(unity_vertical_fov)
    print(width)
    print(height)
    # Convert Unity fov to focal lengths.
    unity_horizontal_fov = hfov_from_vfov(unity_vertical_fov, width, height)
    fx = fx_from_hfov(unity_horizontal_fov, width)
    fy = fy_from_vfov(unity_vertical_fov, height)

    assert(np.isclose(fx, fy))

    f = fx
    
    rgb_left = cv2.cvtColor(left_image, cv2.COLOR_BGR2GRAY)
    rgb_right = cv2.cvtColor(right_image, cv2.COLOR_BGR2GRAY)
    stereo = cv2.StereoBM_create()
    cx = width//2
    cy = height//2
    Tx = -stereo_baseline
    Q = np.float32([[1, 0, 0, -cx],
                 [0, 1, 0, -cy],
                 [0, 0, 0, fx],
                 [0, 0, -1/Tx, 0]])

    disparity = stereo.compute(rgb_left, rgb_right).astype(np.float32) / 16.
    points_3D = cv2.reprojectImageTo3D(disparity, Q)
    depth = stereo_baseline * f / disparity
    return depth, disparity, points_3D

left_image = cv2.imread(teaser_folder + "medkit_image_rgb_left_3.png")
right_image = cv2.imread(teaser_folder + "medkit_image_rgb_right_3.png")
depth, disparity, world_points = stereo_to_3D(left_image, right_image)

# depth = stereo_baseline * f / disparity

plt.figure()
plt.imshow(disparity, 'gray')
plt.figure()
plt.imshow(depth, 'gray')
plt.figure()
image_rgb = cv2.cvtColor(left_image,cv2.COLOR_BGR2RGB)
plt.imshow(image_rgb)

# print(points_3D[236, 328])
# -

# Iterate over every image pair and double check that the computed location of each visible keypoint matches with the ground truth:

# +
def kpts_pixels_to_meters(kpts_gt_pixels, world_points):
    world_pos_list = []
    for x, y, v in zip(*[iter(kpts_gt_pixels)]*3):
#         px = kpts_gt_pixels[i*3]
#         py = kpts_gt_pixels[i*3 + 1]
        world_pos = world_points[y, x]
        world_pos_enu = np.array([world_pos[0], world_pos[2], -world_pos[1]])
        world_pos_list.append(world_pos_enu)
    return np.array(world_pos_list).reshape((3,20))

def evaluate_test_set_stereo():
    # Positions of the keypoints in the CAD frame.
    src_kpts = data["keypointFrame"]["localKeypoints"]
    src_kpts_wrt_world = kpts_unity_to_enu_np(src_kpts)

    # By construction the transform src wrt world is the identity.
    gt_src_wrt_world_tf = np.eye(4)
    
    annotations = data["annotations"]
    est_transformation_arr = np.empty((len(annotations),4,4))
    gt_transformation_arr = np.empty((len(annotations),4,4))
    
    for annotation_id in range(0, len(annotations),2):
        left_annotation = annotations[annotation_id]
        right_annotation = annotations[annotation_id + 1]
        left_image = cv2.imread(teaser_folder + left_annotation["file_name"])
        right_image = cv2.imread(teaser_folder + right_annotation["file_name"])
 
        gt_cam_wrt_world_pos_unity = left_annotation["cameraPosition"]
        gt_cam_wrt_world_rot_unity = left_annotation["cameraRotation"]
        gt_cam_wrt_world_tf = unity_to_enu_transformation(gt_cam_wrt_world_pos_unity, 
                                                            gt_cam_wrt_world_rot_unity)
        
        gt_tar_wrt_world_pos_unity = left_annotation["localPosition"]
        gt_tar_wrt_world_rot_unity = left_annotation["localRotation"]
        gt_tar_wrt_world_tf = unity_to_enu_transformation(gt_tar_wrt_world_pos_unity, gt_tar_wrt_world_rot_unity)

        gt_kpts_wrt_cam_pixels = left_annotation["keypoints"]
        _, world_points = stereo_to_3D(left_image, right_image)
        est_kpts_wrt_cam = kpts_pixels_to_meters(gt_kpts_wrt_cam_pixels, world_points)
        est_kpts_wrt_world = transform_kpts(est_kpts_wrt_cam, gt_cam_wrt_world_tf)
#         print ("Estimate", est_kpts_wrt_world)

        # Target's transformation relative to the camera.
        gt_tar_wrt_cam_tf = np.matmul(gt_tar_wrt_world_tf, np.linalg.inv(gt_cam_wrt_world_tf))

        # Keypoints attached to current target's location relative to the world.
        gt_kpts_wrt_world_unity = left_annotation["groundTruthKeypoints"]
        dst_kpts_wrt_world = kpts_unity_to_enu_np(gt_kpts_wrt_world_unity)
#         print("gt", dst_kpts_wrt_world)

        # Target's tf relative to the keypoints CAD frame.
        est_tar_wrt_src_pos, est_tar_wrt_src_rot = estimate_pose(src_kpts_wrt_world, est_kpts_wrt_world)
        est_tar_wrt_src_tf = make_transformation_matrix(est_tar_wrt_src_rot, est_tar_wrt_src_pos)

        # Estimated pose of target wrt camera frame.
        est_tar_wrt_cam_tf = np.matmul(est_tar_wrt_src_tf, np.matmul(gt_src_wrt_world_tf, np.linalg.inv(gt_cam_wrt_world_tf)))                      

        gt_transformation_arr[annotation_id] = gt_tar_wrt_cam_tf
        est_transformation_arr[annotation_id] = est_tar_wrt_cam_tf
        print("gt tf", gt_tar_wrt_cam_tf)
        print("est tf", est_tar_wrt_cam_tf)

    return est_transformation_arr, gt_transformation_arr


# -

stereo_est_transformation_arr, stereo_gt_transformation_arr = evaluate_test_set_stereo()

evaluate_teaser(stereo_est_transformation_arr, stereo_gt_transformation_arr)


# +
def get_visible_kpt_ids(kpts_gt_pixels):
    ids = []
    idx = 0
    for x, y, v in zip(*[iter(kpts_gt_pixels)]*3):
        if v == 2:
            ids.append(idx)
        idx += 1
    return ids

def kpts_pixels_to_world(kpts_gt_pixels, world_points, visible_kpts_ids):
    world_pos_list = []
    
    for i in visible_kpts_ids:
        px = kpts_gt_pixels[i*3]
        py = kpts_gt_pixels[i*3 + 1]
        world_pos = world_points[py, px]
        world_pos_enu = np.array([world_pos[0], world_pos[2], -world_pos[1]])
        world_pos_list.append(world_pos_enu)
    return np.array(world_pos_list)
        
    
#     kpts_world = np.empty((len(kpts_gt_pixels)//3,3))
#     count = 0
#     for x, y, v in zip(*[iter(kpts_gt_pixels)]*3):
#         if v == 2:  
#             world_pos = world_points[y, x]
#             # LAZY SMH
#             world_pos_enu = np.array([world_pos[0], world_pos[2], -world_pos[1]])
#             world_pos_list.append(world_pos_enu)
# #             print(world_pos_list)
#         count += 1
#     return np.array(world_pos_list)
        
def get_visible_gt_kpts(kpts_gt_world, visible_kpts_ids):
    return kpts_gt_world.T[visible_kpts_ids]
            


annotations = data["annotations"]
for annotation_id in range(0, 1, 2):
    left_annotation = annotations[annotation_id]
    right_annotation = annotations[annotation_id + 1]
    left_image = cv2.imread(teaser_folder + left_annotation["file_name"])
    right_image = cv2.imread(teaser_folder + right_annotation["file_name"])
    
    kpts_gt_pixels = left_annotation["keypoints"]
    visible_kpts_ids = get_visible_kpt_ids(kpts_gt_pixels)
    print(visible_kpts_ids)
    _, world_points = stereo_to_3D(left_image, right_image)
#     print(left_annotation["keypoints"])
    stereo_kpts = kpts_pixels_to_world(kpts_gt_pixels, world_points, visible_kpts_ids)
    gt_kpts = get_visible_gt_kpts(kpts_unity_to_enu_np(left_annotation["groundTruthKeypoints"]), visible_kpts_ids)
    src_kpts = data["keypointFrame"]["localKeypoints"]
    src = (kpts_unity_to_enu_np(src_kpts).T[visible_kpts_ids]).T
    est_pos, est_rot = estimate_pose(src, stereo_kpts.T)
    est_transformation = make_transformation_matrix(est_rot, est_pos)
    
    gt_pos_unity = left_annotation["localPosition"]
    gt_rot_unity = left_annotation["localRotation"]
    gt_transformation = unity_to_enu_transformation(gt_pos_unity, gt_rot_unity)
    
    
    
    print ("Stereo", stereo_kpts)
    print ("Ground Truth", gt_kpts)
    print ("Error", stereo_kpts - gt_kpts)
    print ("Estimated Pose", est_transformation)
    print ("GT Pose", gt_transformation)
# -

evaluate_teaser(np.expand_dims(est_transformation,axis=0), np.expand_dims(gt_transformation, axis=0))


# # Teaser with Keypoint Predictions

# +
def get_visible_kpt_ids(kpts_gt_pixels):
    ids = []
    idx = 0
    for x, y, v in zip(*[iter(kpts_gt_pixels)]*3):
        if v == 2:
            ids.append(idx)
        idx += 1
    return ids

def kpts_pixels_to_world(kpts_gt_pixels, world_points, visible_kpts_ids):
    world_pos_list = []
    
    for i in visible_kpts_ids:
        px = kpts_gt_pixels[i*3]
        py = kpts_gt_pixels[i*3 + 1]
        world_pos = world_points[py, px]
        world_pos_enu = np.array([world_pos[0], world_pos[2], -world_pos[1]])
        world_pos_list.append(world_pos_enu)
    return np.array(world_pos_list)
        
    
#     kpts_world = np.empty((len(kpts_gt_pixels)//3,3))
#     count = 0
#     for x, y, v in zip(*[iter(kpts_gt_pixels)]*3):
#         if v == 2:  
#             world_pos = world_points[y, x]
#             # LAZY SMH
#             world_pos_enu = np.array([world_pos[0], world_pos[2], -world_pos[1]])
#             world_pos_list.append(world_pos_enu)
# #             print(world_pos_list)
#         count += 1
#     return np.array(world_pos_list)
        
def get_visible_gt_kpts(kpts_gt_world, visible_kpts_ids):
    return kpts_gt_world.T[visible_kpts_ids]
            

kpt_cfg.MODEL.WEIGHTS = os.path.join(kpt_cfg.OUTPUT_DIR, "model_final.pth")  # path to the model we just trained
kpt_cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = .7  # set a custom testing threshold
kpt_predictor = DefaultPredictor(kpt_cfg)

annotations = data["annotations"]
for annotation_id in range(0, 1, 2):
    left_annotation = annotations[annotation_id]
    right_annotation = annotations[annotation_id + 1]
    left_image = cv2.imread(teaser_folder + left_annotation["file_name"])
    right_image = cv2.imread(teaser_folder + right_annotation["file_name"])

    outputs = kpt_predictor(image)
    predictions = np.array(outputs["instances"].to("cpu").pred_keypoints)
    
    print (predictions.flatten())
    
    kpts_gt_pixels = left_annotation["keypoints"]
    kpts_nn_pixels = predictions.flatten()
    visible_kpts_ids = get_visible_kpt_ids(kpts_gt_pixels)
    print(visible_kpts_ids)
    print(kpts_gt_pixels)
    _, world_points = stereo_to_3D(left_image, right_image)
#     print(left_annotation["keypoints"])
    stereo_kpts = kpts_pixels_to_world(kpts_nn_pixels, world_points, visible_kpts_ids)
    gt_kpts = get_visible_gt_kpts(kpts_unity_to_enu_np(left_annotation["groundTruthKeypoints"]), visible_kpts_ids)
    src_kpts = data["keypointFrame"]["localKeypoints"]
    src = (kpts_unity_to_enu_np(src_kpts).T[visible_kpts_ids]).T
    est_pos, est_rot = estimate_pose(src, stereo_kpts.T)
    est_transformation = make_transformation_matrix(est_rot, est_pos)
    
    gt_pos_unity = left_annotation["localPosition"]
    gt_rot_unity = left_annotation["localRotation"]
    gt_transformation = unity_to_enu_transformation(gt_pos_unity, gt_rot_unity)
    
    
    
    print ("Stereo", stereo_kpts)
    print ("Ground Truth", gt_kpts)
    print ("Error", stereo_kpts - gt_kpts)
    print ("Estimated Pose", est_transformation)
    print ("GT Pose", gt_transformation)


# -




