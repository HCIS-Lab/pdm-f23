import sys
import cv2
import numpy as np
import open3d as o3d
import quaternion
import matplotlib.pyplot as plt
from os import listdir
from sklearn.neighbors import NearestNeighbors
from tqdm import tqdm
import argparse



def depth_image_to_point_cloud(rgb, depth):
    ### TODO ###
    # Get point cloud from rgb and depth image 
    # - Hint: You need to calculate intrinsic matrix of camera
    raise NotImplementedError
    return pcd


def preprocess_point_cloud(pcd, voxel_size):
    ### TODO ###
    # Do voxelization to reduce the number of points for less memory usage and speedup
    raise NotImplementedError
    return pcd_down


def execute_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size):
    ### TODO ###
    raise NotImplementedError
    return result


def local_icp_algorithm(source_down, target_down, trans_init, threshold):
    ### TODO ###
    # Use Open3D local ICP registration (i.e. point to point or point to plane) 
    raise NotImplementedError
    return result


def my_local_icp_algorithm(source_down, target_down, trans_init, voxel_size):
    ### TODO ###
    # Write your own ICP registration, for the rest, you can also use open3d library.
    raise NotImplementedError
    return result


def reconstruct(args):
    ### TODO ###
    # Use above functions to do scene reconstruction
    # Return reconstructed point cloud and predicted camera poses
    """
    Example:
        ...
        if args.version == 'open3d':
            result_pcd = local_icp_algorithm()
        elif args.veresion == 'my_icp':
            result_pcd = my_local_icp_algorithm()
        ...
    
    Returns:
        result_pcd: point cloud of whole scene from all images
        pred_cam_pos: the predicted camera pose (you can get them during the process)
    """
    
    raise NotImplementedError
    return result_pcd, pred_cam_pos


def get_camera_pos(gt_cam_pos_file):
    data = np.load(gt_cam_pos_file)

    t = data[0, :3]
    q = quaternion.as_quat_array(data[0, 3:])
    matrix = quaternion.as_rotation_matrix(q)
    matrix = np.insert(matrix, 3, matrix @ t, axis=1)  # t = (RT)
    matrix = np.insert(matrix, 3, np.array([0., 0., 0., 1.]), axis=0) # [R|t]

    gt_pose = np.ones((data.shape[0], 4))
    gt_pose[:, :3] = data[:, :3]

    for i in range(data.shape[0]):
        gt_pose[i] = np.linalg.inv(matrix) @ gt_pose[i]
    gt_pose = gt_pose[:, 0:3] / 10. * 255. / 1000.

    return gt_pose


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-f', '--floor', type=int, default=1)
    parser.add_argument('-v', '--version', type=str, default='my_icp', help='open3d or my_icp')
    parser.add_argument('--data_root', type=str, default='data_collection/first_floor/')
    args = parser.parse_args()

    if args.floor == 1:
        args.data_root = "data_collection/first_floor/"
    elif args.floor == 2:
        args.data_root = "data_collection/second_floor/"
    
    result_pcd, pred_cam_pos = reconstruct(args)
    
    gt_cam_pos_file = args.data_root + 'GT_pose.npy'
    gt_cam_pos = get_camera_pos(gt_cam_pos_file)

    # Draw ground truth line
    line = np.array([[i, i+1] for i in range(len(gt_cam_pos)-1)])
    gt_line_set = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(gt_cam_pos),
        lines=o3d.utility.Vector2iVector(line),
    )
    black = [[0, 0, 0] for i in range(len(line))]
    gt_line_set.colors = o3d.utility.Vector3dVector(black)

    # Draw predicted line
    line = np.array([[i, i+1] for i in range(len(pred_cam_pos)-1)])
    pred_line_set = o3d.geometry.LineSet(
            points=o3d.utility.Vector3dVector(pred_cam_pos),
            lines=o3d.utility.Vector2iVector(line),
    )
    red = [[1, 0, 0] for i in range(len(line))]
    pred_line_set.colors = o3d.utility.Vector3dVector(red)

    # Calculate L2 distance
    mean_dist = np.mean(np.linalg.norm(np.asarray(gt_line_set.points) - np.asarray(pred_line_set.points)))
    print("Mean L2 distance:", mean_dist)

    o3d.visualization.draw_geometries([result_pcd, gt_line_set, pred_line_set])