import numpy as np
import open3d as o3d
import argparse

def custom_voxel_down(pcd, voxel_size):
    #TODO: implement your own voxel down
    raise NotImplementedError

def reconstruct(args):
    #TODO: reconstruct the 3d semantic map
    raise NotImplementedError

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-f', '--floor', type=int, default=1)
    parser.add_argument('--data_root', type=str, default='data_collection/first_floor')
    args = parser.parse_args()
