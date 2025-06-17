from pose_estimation_pkg.libs.contour import Segment
from pose_estimation_pkg.libs.utils.utils import get_slope, compute_angle, apply_rotation,get_rotation_matrix_from_transformation, combine_icp_rotations, rotation_matrix_to_euler,auto_voxel_downsample_pair

from pose_estimation_pkg.libs.utils.icp_utils import align_centers,prepare_dataset, execute_global_registration, draw_registration_result, icp_registration

import open3d as o3d
import cv2
import time
import numpy as np
import copy

class IcpRegistration:
    def __init__(self, pointcloud, package_path, display=False):
        self.voxel_size = 0.005
        self.display = display
        self.package_path = package_path 
        self.pointcloud = pointcloud 
        self.model = Segment(package_path=package_path)
        self.source_pcd = self.get_source_pcd()
        self.identity_matrix = np.asarray([[0.0, 0.0, 1.0, 0.0], [1.0, 0.0, 0.0, 0.0],
                             [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]])

    def get_source_pcd(self):
        o3d.utility.random.seed(42)
        source_path = f"{self.package_path}/libs/datasets/white_connector/mesh/ref_pcd_latest.ply"
        source_pcd = o3d.io.read_point_cloud(source_path)
        return source_pcd

    def read_source_image(self):
        source_imgpath = f"{self.package_path}/libs/datasets/white_connector/images/slanted_ref_image.jpg"

        source_image = cv2.imread(source_imgpath)
        return source_image
    

    def get_pose(self,target_image, target_depth, object_name,white_neg_x_pick_offset,white_pos_x_pick_offset,blue_neg_x_pick_offset,blue_pos_x_pick_offset):
        # Start fresh each time.
        o3d.utility.random.seed(42)
        source_pcd = copy.deepcopy(self.source_pcd)
        segmenation_time = time.time()
        angle_target, slope_sign, contour,line_tip = self.model.get_cntr_angle(target_image)
        seg_time = time.time() - segmenation_time
        print(f"Segmentation time {seg_time}")
        if (angle_target != None):
            target_pcd = self.pointcloud.crop_pcd(contour, target_image, target_depth,dof_6d=True)
            _, centerpoint = self.pointcloud.filter_point_cloud(target_pcd, display=False)
            print("Center Point New:", centerpoint)

            if not isinstance(centerpoint, np.ndarray):
                return None, True

            flip_transform = [[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]]
            target_pcd.transform(flip_transform)

            if self.display:
                o3d.visualization.draw_geometries([target_pcd, source_pcd], window_name="Initial orientation of Source and Target Pointclouds")
            
            angle_difference = angle_target - 90
            angle_rad = np.radians(angle_difference)

            print(f"Rotation angle: {angle_rad:.2f} rads, {angle_difference:.2f} degrees")

            # Align centers or the two pointclouds
            target_pcd = align_centers(source=source_pcd, target=target_pcd)
            source_pcd, rot_ang_marix = apply_rotation(source_pcd, angle_rad)

            if self.display:
                o3d.visualization.draw_geometries([target_pcd, source_pcd], window_name="PCD Orientation after Applying rotation")


            # Down sample the point cloud
            source_down, target_down,voxel_size = auto_voxel_downsample_pair(source_pcd=source_pcd, target_pcd=target_pcd)
            icp = icp_registration(source=source_down, target=target_down,
                                    result_ransac= np.eye(4) ,max_correspondence_distance_fine=voxel_size*15)

            rot_icp_matrix = get_rotation_matrix_from_transformation(icp.transformation)

            if self.display:
                draw_registration_result(source_pcd, target_pcd, icp.transformation,window_name="Local ICP Registration")

            # Get combined angles
            rot_final = combine_icp_rotations(rot_ang_marix, rot_icp_matrix)
            roll, pitch, yaw = rotation_matrix_to_euler(rot_final)
            # Debug
            target_pcd.paint_uniform_color([0, 1, 0])
            source_pcd.paint_uniform_color([1,0, 0])
            
            source_pcd.transform(icp.transformation)
            result_pcd = target_pcd + source_pcd
            print(f"X: {centerpoint[0]:.5f}, Y: {centerpoint[1]:.5f}, Z: {centerpoint[2]:.5f}, "
                f"Roll: {roll:.2f} degrees, Pitch: {pitch:.2f} degrees, Yaw: {yaw:.2f} degrees")

            o3d.io.write_point_cloud("current_pcd.pcd", result_pcd)

            return (centerpoint[0], centerpoint[1], centerpoint[2], roll, pitch ,yaw), True

        
        return None, False


    def shift_center_by_offset(self, center_point, object_name,white_neg_x_pick_offset,white_pos_x_pick_offset,blue_neg_x_pick_offset,blue_pos_x_pick_offset):

        if not isinstance(center_point, np.ndarray):
            return None
        x, y, z = center_point.tolist()
        if x < 0:
            if object_name == "white_connector":
                x += (x*white_neg_x_pick_offset)
            else:
                x += (x*blue_neg_x_pick_offset)
        elif x > 0:
            if object_name == "white_connector":
                x -= (x*white_pos_x_pick_offset)
            else:
                x -= (x*blue_pos_x_pick_offset)
        
        return np.array([x, y, z])



def rotate_yaw_90(rot_final):
    theta = -np.pi / 2  # -90 degrees in radians
    rotation_matrix = np.array([
        [np.cos(theta), -np.sin(theta), 0],
        [np.sin(theta),  np.cos(theta), 0],
        [0,              0,             1]
    ])

    # Apply rotation to the point cloud
    R_combined = rotation_matrix @ rot_final 
    return R_combined