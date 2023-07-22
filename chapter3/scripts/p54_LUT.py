#!/usr/bin/env python
import numpy as np

def get_lut(world_max_x, world_min_x, x_interval,
            world_max_y, world_min_y, y_interval,
            intrinsic_matrix, extrinsic_matrix):

        x_coords_range = np.arange(world_max_x, world_min_x, -x_interval)
        y_coords_range = np.arange(world_max_y, world_min_y, -y_interval)
        top_view_height = len(x_coords_range)
        top_view_width = len(y_coords_range)
        lut_x = np.zeros((top_view_height, top_view_width)).astype(np.float32)
        lut_y = np.zeros((top_view_height, top_view_width)).astype(np.float32)

        for i, world_x in enumerate(x_coords_range):
            for j, world_y in enumerate(y_coords_range):                
                world_coord_point =  np.array([world_x, world_y, 0, 1])

                camera_img_point = intrinsic_matrix @ extrinsic_matrix @ world_coord_point.T
                camera_img_point /= camera_img_point[2]
                
                camera_img_point_u = camera_img_point[0]
                camera_img_point_v = camera_img_point[1]
                lut_x[i][j] = camera_img_point_u
                lut_y[i][j] = camera_img_point_v
        
        return lut_x, lut_y




        