#!/usr/bin/env python
import numpy as np


class LUT:
    def __init__(self, world_max_x, world_min_x, x_interval,
                 world_max_y, world_min_y, y_interval,
                 intrinsic_matrix, extrinsic_matrix):

        self.intrinsic_matrix = intrinsic_matrix 
        self.extrinsic_matrix = extrinsic_matrix
    
        self.x_coords_range = np.arange(world_max_x, world_min_x, -x_interval)
        self.y_coords_range = np.arange(world_max_y, world_min_y, -y_interval)

        top_view_height = len(self.x_coords_range)
        top_view_width = len(self.y_coords_range)

        self.lut_x = np.zeros((top_view_height, top_view_width)).astype(np.float32)
        self.lut_y = np.zeros((top_view_height, top_view_width)).astype(np.float32)


    def get_lut(self):

        for i, world_x in enumerate(self.x_coords_range):
            for j, world_y in enumerate(self.y_coords_range):
                
                world_coord_point =  [world_x, world_y, 0, 1]

                camera_coord = self.extrinsic_matrix @ world_coord_point
                camera_img_point = self.intrinsic_matrix @ camera_coord
                
                # ???????????????
                camera_img_point /= camera_img_point[2]
                
                camera_img_point_u = camera_img_point[0]
                camera_img_point_v = camera_img_point[1]
                self.lut_x[i][j] = camera_img_point_u
                self.lut_y[i][j] = camera_img_point_v
        
        return self.lut_x, self.lut_y