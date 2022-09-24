# -*- coding: utf-8 -*-
"""
ipad pointcloud streaming with open3d
by kentaroy47
"""

import copy
import numpy as np
import open3d as o3d
import cv2
from record3d import Record3DStream
from threading import Event

class DemoApp:
    def __init__(self):
        self.event = Event()
        self.session = None
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window()
        
    def on_new_frame(self):
        """
        This method is called from non-main thread, therefore cannot be used for presenting UI.
        """
        self.event.set()  # Notify the main thread to stop waiting and process new frame.

    def on_stream_stopped(self):
        print('Stream stopped')

    def connect_to_device(self, dev_idx):
        print('Searching for devices')
        devs = Record3DStream.get_connected_devices()
        print('{} device(s) found'.format(len(devs)))
        for dev in devs:
            print('\tID: {}\n\tUDID: {}\n'.format(dev.product_id, dev.udid))

        if len(devs) <= dev_idx:
            raise RuntimeError('Cannot connect to device #{}, try different index.'
                               .format(dev_idx))

        dev = devs[dev_idx]
        self.session = Record3DStream()
        self.session.on_new_frame = self.on_new_frame
        self.session.on_stream_stopped = self.on_stream_stopped
        self.session.connect(dev)  # Initiate connection and start capturing

    def get_intrinsic_mat_from_coeffs(self, coeffs):
        return np.array([[coeffs.fx,         0, coeffs.tx],
                         [        0, coeffs.fy, coeffs.ty],
                         [        0,         0,         1]])

    def create_point_cloud(self):
        depth = self.session.get_depth_frame()
        rgb = cv2.resize(self.session.get_rgb_frame(), np.flip(np.shape(depth)))
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(o3d.geometry.Image(rgb),
                                                                  o3d.geometry.Image(depth),
                                                                  convert_rgb_to_intensity=False)
        return o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, self.intrinsic)

    def start_processing_stream(self):
        self.event.wait()  # Wait for new frame to arrive
        
        # get intrinsic parameters
        intrinsic_mat = self.get_intrinsic_mat_from_coeffs(self.session.get_intrinsic_mat())

        # setup point clouds
        # lidar depth map = 256x192
        # intrinsics work well divided by 4 than the given value from record3d
        self.intrinsic = o3d.camera.PinholeCameraIntrinsic(*np.shape(self.session.get_depth_frame()),
                                                           intrinsic_mat[0,0]/4,
                                                           intrinsic_mat[1,1]/4,
                                                           intrinsic_mat[0,2]/4,
                                                           intrinsic_mat[1,2]/4)
        self.pcd = self.create_point_cloud()
        # add geometry
        self.vis.add_geometry(self.pcd)
        
        # Loop for point clouds
        while True:
            self.event.wait()  # Wait for new frame to arrive

            # update pointclouds
            pcd = self.create_point_cloud()
            self.pcd.points = pcd.points
            self.pcd.colors = pcd.colors
            
            # update geometry
            self.vis.update_geometry(self.pcd)
            self.vis.poll_events()
            self.vis.update_renderer()
    
            # close connection
            self.event.clear()

if __name__ == '__main__':
    getter = DemoApp()
    getter.connect_to_device(dev_idx=0)
    getter.start_processing_stream()
