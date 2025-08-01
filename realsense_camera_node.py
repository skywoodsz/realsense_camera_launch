#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import pyrealsense2 as rs


class VisualHandlerNode:
    """ Wrapper for Intel RealSense depth camera with filters and ROS publishing """

    def __init__(self,
                 rs_resolution: tuple = (848, 480),  # width, height
                 rs_fps: int = 30,
                 forward_depth_image_topic="/camera/depth/image_rect_raw"):
        self.rs_resolution = rs_resolution
        self.rs_fps = rs_fps
        self.forward_depth_image_topic = forward_depth_image_topic

        self.bridge = CvBridge()
        self.depth_pub = rospy.Publisher(self.forward_depth_image_topic, Image, queue_size=10)

        self.start_pipeline()

    def start_pipeline(self):
        """Initialize RealSense pipeline and filters"""
        self.rs_pipeline = rs.pipeline()
        self.rs_config = rs.config()
        self.rs_config.enable_stream(
            rs.stream.depth,
            self.rs_resolution[0],
            self.rs_resolution[1],
            rs.format.z16,
            self.rs_fps
        )

        self.rs_profile = self.rs_pipeline.start(self.rs_config)

        # Alignment (depth only, but if RGB needed: rs.stream.color)
        self.rs_align = rs.align(rs.stream.depth)

        ## ===== Filters =====
        # Decimation (downsample to reduce noise)
        self.rs_decimation_filter = rs.decimation_filter()
        self.rs_decimation_filter.set_option(rs.option.filter_magnitude, 2)  # 1=none, 2~3 moderate

        # Spatial (spatial smoothing & hole filling)
        self.rs_spatial_filter = rs.spatial_filter()
        self.rs_spatial_filter.set_option(rs.option.filter_magnitude, 3)
        self.rs_spatial_filter.set_option(rs.option.filter_smooth_alpha, 0.3)
        self.rs_spatial_filter.set_option(rs.option.filter_smooth_delta, 20)
        self.rs_spatial_filter.set_option(rs.option.holes_fill, 1)  # 0=off,1~5 increasing fill

        # Temporal (temporal smoothing)
        self.rs_temporal_filter = rs.temporal_filter()
        self.rs_temporal_filter.set_option(rs.option.filter_smooth_alpha, 0.3)
        self.rs_temporal_filter.set_option(rs.option.filter_smooth_delta, 20)

        # Hole filling (final step)
        self.rs_hole_filling_filter = rs.hole_filling_filter()
        self.rs_hole_filling_filter.set_option(rs.option.holes_fill, 1)  # 1=farest,2=nearest

        ## Filter sequence (recommended)
        self.rs_filters = [
            self.rs_decimation_filter,
            self.rs_spatial_filter,
            self.rs_temporal_filter,
            self.rs_hole_filling_filter
        ]

        rospy.loginfo("RealSense pipeline started with filters applied.")

    def get_depth_frame(self):
        """Get and filter the latest depth frame"""
        # frames = self.rs_pipeline.wait_for_frames()
        frames = self.rs_pipeline.poll_for_frames()
        depth_frame = frames.get_depth_frame()

        if not depth_frame:
            rospy.logwarn("No depth frame received")
            return None

        for rs_filter in self.rs_filters:
            depth_frame = rs_filter.process(depth_frame)

        return depth_frame

    def publish_depth_image(self):
        """Publish filtered depth image to ROS topic"""
        depth_frame = self.get_depth_frame()
        if depth_frame is None:
            return

        depth_image = np.asanyarray(depth_frame.get_data())
        depth_image_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding="16UC1")
        self.depth_pub.publish(depth_image_msg)


if __name__ == "__main__":
    rospy.init_node('realsense_camera', anonymous=True)

    visual_node = VisualHandlerNode()

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        try:
            visual_node.publish_depth_image()
            rate.sleep()
        except rospy.ROSInterruptException:
            rospy.logerr("Something went wrong in realsense camera node!")
            break
