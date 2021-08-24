import pyrealsense2 as rs
import numpy as np
import cv2
import logging

ctx = rs.context()
devices = ctx.query_devices()
print(devices[0])
print(devices[1])

# Configure depth and color streams...
# ...from Camera 1
pipeline_1 = rs.pipeline()
config_1 = rs.config()
config_1.enable_device('908412110479')

# ...from Camera 2
pipeline_2 = rs.pipeline()
config_2 = rs.config()
config_2.enable_device('040322071708')

# Start streaming from both cameras
pipeline_1.start(config_1)
pipeline_2.start(config_2)

try:
    while True:

        # Camera 1
        # Wait for a coherent pair of frames: depth and color
        frames_1 = pipeline_1.wait_for_frames()
        frameset = frames_1.as_frameset()
        f1 = frameset.get_fisheye_frame(1).as_video_frame()
        f2 = frameset.get_fisheye_frame(2).as_video_frame()
        left_data = np.asanyarray(f2.get_data())
        fe_image = np.asanyarray(f1.get_data())
        print(fe_image.shape)

        # Camera 2
        # Wait for a coherent pair of frames: depth and color
        frames_2 = pipeline_2.wait_for_frames()
        color_frame = frames_2.get_color_frame()
        color_image = np.asanyarray(color_frame.get_data())
        color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)

        depth_frame_2 = frames_2.get_depth_frame()
        depth_image_2 = np.asanyarray(depth_frame_2.get_data())
        depth_colormap_2 = cv2.applyColorMap(cv2.convertScaleAbs(depth_image_2, alpha=0.1), cv2.COLORMAP_JET)
        print(depth_colormap_2.shape)
        # Show images from both cameras
        # cv2.imshow('RealSense', fe_image)
        # cv2.imshow('RealSense2', color_image)
        # cv2.imshow('3', depth_colormap_2)
        # cv2.waitKey(1)



finally:

    # Stop streaming
    pipeline_1.stop()
    pipeline_2.stop()
