## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

###############################################
##      Open CV and Numpy integration        ##
###############################################

import pyrealsense2 as rs
import numpy as np
import cv2
import os
from datetime import datetime

def create_folder():
    base_dir = os.path.expanduser("~/Plant_image")
    # Generate folder name based on current date, hour, and minute
    folder_name = datetime.now().strftime("%Y%m%d_%H%M")
    full_path = os.path.join(base_dir, folder_name)
    print(full_path)
    if not os.path.exists(full_path):
        os.makedirs(full_path)
    return full_path

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)

# Start streaming
pipeline.start(config)

align_to = rs.stream.color
align = rs.align(align_to)
cnt =0

try:
    while True:

        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)

        depth_frame = frames.get_depth_frame()
        aligned_depth_frame = aligned_frames.get_depth_frame() 
        color_frame = frames.get_color_frame()
        aligned_color_frame = aligned_frames.get_color_frame()
        if not depth_frame or not color_frame and cnt<1000:
             cnt+=1
             continue
        break
        
    

    # Convert images to numpy arrays
    depth_image = np.asanyarray(aligned_depth_frame.get_data())
    color_image = np.asanyarray(aligned_color_frame.get_data())

    # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

    depth_colormap_dim = depth_colormap.shape
    color_colormap_dim = color_image.shape


    # Show images
    
    cv2.namedWindow('Color', cv2.WINDOW_AUTOSIZE)
    cv2.imshow('Color', color_image)

    cv2.namedWindow('Depth', cv2.WINDOW_AUTOSIZE)
    cv2.imshow('Depth', depth_image)
    cv2.waitKey(0)
    folder_path = create_folder()
    cv2.imwrite(folder_path+'/color.jpg',color_image)

    cv2.imwrite(folder_path+'/depth.png',depth_image)


finally:

    # Stop streaming
    pipeline.stop()
