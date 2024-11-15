import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Import message type for the topic
from camera_info.srv import CameraInfo
import pyrealsense2 as rs
import numpy as np
import cv2
import time
import os

class RealSenseNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        # Initialize RealSense pipeline and config
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        # Service to start or stop capture
        self.service = self.create_service(CameraInfo, 'camera_info', self.handle_service_request)
        self.get_logger().info("Service 'start_stop_capture' is ready.")
        # Initialize variables for RealSense device
        self.pipeline_started = False

    def handle_service_request(self, request, response):
        # Handle service to start or stop capture
        pos = request.position
        folder_path=request.path  
        if not self.pipeline_started:
            self.start_stream(pos,folder_path)
            response.camera_response="Success capturing image"
        else:
            self.stop_stream()

        self.get_logger().info(response.camera_response)
        return response
    def start_stream(self,pos,folder_path):
        # Start the RealSense stream
        if not self.pipeline_started:
            self.configure_stream()
            self.pipeline.start(self.config)
            self.pipeline_started = True
            self.get_logger().info("RealSense camera started streaming.")

            # Run the capture loop in a separate method
            self.capture_frames(pos,folder_path)

    def stop_stream(self):
        # Stop the RealSense stream
        if self.pipeline_started:
            self.pipeline.stop()
            self.pipeline_started = False
            self.get_logger().info("RealSense camera stopped streaming.")

    def configure_stream(self):
        # Configure depth and color streams
        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = self.config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        device_product_line = str(device.get_info(rs.camera_info.product_line))

        found_rgb = False
        for s in device.sensors:
            if s.get_info(rs.camera_info.name) == 'RGB Camera':
                found_rgb = True
                break
        if not found_rgb:
            self.get_logger().error("The demo requires a Depth camera with a Color sensor.")
            exit(0)

        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)

    def capture_frames(self,pos,folder_path):
        align_to = rs.stream.color
        align = rs.align(align_to)
        

        try:
            start_time = time.time()
            while time.time() - start_time < 5:  # Capture for 5 seconds or until stopped
                frames = self.pipeline.wait_for_frames()
                aligned_frames = align.process(frames)

                depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()
                
                if not depth_frame or not color_frame:
                    continue

                # Convert images to numpy arrays
                depth_image = np.asanyarray(depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())

                # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
                depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

                # Display images
                cv2.namedWindow('Color', cv2.WINDOW_AUTOSIZE)
                cv2.namedWindow('Depth', cv2.WINDOW_AUTOSIZE)
                cv2.imshow('Color', color_image)
                cv2.imshow('Depth', depth_colormap)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            print(folder_path+f"/{pos}.jpg")
            cv2.imwrite(folder_path+f"/{pos}.jpg", color_image)
            cv2.imwrite(folder_path+f"/{pos}_depth.png", depth_image)

        finally:
            # Ensure the pipeline stops
            self.stop_stream()
            cv2.destroyAllWindows()
def main(args=None):
    rclpy.init(args=args)
    node = RealSenseNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_stream()  # Ensure the stream stops before shutting down
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
