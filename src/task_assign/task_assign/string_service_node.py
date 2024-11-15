import rclpy
from rclpy.node import Node
from sent_message.srv import Message
from camera_info.srv import CameraInfo
import serial
import socket
import time
from datetime import datetime
import os
arduino = serial.Serial(port='/dev/ttyACM0', baudrate=9600, timeout=1)
esp32_ip = '192.168.0.130'
port = 8080
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

            
                

class StringServiceNode(Node):
    def __init__(self):
        super().__init__('string_service_node')
        self.service = self.create_service(Message, 'string_service', self.handle_service)
        self.sub_node=rclpy.create_node('sub_node')
        self.sub_client = self.sub_node.create_client(CameraInfo, 'camera_info')
        self.task = 0
    def handle_service(self, request, response):
        # Process the request and set the response
        self.sub_client.wait_for_service()
        sub_request = CameraInfo.Request()
        if request.request.lower() == 'task1':
            sub_request.path = self.create_folder()
            messages = [(100, 750, 100, 0,1), (400, 1050,100, 1,1), (700, 750, 100, 2,1), (400, 450, 100, 3,1)]
        elif request.request.lower() == 'calibrate':
            messages=[(0,0,0,0,0)]
            
            
            # Task 1: Start a specific action
        #     response.response = 'Task Done'
        # elif request.request.lower() == 'stop':
        #     # Task 2: Stop a specific action
        #     self.stop_task()
        #     response.response = 'Task stopped'
        # elif request.request.lower() == 'status':
        #     # Task 3: Provide status information
        #     response.response = self.get_status()
        # else:
        #     # Default response for unknown requests
        #     response.response = f'Unknown task: "{request.request}"'
        for msg in messages:
            x, y, z, esp_pos,task = msg
            self.send_message(x, y, z, esp_pos,task)
            sub_request.position = esp_pos
            if task != 0:
                future = self.sub_client.call_async(sub_request)
                rclpy.spin_until_future_complete(self.sub_node,future)
            
            self.get_logger().info(f'Finish: "{request.request}"')
        response.response = f'You sent: {request.request}'
        str_val="Done"
        byte_val=str_val.encode()
        sock.sendto(byte_val, (esp32_ip, port))
        return response  
    def send_message(self,x, y, z, esp_pos, task):
    # Format the message as a string
        message = f"({x},{y},{z},{task})\n"
        farmbot_reach=False
        esp32_reach=False#test
    # Send the message to the Arduino
        arduino.write(message.encode())
        str_val = str(esp_pos)
        byte_val = str_val.encode()
        sock.sendto(byte_val, (esp32_ip, port))

    # Wait for the "Done" message from Arduino
        while not farmbot_reach and not esp32_reach:
            # Read data from Arduino
            if not farmbot_reach:
                response = arduino.readline().decode().strip()
                if response == "Done":
                    farmbot_reach=True
                    #self.get_logger().info("Reach Position")
                    print("Farmbot Done")
            if not esp32_reach:
                data,server = sock.recvfrom(1024)
                if data.decode()=="Done":
                    print("Esp Done")
                    esp32_reach=True
        
        return
    def create_folder(self):
        base_dir = os.path.expanduser("~/Plant_image")
        # Generate folder name based on current date, hour, and minute
        folder_name = datetime.now().strftime("%Y%m%d_%H%M")
        full_path = os.path.join(base_dir, folder_name)
        print(full_path)
        if not os.path.exists(full_path):
            os.makedirs(full_path)
        return full_path
    def service_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Service response: "{response.response}"')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
def main(args=None):
    
    rclpy.init(args=args)
    node = StringServiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
