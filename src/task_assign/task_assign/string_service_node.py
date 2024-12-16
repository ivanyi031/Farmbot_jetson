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
            #(400,750)
            messages = [(30, 610 ,280, 0,1), (397, 932,280, 1,1), (716, 574, 280, 2,1), (352, 256, 280, 3,1),(340, 614, 193,4,1)]
            #messages = [(76, 615, 500, 0,1), (468, 941,500, 1,1), (794, 550, 500, 2,1), (403, 223, 500, 3,1),(345, 615, 300,4,1)]
        elif request.request.lower() == 'calibrate':
            messages=[(0,0,0,0,-1)]
        elif request.request.lower() == 'tracking':
            self.tracking_motor()
            response.response = "done tracking"
            return response
        else:
            response.response = "wrong message"
            return response

            
            
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
            if task != -1:
                future = self.sub_client.call_async(sub_request)
                rclpy.spin_until_future_complete(self.sub_node,future)
            
            self.get_logger().info(f'Finish: "{request.request}"')
        response.response = f'You sent: {request.request}'
        if(task != -1):
            str_val="Done"
            byte_val=str_val.encode()
            sock.sendto(byte_val, (esp32_ip, port))
        return response  
    def send_message(self,x, y, z, esp_pos, task):
    # Format the message as a string
        arduino.readall()
        message = f"({x},{y},{z},{esp_pos},{task})\n"
        farmbot_reach=False
        esp32_reach=False#test
    # Send the message to the Arduino
        arduino.write(message.encode())
        str_val = str(esp_pos)
        byte_val = str_val.encode()
        sock.sendto(byte_val, (esp32_ip, port))

        while not farmbot_reach: #and not esp32_reach
            # Read data from Arduino
                response = arduino.readline().decode().strip()
                if(len(response)):
                    print(response)
                    if response == "Done":
                        farmbot_reach=True
                        #self.get_logger().info("Reach Position")
                        print("Farmbot Done")
        if task !=-1:
            data,server = sock.recvfrom(1024)
            if data.decode()=="Done":
                    print("Esp Done")
                    esp32_reach=True
    # Wait for the "Done" message from Arduino
        
    def tracking_motor(self):
        arduino.readall()
        message = "(0,0,0,0,0)\n"
        arduino.write(message.encode())
        #time.sleep(5)
        while(1):
            try:
                response = arduino.readline().decode().strip()
                if(len(response)):
                    print(response)
            except KeyboardInterrupt:
                arduino.write("Done\n".encode())
                print("tracking over")
                break;       
        return
    def create_folder(self):
        base_dir = os.path.expanduser("~/Camera_calibration")#("~/Plant_image")
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
