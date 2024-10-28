import rclpy
from rclpy.node import Node
from sent_message.srv import Message
import serial
import socket
import time
arduino = serial.Serial(port='/dev/ttyACM0', baudrate=9600, timeout=1)
esp32_ip = '192.168.1.130'
port = 8080
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
def send_message(x, y, z, esp_pos, task):
    # Format the message as a string
        message = f"({x},{y},{z},{task})\n"
        farmbot_reach=False
        esp32_reach=False
    
    # Send the message to the Arduino
        arduino.write(message.encode())
        sock.sendto(esp_pos, (esp32_ip, port))

    # Wait for the "Done" message from Arduino
        while not farmbot_reach and not esp32_reach:
            # Read data from Arduino
            if not farmbot_reach:
                response = arduino.readline().decode().strip()
                if response == "Done":
                    farmbot_reach=True
                    print("Arduino: Done")
            if not esp32_reach:
                data,server = sock.recvfrom(1024)
                if data.decode()=="Done":
                    print("Esp Done")
                    esp32_reach=True
        return
            
                

class StringServiceNode(Node):
    def __init__(self):
        super().__init__('string_service_node')
        self.service = self.create_service(Message, 'string_service', self.handle_service)
        self.task = 0
    def handle_service(self, request, response):
        self.get_logger().info(f'Received request: "{request.request}"')
        # Process the request and set the response
        if request.request.lower() == 'task1':
            self.task = 1
            # Task 1: Start a specific action
            self.do_task()
            response.response = 'Task started'
        elif request.request.lower() == 'stop':
            # Task 2: Stop a specific action
            self.stop_task()
            response.response = 'Task stopped'
        elif request.request.lower() == 'status':
            # Task 3: Provide status information
            response.response = self.get_status()
        else:
            # Default response for unknown requests
            response.response = f'Unknown task: "{request.request}"'

        return response
    def do_task(self):
        if(self.task==1):
            messages = [(100, 750, 300, 0,1), (400, 1050, 300, 1,1), (700, 750, 300, 2,1), (400, 450, 300, 3,1)]
        elif(self.task==2):
            messages = [(100, 750, 300, 1), (400, 1050, 300, 1), (700, 750, 300, 1), (400, 450, 300, 1)]
        for msg in messages:
            x, y, z, esp_pos,task = msg
            send_message(x, y, z, esp_pos,task)
    

 
        
        ##response.response = f'You sent: {request.request}'
        #return response

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
