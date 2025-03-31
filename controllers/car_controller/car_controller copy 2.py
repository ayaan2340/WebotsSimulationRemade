from controller import Supervisor, Keyboard
import numpy as np
import cv2
import random

max_steering_angle = 0.5
max_speed = 30
road_length = 300
road_width = 10


class CarController(Supervisor):
    def __init__(self):
        super().__init__()
        self.time_step = int(self.getBasicTimeStep())
        self.car_node = self.getFromDef("car")
        self.end = self.getFromDef("redEnd")
        if self.car_node is None:
            raise ValueError("Robot node with DEF name 'car' not found.")
        self.initialize_devices()
        
        self.kp = 0.05  # Proportional gain
        self.ki = 0  # Integral gain
        self.kd = 8   # Derivative gain
        
        self.prev_error = 0
        self.integral = 0

    def initialize_devices(self):
        self.left_steer = self.getDevice("left_steer")
        self.right_steer = self.getDevice("right_steer")
        self.left_front_wheel = self.getDevice("left_front_wheel")
        self.right_front_wheel = self.getDevice("right_front_wheel") 
      
        self.camera = self.getDevice("camera")
        if not self.camera:
            raise ValueError("Camera device not found.")
        self.camera.enable(self.time_step)
        self.left_front_wheel.setPosition(float('inf'))
        self.right_front_wheel.setPosition(float('inf'))
        self.left_front_wheel.setVelocity(0)
        self.right_front_wheel.setVelocity(0)
        self.keyboard = self.getKeyboard()
        self.keyboard.enable(self.time_step)
        
    def set_speed(self, speed):
        self.left_front_wheel.setVelocity(speed)
        self.right_front_wheel.setVelocity(speed)
    
    def set_steering(self, angle):
        self.left_steer.setPosition(angle)
        self.right_steer.setPosition(angle)
    
    def process_camera_image(self):
        image = self.camera.getImage()
        if image is None:
            return None
        
        width = self.camera.getWidth()
        height = self.camera.getHeight()
        image = np.frombuffer(image, np.uint8).reshape((height, width, 4))
        
        # Convert to grayscale for edge detection
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGRA2GRAY)
        
        # Apply Gaussian blur to reduce noise
        blurred = cv2.GaussianBlur(gray_image, (5, 5), 0)
        
        # Edge detection
        edges = cv2.Canny(blurred, 50, 150)
        return edges
    
    def get_road_center_error(self):
        edges = self.process_camera_image()
        if edges is None:
            return 10
    
        height, width = edges.shape
    
        # Detect lines using Hough Transform
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 30, minLineLength=30, maxLineGap=30)
        if lines is None:
            return 10
    
        left_lines = []
        right_lines = []
    
        for line in lines:
            x1, y1, x2, y2 = line[0]
            dy = y2 - y1
            dx = x2 - x1 + 1e-5  # Avoid division by zero
            slope = dy / dx
    
            # Filter lines based on slope and position
            if abs(slope) < 0.1 or abs(slope) > 200:  # Ignore nearly horizontal lines
                continue
    
            if slope < 0 and x1 < 3 * width / 4:  # Left lane lines
                left_lines.append(line)
            elif slope > 0 and x2 > width / 4:  # Right lane lines
                right_lines.append(line)
    
        # Calculate lane positions
        left_x = self.calculate_lane_position(left_lines, height)
        right_x = self.calculate_lane_position(right_lines, height)
    
        # If both lanes detected
        if left_x is not None and right_x is not None:
            lane_center = (left_x + right_x) / 2
        elif left_x is not None:  # Only left lane detected
            lane_center = left_x + road_width / 2 * width / road_width
        elif right_x is not None:  # Only right lane detected
            lane_center = right_x - road_width / 2 * width / road_width
        else:  # No lanes detected
            return 10
    
        car_position = width / 2
        error = (lane_center - car_position) / width
        return error

    def build_roads(self):
        global road_width, road_length
        root = self.getRoot()
        children_field = root.getField('children')

        # Define road segment as a function for reusability
        def create_horizontal_road(x, y):
            x = x - road_length / 2
            return f"""
            StraightRoadSegment {{
              translation {x} {y} 0.1
              rotation 0 0 1 0
              rightBorder FALSE
              leftBorder FALSE
              length {road_length}
              width {road_width}
              numberOfLanes 1
            }}
            """
        def create_vertical_road(x, y):
            y = y - road_length / 2
            return f"""
            StraightRoadSegment {{
              translation {x} {y} 0.1
              rotation 0 0 1 1.57
              rightBorder FALSE
              leftBorder FALSE
              length {road_length}
              width {road_width}
              numberOfLanes 1
            }}
            """

        # def create_intersection(x, y):
            # return f"""
            # RoadIntersection {{
              # translation {x} {y} 0
            # }}
            # """

        # Add horizontal roads
        children_field.importMFNodeFromString(-1, create_horizontal_road(0, 20))
        # children_field.importMFNodeFromString(-1, create_horizontal_road(0, -20))

        # Add vertical roads
        # children_field.importMFNodeFromString(-1, create_vertical_road(-20, 0))  # 90-degree rotation
        # children_field.importMFNodeFromString(-1, create_vertical_road(20, 0))
    

    def pid_control(self):
        error = self.get_road_center_error()
        self.integral += error
        derivative = error - self.prev_error
        
        steering_angle = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        steering_angle = max(-max_steering_angle, min(max_steering_angle, steering_angle))  # Clamp angle
        self.prev_error = error
        print(derivative)
        return steering_angle
        
    def drive_autonomously(self):
        self.car_node.getField("rotation").setSFRotation([0, 0, 1, random.random() * 6])
        while self.step(self.time_step) != -1:
            steering_angle = self.pid_control()
            self.set_speed(30)
            self.set_steering(steering_angle)


# Enter here exit cleanup code.
if __name__ == '__main__':
    controller = CarController()
    controller.build_roads()
    controller.drive_autonomously()