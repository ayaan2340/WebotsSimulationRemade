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
        
        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGRA2GRAY)
        
        # Apply threshold to isolate road
        _, binary = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
        
        # Apply bird's eye view transformation
        src_points = np.float32([[0, height], [width, height], 
                                [0, height//2], [width, height//2]])
        dst_points = np.float32([[0, height], [width, height],
                                [0, 0], [width, 0]])
        
        matrix = cv2.getPerspectiveTransform(src_points, dst_points)
        warped = cv2.warpPerspective(binary, matrix, (width, height))
        cv2.imwrite("warped.png", warped)
        return warped

    def get_road_center_error(self):
        warped = self.process_camera_image()
        
        if warped is None:
            return 0
            
        height, width = warped.shape
        
        # Define region of interest for lane detection
        roi_height = height // 3
        roi = warped[height-roi_height:height, :]
        
        # Find lane pixels using histogram
        histogram = np.sum(roi, axis=0)
        midpoint = width // 2
        
        # Find peaks in left and right halves
        left_x = np.argmax(histogram[:midpoint])
        right_x = np.argmax(histogram[midpoint:]) + midpoint
        
        # If no clear peaks are found, use fallback values
        if histogram[left_x] == 0:
            left_x = width * 0.25
        if histogram[right_x] == 0:
            right_x = width * 0.75
        
        # Calculate road center
        road_center = (left_x + right_x) / 2
        car_position = width / 2
        
        # Calculate normalized error (-1 to 1)
        error = (road_center - car_position) / (width / 2)
        
        # Apply smoothing to error
        error = np.clip(error, -1, 1)
        return error

    def detect_curve(self):
        warped = self.process_camera_image()
        if warped is None:
            return 0
            
        height, width = warped.shape
        
        # Analyze multiple horizontal slices
        slices = 5
        slice_height = height // slices
        centers = []
        
        for i in range(slices):
            y_start = height - (i + 1) * slice_height
            y_end = height - i * slice_height
            slice_img = warped[y_start:y_end, :]
            
            # Find road boundaries in slice
            histogram = np.sum(slice_img, axis=0)
            midpoint = width // 2
            left_x = np.argmax(histogram[:midpoint])
            right_x = np.argmax(histogram[midpoint:]) + midpoint
            
            if histogram[left_x] > 0 and histogram[right_x] > 0:
                center = (left_x + right_x) / 2
                centers.append(center)
        
        if len(centers) > 1:
            # Calculate curve direction from center points
            curve = (centers[-1] - centers[0]) / width
            return curve
        return 0

    def pid_control(self):
        error = self.get_road_center_error()
        curve = self.detect_curve()
        print(error)
        # Combine lateral error and curve detection
        combined_error = error + 0.5 * curve
        
        self.integral += combined_error
        derivative = combined_error - self.prev_error
        
        # Adjust PID gains based on curve detection
        if abs(curve) > 0.2:
            kp = self.kp * 1.5  # Increase proportional gain for curves
            kd = self.kd * 0.8  # Reduce derivative gain for smoother curve handling
        else:
            kp = self.kp
            kd = self.kd
        
        steering_angle = (kp * combined_error) + \
                        (self.ki * self.integral) + \
                        (kd * derivative)
        
        # Clamp steering angle
        steering_angle = max(-max_steering_angle, min(max_steering_angle, steering_angle))
        self.prev_error = combined_error
        
        return steering_angle
        
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