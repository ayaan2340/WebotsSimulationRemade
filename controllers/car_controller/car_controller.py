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
        self.kd = 4   # Derivative gain
        
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
        
        # Convert to grayscale and apply thresholding
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGRA2GRAY)
        blurred = cv2.GaussianBlur(gray_image, (5, 5), 0)
        _, thresh = cv2.threshold(blurred, 127, 255, cv2.THRESH_BINARY)
        
        return thresh

    def get_road_center_error(self):
        image = self.process_camera_image()
        if image is None:
            return 0
            
        height, width = image.shape
        
        # Define parameters for sliding windows
        n_windows = 9
        window_height = height // n_windows
        margin = width // 10  # Width of the window
        
        # Start at the bottom of the image
        y_current = height - window_height // 2
        
        # Find initial position of lanes using histogram
        bottom_half = image[height//2:, :]
        histogram = np.sum(bottom_half, axis=0)
        midpoint = width // 2
        
        # Find the peak of the left and right halves of the histogram
        left_x_base = np.argmax(histogram[:midpoint])
        right_x_base = np.argmax(histogram[midpoint:]) + midpoint
        
        # Current positions to be updated for each window
        left_x_current = left_x_base
        right_x_current = right_x_base
        
        left_lane_pts = []
        right_lane_pts = []
        
        # Step through the windows one by one
        for window in range(n_windows):
            # Identify window boundaries
            win_y_low = height - (window + 1) * window_height
            win_y_high = height - window * window_height
            
            # Find nonzero pixels within the window
            good_left_inds = []
            good_right_inds = []
            
            if left_x_current != 0:  # If we found left lane
                win_x_left_low = max(0, left_x_current - margin)
                win_x_left_high = min(width, left_x_current + margin)
                img_window = image[win_y_low:win_y_high, win_x_left_low:win_x_left_high]
                if img_window.any():
                    good_left_inds = np.argmax(np.sum(img_window, axis=0)) + win_x_left_low
                    left_lane_pts.append((good_left_inds, (win_y_low + win_y_high) // 2))
                    left_x_current = good_left_inds
                    
            if right_x_current != 0:  # If we found right lane
                win_x_right_low = max(0, right_x_current - margin)
                win_x_right_high = min(width, right_x_current + margin)
                img_window = image[win_y_low:win_y_high, win_x_right_low:win_x_right_high]
                if img_window.any():
                    good_right_inds = np.argmax(np.sum(img_window, axis=0)) + win_x_right_low
                    right_lane_pts.append((good_right_inds, (win_y_low + win_y_high) // 2))
                    right_x_current = good_right_inds
                    
        bottom_third = -len(left_lane_pts)//3
        # Calculate road center from detected lane points
        if len(left_lane_pts) > 0 and len(right_lane_pts) > 0:
            # Use the average of bottom third of detected points
            left_x = np.mean([pt[0] for pt in left_lane_pts[bottom_third:]])
            right_x = np.mean([pt[0] for pt in right_lane_pts[bottom_third:]])
            road_center = (left_x + right_x) / 2
        elif len(left_lane_pts) > 0:
            left_x = np.mean([pt[0] for pt in left_lane_pts[bottom_third:]])
            road_center = left_x + width//4
        elif len(right_lane_pts) > 0:
            right_x = np.mean([pt[0] for pt in right_lane_pts[bottom_third:]])
            road_center = right_x - width//4
        else:
            return 0
        
        # Calculate error relative to the bottom center of the image
        car_position = width / 2
        error = (road_center - car_position) / (width / 2)  # Normalize to [-1, 1]
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
        
        # Reduce integral windup
        self.integral = np.clip(self.integral + error, -1.0, 1.0)
        derivative = error - self.prev_error
        
        # Adjust PID gains based on error magnitude
        if abs(error) > 0.5:
            kp = self.kp * 1.5  # Increase proportional gain for large errors
            kd = self.kd * 0.5  # Reduce derivative term to prevent oscillation
        else:
            kp = self.kp
            kd = self.kd
        
        steering_angle = (kp * error) + (self.ki * self.integral) + (kd * derivative)
        steering_angle = max(-max_steering_angle, min(max_steering_angle, steering_angle))
        self.prev_error = error
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