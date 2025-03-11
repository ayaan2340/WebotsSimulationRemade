from controller import Supervisor, Keyboard
import numpy as np
import random
import math
import pickle

max_steering_angle = 0.5
max_speed = 30
road_length = 70
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
        
        self.left_front_wheel.setPosition(float('inf'))
        self.right_front_wheel.setPosition(float('inf'))
        self.left_front_wheel.setVelocity(0)
        self.right_front_wheel.setVelocity(0)
        self.keyboard.enable(self.time_step)
        self.keyboard = self.getKeyboard()
        
    def set_speed(self, speed):
        self.left_front_wheel.setVelocity(speed)
        self.right_front_wheel.setVelocity(speed)
    
    def set_steering(self, angle):
        self.left_steer.setPosition(angle);
        self.right_steer.setPosition(angle);
    
        
    def teleport_obj(self, obj, position):
        obj.getField("translation").setSFVec3f(position)
         
    def drive_manually(self):
        self.teleport_obj(self.car_node, [0,5,0])
        while self.step(self.time_step) != -1:
            keys = []
            key = self.keyboard.getKey()
            while key != -1:
                keys.append(key)
                key = self.keyboard.getKey()
            speed = 0.0
            steering_angle = 0.0
            if Keyboard.UP in keys:
                speed = max_speed
            elif Keyboard.DOWN in keys:
                speed = -max_speed
            
            if Keyboard.LEFT in keys:
                steering_angle = -max_steering_angle
            elif Keyboard.RIGHT in keys:
                steering_angle = max_steering_angle
            
            self.set_speed(speed)
            self.set_steering(steering_angle)
            
        
    def get_distance(self):
        car_pos = np.array(self.car_node.getPosition())
        end_pos = np.array(self.end.getPosition())
        distance = np.linalg.norm(car_pos - end_pos)
        return distance
        
    def build_roads(self):
        global road_width, road_length
        root = self.getRoot()
        children_field = root.getField('children')

        # Define road segment as a function for reusability
        def create_horizontal_road(x, y):
            x = x - 35
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
            y = y - 35
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
        children_field.importMFNodeFromString(-1, create_horizontal_road(0, -20))

        # Add vertical roads
        children_field.importMFNodeFromString(-1, create_vertical_road(-20, 0))  # 90-degree rotation
        children_field.importMFNodeFromString(-1, create_vertical_road(20, 0))
    
    def get_road_center_error(self):
        car_pos = self.car_node.getPosition()
        left_pos = 25
        right_pos = 15
        
        road_center_x = (left_pos + right_pos) / 2
        car_x = car_pos[1]
        
        return road_center_x - car_x  # Positive if car is right of center, negative if left
    

    def pid_control(self):
        error = self.get_road_center_error()
        self.integral += error
        derivative = error - self.prev_error
        
        steering_angle = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        steering_angle = max(-max_steering_angle, min(max_steering_angle, steering_angle))  # Clamp angle
        print(steering_angle)
        self.prev_error = error
        return steering_angle
        
    def drive_autonomously(self):
        while self.step(self.time_step) != -1:
            steering_angle = self.pid_control()
            self.set_speed(5)
            self.set_steering(steering_angle)

# Enter here exit cleanup code.
if __name__ == '__main__':
    controller = CarController()
    controller.build_roads()
    controller.drive_autonomously()