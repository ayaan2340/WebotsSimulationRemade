from controller import Supervisor, Keyboard
import numpy as np
import random
import math
import pickle

ALPHA = 0.1
GAMMA = 0.9
EPSILON = 0.3

max_steering_angle = 0.5
max_speed = 30
ACTIONS = [(-0.5, 30), (0.5, 30), (0, 30)]
Q_TABLE = {}
class CarController(Supervisor):
    def __init__(self):
        super().__init__()
        self.time_step = int(self.getBasicTimeStep())
        self.car_node = self.getFromDef("car")
        self.end = self.getFromDef("redEnd")
        if self.car_node is None:
            raise ValueError("Robot node with DEF name 'car' not found.")
        self.initialize_devices()

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
            
    # Start of RL Code
    def get_state(self):
        car_pos = np.array(self.car_node.getPosition())
        end_pos = np.array(self.end.getPosition())
        distance = np.linalg.norm(car_pos - end_pos)
        return tuple(np.round(car_pos, 0))
        
    def get_distance(self):
        car_pos = np.array(self.car_node.getPosition())
        end_pos = np.array(self.end.getPosition())
        distance = np.linalg.norm(car_pos - end_pos)
        return distance
        
    def choose_action(self, state):
        if random.uniform(0, 1) < EPSILON or state not in Q_TABLE:
            return random.choice(ACTIONS)
        return max(Q_TABLE[state], key=Q_TABLE[state].get)
        
    def set_action(self, action):
        steering, speed = action
        self.set_speed(speed)
        self.set_steering(steering)
        
    def get_reward(self, prev_distance, new_distance):

        reward = 0 if abs(new_distance - prev_distance) > 2 else prev_distance - new_distance
        return reward

    def update_q_table(self, state, action, reward, next_state):
        if state not in Q_TABLE:
            Q_TABLE[state] = {a: 0 for a in ACTIONS}
        if next_state not in Q_TABLE:
            Q_TABLE[next_state] = {a: 0 for a in ACTIONS}
        best_next_action = max(Q_TABLE[next_state], key=Q_TABLE[next_state].get)
        Q_TABLE[state][action] += ALPHA * (reward + GAMMA * Q_TABLE[next_state][best_next_action] - Q_TABLE[state][action])
    
    def train(self, episodes=100000):
        global ALPHA, EPSILON
        count = 0
        for episode in range(episodes):
            self.car_node.resetPhysics()
            self.car_node.getField("translation").setSFVec3f([0, 0, 0])
            self.car_node.getField("rotation").setSFRotation([0, 0, 1, 1.57])
            self.set_speed(0)
            self.set_steering(0)

            state = self.get_state()
            distance = self.get_distance()
            start_time = self.getTime()
            while self.step(self.time_step) != -1:
                if self.getTime() - start_time > 30:
                    break
                action = self.choose_action(state)
                self.set_action(action)
                self.step(self.time_step)
                next_state = self.get_state()
                next_distance = self.get_distance()
                reward = self.get_reward(distance, next_distance)
                self.update_q_table(state, action, reward, next_state)
                state = next_state
                distance = next_distance
                if distance > 35:
                    break
                if distance < 2:
                    print(f"worked {count}")
                    # ALPHA = ALPHA * (1.2**(-count*0.1))
                    # EPSILON = EPSILON * (1.2**(-count*0.1))
                    print(ALPHA)
                    with open(f"./saved_models/model{count}", "wb") as f:
                        pickle.dump(Q_TABLE, f)
                    count += 1
                    break  # Stop when close to the target
                    
    def replay(self):
        self.car_node.getField("translation").setSFVec3f([0, 5, 0])
        with open("/Users/ayaan/Documents/Simulation/controllers/car_controller/saved_models/model57", "rb") as f:
            Q_TABLE = pickle.load(f)
        for i in range(100):
            self.car_node.resetPhysics()
            self.car_node.getField("translation").setSFVec3f([0, 0, 0])
            self.car_node.getField("rotation").setSFRotation([0, 0, 1, 1.57])
            self.set_speed(0)
            self.set_steering(0)
    
            state = self.get_state()
            distance = self.get_distance()
            start_time = self.getTime()
            state = self.get_state()
            while self.step(self.time_step) != -1:
                if self.getTime() - start_time > 30:
                   break
                action = max(Q_TABLE.get(state, {}), key=Q_TABLE.get(state, {}).get, default=random.choice(ACTIONS))
                self.set_action(action)
                self.step(self.time_step)
                distance = self.get_distance()
                state = self.get_state()
                if distance > 35:
                    break
                if distance < 2.0:
                    break

# Enter here exit cleanup code.
if __name__ == '__main__':
    controller = CarController()
    controller.train()