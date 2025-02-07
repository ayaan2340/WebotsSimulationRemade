from controller import Supervisor, Keyboard

max_steering_angle = 0.5
max_speed = 30

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
    
    def get_position(self):
        return self.car_node.getPosition()
        
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


# Enter here exit cleanup code.
if __name__ == '__main__':
    controller = CarController()
    controller.drive_manually()