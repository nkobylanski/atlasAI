from coppeliasim_zmqremoteapi_client import *
import time
import numpy as np
import cv2 as cv

class PioneerP3DX_Robot:
    
    def __init__(self, client):
        self.sim = client.require('sim')
        self.motorLeft = self.sim.getObject('./leftMotor')
        self.motorRight = self.sim.getObject('./rightMotor')
        
        self.sensorHandles = [self.sim.getObject(f'./ultrasonicSensor[{i}]') for i in range(16)]

    def move(self, vLeft, vRight):
        self.sim.setJointTargetVelocity(self.motorLeft, vLeft * np.pi / 180)
        self.sim.setJointTargetVelocity(self.motorRight, vRight * np.pi / 180)

    def rotate_left(self, speed):
        self.move(-speed, speed)

    def stop(self):
        self.move(0, 0)

    def read_ultrasonic_sensor(self):
        distances = []
        for handle in self.sensorHandles:
            state, dist, *_ = self.sim.readProximitySensor(handle)
            distances.append(dist if dist > 0 else 2.0)
        return np.array(distances)


def calculate_avoidance_correction(distances, threshold=0.4):
    """Calculate gentle steering correction to avoid obstacles"""
    front_center = min(distances[3], distances[4])
    front_left = min(distances[0:3])
    front_right = min(distances[5:8])
    side_left = distances[15]
    side_right = distances[8]
    
    correction = 0
    
    if front_center < threshold:
        correction += 30 * (1 - front_center / threshold) * (1 if front_left < front_right else -1)
    
    if front_left < threshold:
        correction += 20 * (1 - front_left / threshold)
    
    if front_right < threshold:
        correction -= 20 * (1 - front_right / threshold)
    
    if side_left < threshold * 0.7:
        correction += 15 * (1 - side_left / (threshold * 0.7))
    
    if side_right < threshold * 0.7:
        correction -= 15 * (1 - side_right / (threshold * 0.7))
    
    return np.clip(correction, -50, 50)

def read_frame(sim, sensor):
    buf, res = sim.getVisionSensorImg(sensor)
    img = np.frombuffer(buf, dtype=np.uint8).reshape(*res, 3)
    img = cv.cvtColor(img, cv.COLOR_RGB2BGR)
    return cv.flip(img, 1)

def filter_color(img, color):
    color_ranges = {
        'red': ([0, 70, 50], [10, 255, 255]),
        'green': ([36, 25, 25], [70, 255, 255]),
        'blue': ([105, 70, 50], [130, 255, 255]),
        'yellow': ([25, 50, 70], [35, 255, 255])
    }
    
    lower, upper = color_ranges[color]
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    mask = cv.inRange(hsv, np.array(lower), np.array(upper))
    
    contours, _ = cv.findContours(mask, cv.RETR_LIST, cv.CHAIN_APPROX_NONE)
    
    for contour in contours:
        if cv.contourArea(contour) > 100:
            area = cv.contourArea(contour)
            x, y, w, h = cv.boundingRect(contour)
            cx, cy = x + w // 2, y + h // 2
            cv.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 1)
            cv.putText(img, f'{color} ({cx}, {cy})', (x, y - 10), 
                      cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            return img, True, cx, cy, area
    
    return img, False, 0, 0, 0


client = RemoteAPIClient()
sim = client.require('sim')
sim.startSimulation()

robot = PioneerP3DX_Robot(client)
visionSensor = sim.getObject('./visionSensor')
colors = ['red', 'green', 'blue', 'yellow']
color_idx = 0
