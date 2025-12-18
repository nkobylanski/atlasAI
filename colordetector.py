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
