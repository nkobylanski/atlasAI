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

while True:
    distances = robot.read_ultrasonic_sensor()
    avoidance_correction = calculate_avoidance_correction(distances)
    
    img = read_frame(sim, visionSensor)
    img, detected, x, y, size = filter_color(img, colors[color_idx])
    
    cv.putText(img, f'Front: {min(distances[3:5]):.2f}m', (10, 20), 
               cv.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
    cv.putText(img, f'L: {distances[15]:.2f}m R: {distances[8]:.2f}m', (10, 40), 
               cv.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
    if avoidance_correction != 0:
        cv.putText(img, f'Avoiding: {avoidance_correction:.1f}', (10, 60), 
                   cv.FONT_HERSHEY_SIMPLEX, 0.4, (255, 165, 0), 1)
    
    cv.imshow('img', img)

    if detected:
        color_correction = 0.4 * (128 - x)
        total_correction = color_correction - avoidance_correction
        robot.move(150 - total_correction, 150 + total_correction)

        if size > 15000:
            color_idx = (color_idx + 1) % 4
            robot.rotate_left(50)
            time.sleep(1)
    else:
        robot.move(-50 - avoidance_correction, 50 + avoidance_correction)
    
    if cv.waitKey(1) & 0xFF == ord('e'):
        break

robot.stop()
cv.destroyAllWindows()
sim.stopSimulation()
