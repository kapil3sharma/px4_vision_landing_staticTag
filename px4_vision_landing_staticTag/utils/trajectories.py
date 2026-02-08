#!/usr/bin/env python3

import math

# ----------------------- Trajectories ------------------------
def square_trajectory(home_x, home_y, altitude, size=2.0):
    '''
        Generate a square trajectory around the home position.
    '''
    waypoints = [
                    [home_x + size/2, home_y + size/2, altitude],
                    [home_x + size/2, home_y - size/2, altitude],
                    [home_x - size/2, home_y - size/2, altitude],
                    [home_x - size/2, home_y + size/2, altitude],
                    [home_x + size/2, home_y + size/2, altitude],
                ]
    
    return waypoints

def spiral_trajectory(home_x, home_y, altitude, radius=0.5, flight_speed=0.3):
    '''
        Generate a spiral trajectory around the home position.
    '''
    theta = 0.0
    waypoints = []

    for _ in range(1000):
        x = home_x + radius * math.cos(theta)
        y = home_y + radius * math.sin(theta)

        waypoints.append([x, y, altitude])
        theta += 0.2 * flight_speed
        radius += 0.08  * flight_speed

    return waypoints

def circle_trajectory(home_x, home_y, altitude, radius=1.0, num_points=100):
    '''
        Generate a circular trajectory around the home position.
    '''
    waypoints = []

    for i in range(num_points):
        theta = 2 * math.pi * i / num_points
        
        x = home_x + radius * math.cos(theta)
        y = home_y + radius * math.sin(theta)

        waypoints.append([x, y, altitude])
    
    return waypoints