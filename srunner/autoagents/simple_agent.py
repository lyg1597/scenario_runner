#!/usr/bin/env python

# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module provides a dummy agent to control the ego vehicle
"""

from __future__ import print_function

import carla

import numpy as np

from srunner.scenariomanager.carla_data_provider import *

# from leaderboard.autoagents.autonomous_agent import AutonomousAgent, Track
from srunner.autoagents.autonomous_agent import AutonomousAgent

def get_entry_point():
    return 'SimpleAgent'

class PerceptionModule():
    def __init__(self, carla_world, role_name, radius=20):
        self.sensing_radius = radius # default ?????
        self.world = carla_world
        self.vehicle = None
        self.role_name = role_name
        
        self.find_ego_vehicle()
        
    # find ego vehicle
    # reference: 
    def find_ego_vehicle(self):
        if self.world is None:
            self.world = CarlaDataProvider.get_world()
            return
        for actor in self.world.get_actors():
            if actor.attributes.get('role_name') == self.role_name:
            # if 'vehicle' in actor.type_id:
                self.vehicle = actor
                break

    def get_curr_state(self):
        vehicle_state = self.vehicle.get_transform()
        vehicle_velocity = self.vehicle.get_velocity()
        return [vehicle_state, vehicle_velocity]

    # return all the obstacles within the sensing radius of the vehicle
    def get_all_obstacles_within_range(self):
        # get every actor on stage
        if self.vehicle == None:
            self.world = CarlaDataProvider.get_world()
            self.find_ego_vehicle()
            # rospy.loginfo("No ego vehicle.")
            return []
        vehicle = self.vehicle
        vehicle_loc = vehicle.get_location()
        all_actors = self.world.get_actors()
        radius = self.sensing_radius
        filtered_obstacles = []
        for actor in all_actors:
            # get actor's location
            cur_loc = actor.get_location()
            # determine whether actor is within the radius
            if vehicle_loc.distance(cur_loc) <= radius:
                # we need to throw out actors such as camera
                # types we need: vehicle, walkers, Traffic signs and traffic lights
                # reference: https://github.com/carla-simulator/carla/blob/master/PythonAPI/carla/scene_layout.py

                if 'vehicle' in actor.type_id and actor.id != vehicle.id:
                    filtered_obstacles.append(actor)
                elif 'pedestrian' in actor.type_id:
                    filtered_obstacles.append(actor)
                elif 'static.prop' in actor.type_id:
                    filtered_obstacles.append(actor)
        return filtered_obstacles

    def set_radius(self, new_radius):
        self.sensing_radius = new_radius
    
    def get_radius(self):
        return self.sensing_radius
    
    # get set of waypoints separated by parameter -- distance -- along the lane
    # reference: https://github.com/carla-simulator/carla/issues/1254
    def get_lane_way_point(self, distance=0.5):
        if self.vehicle == None:
            self.find_ego_vehicle()
            # rospy.loginfo("No ego vehicle.")
            return
        vehicle = self.vehicle
        carla_map = self.world.get_map()
        vehicle_location = vehicle.get_location()
        # get a nearest waypoint
        cur_waypoint = carla_map.get_waypoint(vehicle_location)
        # return list of waypoints from cur_waypoint to 10 meters ahead
        wp_to_end = cur_waypoint.next_until_lane_end(distance)
        if len(wp_to_end) > 20:
            wp_to_end = wp_to_end[0:20]
        return wp_to_end

class VehicleDecision():
    def __init__(self, carla_world, role_name):

        self.vehicle_state = 'straight'
        self.vehicle_lane = 'middle'
        self.start_lanechange = False
        self.target_lane_id = None
        self.counter = 0

        self.waypoint = None
        self.target_x = None 
        self.target_y = None
        self.change_lane = False
        self.change_lane_wp_idx = 0
        self.detect_dist = 15
        self.speed = 20

        self.vehicle = None
        self.role_name = role_name
        self.world = carla_world


        self.find_ego_vehicle()
        
    def find_ego_vehicle(self):
        if self.world is None:
            self.world = CarlaDataProvider.get_world()
            return
        for actor in self.world.get_actors():
            if actor.attributes.get('role_name') == self.role_name:
            # if 'vehicle' in actor.type_id:
                self.vehicle = actor
                break

    def get_ref_state(self, currState, obstacleList):
        if self.vehicle is None:
            self.find_ego_vehicle()
            return [currState.location.x, currState.location.y, -1]

        curr_x = currState.location.x
        curr_y = currState.location.y
        
        obs_front = False
        obs_left = False
        obs_right = False
        dist = 100
        if obstacleList:
            for obs in obstacleList:
                obs_center = obs.get_transform().location
                dy = obs_center.y - curr_y
                dx = obs_center.x - curr_x
                yaw = currState.rotation.yaw*np.pi/180
                rx = np.cos(-yaw) * dx - np.sin(-yaw) * dy
                ry = np.cos(-yaw) * dy + np.sin(-yaw) * dx

                dist = np.sqrt(dy*dy + dx*dx)
                psi = np.arctan2(ry,rx)
                # print("detected object is at {} away and {} radians".format(front_dist, psi))
                if psi < 0.2 and psi > -0.2 and dist < 15:
                    obs_front = True
                if psi > 0.2:
                    if rx < 0 and dist<10:
                        obs_right = True
                    elif rx > 0 and dist<10:
                        obs_right = True
                if psi < -0.2:
                    if rx < 0 and dist < 10:
                        obs_left = True 
                    elif rx > 0 and dist<10:
                        obs_left = True
        
        vehicle_location = self.vehicle.get_transform().location
        waypoints = self.get_lane_way_point(vehicle_location)
        if len(waypoints) > 6:
            waypoint = waypoints[5]
        else:
            waypoint = waypoints[-1]

        if obs_front or obs_left or obs_right:
            return [curr_x, curr_y, -1]
        else:
            return [waypoint.transform.location.x, 
                waypoint.transform.location.y, 
                20
            ]

    def get_lane_way_point(self, vehicle_location, distance=0.5):
        carla_map = self.world.get_map()
        # get a nearest waypoint
        cur_waypoint = carla_map.get_waypoint(vehicle_location)
        # return list of waypoints from cur_waypoint to 10 meters ahead
        wp_to_end = cur_waypoint.next_until_lane_end(distance)
        if len(wp_to_end) > 20:
            wp_to_end = wp_to_end[0:20]
        return wp_to_end

class VehicleController():
    def __init__(self, role_name='ego_vehicle'):
        # Publisher to publish the control input to the vehicle model
        pass

    def stop(self):
        throttle = 0
        brake = 1
        steer = 0
        return throttle, brake, steer

    def execute(self, currentPose, targetPose):
        """
            This function takes the current state of the vehicle and 
            the target state to compute low-level control input to the vehicle
            Inputs: 
                currentPose: ModelState, the current state of vehicle
                targetPose: The desired state of the vehicle
        """

        currentEuler = currentPose[1] * np.pi/180
        curr_x = currentPose[0][0]
        curr_y = currentPose[0][1]


        target_x = targetPose[0]
        target_y = targetPose[1]
        target_v = targetPose[2]
        
        k_s = 1
        k_ds = 1
        k_n = 0.1
        k_theta = 0.5

        # compute errors
        dx = target_x - curr_x
        dy = target_y - curr_y
        xError = (target_x - curr_x) * np.cos(currentEuler) + (target_y - curr_y) * np.sin(currentEuler)
        yError = -(target_x - curr_x) * np.sin(currentEuler) + (target_y - curr_y) * np.cos(currentEuler)
        curr_v = np.sqrt(currentPose[2][0]**2 + currentPose[2][1]**2)
        vError = target_v - curr_v
        thetaError = np.pi/2 - currentEuler
        
        v = xError*k_s + vError*k_ds
        delta = k_n*yError + k_theta*thetaError

        if delta > 1:
            delta = 1
        elif delta < -1:
            delta = -1

        if v>1:
            v = 1
        elif v<-1:
            v = -1

        # print(target_x, curr_x, xError, target_y, curr_y, yError, v, delta)

        # Checking if the vehicle need to stop
        if target_v < 0:
            return 0, 1, delta
        elif v > 0:
            #Send computed control input to vehicle
            # newAckermannCmd = AckermannDrive()
            # newAckermannCmd.speed = v
            # newAckermannCmd.steering_angle = delta
            # self.controlPub.publish(newAckermannCmd)
            return v, 0, delta
        else:
            return 0, v, delta

class SimpleAgent(AutonomousAgent):

    """
    Dummy autonomous agent to control the ego vehicle
    """
    def setup(self, path_to_conf_file):
        """
        Setup the agent parameters
        """
        self.world = CarlaDataProvider.get_world()
        self.role_name = "hero"
        self.vehicle_perception = PerceptionModule(self.world, self.role_name)
        self.vehicle_decision = VehicleDecision(self.world, self.role_name)
        self.vehicle_control = VehicleController(self.role_name)
        # self.track = Track.MAP

    def sensors(self):
        """
        Define the sensor suite required by the agent

        :return: a list containing the required sensors in the following format:

        [
            {'type': 'sensor.camera.rgb', 'x': 0.7, 'y': -0.4, 'z': 1.60, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
                      'width': 300, 'height': 200, 'fov': 100, 'id': 'Left'},

            {'type': 'sensor.camera.rgb', 'x': 0.7, 'y': 0.4, 'z': 1.60, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
                      'width': 300, 'height': 200, 'fov': 100, 'id': 'Right'},

            {'type': 'sensor.lidar.ray_cast', 'x': 0.7, 'y': 0.0, 'z': 1.60, 'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0,
             'id': 'LIDAR'}


        """

        sensors = [
            {'type': 'sensor.camera.rgb', 'x': 0.7, 'y': -0.4, 'z': 1.60, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
             'width': 300, 'height': 200, 'fov': 100, 'id': 'Left'},
        ]

        return sensors

    def run_step(self, input_data, timestamp):
        """
        Execute one step of navigation.
        """
        # print("=====================>")
        # for key, val in input_data.items():
        #     if hasattr(val[1], 'shape'):
        #         shape = val[1].shape
        #         print("[{} -- {:06d}] with shape {}".format(key, val[0], shape))
        #     else:
        #         print("[{} -- {:06d}] ".format(key, val[0]))
        # print("<=====================")

        # DO SOMETHING SMART
        obstacle_list = self.vehicle_perception.get_all_obstacles_within_range()
        # lane_waypoints = self.vehicle_perception.get_lane_way_point()
        curr_state = self.vehicle_perception.get_curr_state()
        target_pose = self.vehicle_decision.get_ref_state(curr_state[0], obstacle_list)
        curr_x = curr_state[0].location.x
        curr_y = curr_state[0].location.y
        curr_theta = curr_state[0].rotation.yaw
        curr_vx = curr_state[1].x
        curr_vy = curr_state[1].y
        throttle, brake, steer = self.vehicle_control.execute(\
            [[curr_x, curr_y], curr_theta, [curr_vx, curr_vy]], target_pose)

        # RETURN CONTROL
        control = carla.VehicleControl()
        control.steer = steer
        control.throttle = throttle
        control.brake = brake
        control.hand_brake = False
        # print(throttle, steer, brake)
        return control
