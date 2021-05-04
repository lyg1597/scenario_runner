#!/usr/bin/env python

# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

# modified by Yangge Li

"""
Blank Scenario:

Blank Scenario Template for Creating Scenarios
"""

import random
import numpy as np

import py_trees

import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import *
from srunner.scenariomanager.scenarioatomics.atomic_criteria import *
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import *

from srunner.scenariomanager.timer import TimeOut
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.tools.scenario_helper import get_waypoint_in_distance

from importlib import import_module

type_map = {
    "behaviors": "srunner.scenariomanager.scenarioatomics.atomic_behaviors",
    "criteria": "srunner.scenariomanager.scenarioatomics.atomic_criteria",
    "trigger_conditions": "srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions"
}

class BlankScenario(BasicScenario):

    """
    This class holds everything required for a simple "Follow a leading vehicle"
    scenario involving two vehicles.  (Traffic Scenario 2)

    This is a single ego vehicle scenario
    """

    timeout = 120            # Timeout of scenario in seconds

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=60):
        """
        Setup all relevant parameters and create scenario

        If randomize is True, the scenario parameters are randomized
        """

        self._map = CarlaDataProvider.get_map()
        self._reference_waypoint = self._map.get_waypoint(config.trigger_points[0].location)
        self._other_actor_max_brake = 1.0
        self._other_actor_stop_in_front_intersection = 20
        self._other_actor_transform = None
        # Timeout of scenario in seconds
        self.timeout = timeout
        self.scenario_config = config.scenario_dict

        super(BlankScenario, self).__init__("BlankScenario",
                                                   ego_vehicles,
                                                   config,
                                                   world,
                                                   debug_mode,
                                                   criteria_enable=criteria_enable)

        if randomize:
            self._ego_other_distance_start = random.randint(4, 8)

            # Example code how to randomize start location
            # distance = random.randint(20, 80)
            # new_location, _ = get_location_in_distance(self.ego_vehicles[0], distance)
            # waypoint = CarlaDataProvider.get_map().get_waypoint(new_location)
            # waypoint.transform.location.z += 39
            # self.other_actors[0].set_transform(waypoint.transform)

    def _initialize_actors(self, config):
        """
        Custom initialization
        """

        # first_vehicle_waypoint, _ = get_waypoint_in_distance(self._reference_waypoint, self._first_vehicle_location)
        # self._other_actor_transform = carla.Transform(
        #     carla.Location(first_vehicle_waypoint.transform.location.x,
        #                    first_vehicle_waypoint.transform.location.y + 30,
        #                    first_vehicle_waypoint.transform.location.z + 0.1),
        #     first_vehicle_waypoint.transform.rotation)
        # first_vehicle_transform = carla.Transform(
        #     carla.Location(self._other_actor_transform.location.x ,
        #                    self._other_actor_transform.location.y + 30,
        #                    self._other_actor_transform.location.z - 500),
        #     self._other_actor_transform.rotation)
        # first_vehicle = CarlaDataProvider.request_new_actor('vehicle.tesla.model3',
        #                                                     first_vehicle_transform)
        # first_vehicle.set_simulate_physics(enabled=True)
        # self.other_actors.append(first_vehicle)
        npc_config_list = self.scenario_config['npc_vehicle']
        for npc_config in npc_config_list:
            if 'global_transform' in npc_config:
                pass
            elif 'relative_transform' in npc_config:
                relative_transform = npc_config['relative_transform']

                trigger_point, _ = get_waypoint_in_distance(self._reference_waypoint, 0)
                trigger_x = trigger_point.transform.location.x
                trigger_y = trigger_point.transform.location.y
                trigger_z = trigger_point.transform.location.z
                trigger_yaw = trigger_point.transform.rotation.yaw
                offset_x = np.cos(trigger_yaw*np.pi/180)*relative_transform['x'] - np.sin(trigger_yaw*np.pi/180)*relative_transform['y']
                offset_y = np.sin(trigger_yaw*np.pi/180)*relative_transform['x'] + np.cos(trigger_yaw*np.pi/180)*relative_transform['y']
                offset_z = relative_transform['z']
                offset_yaw = relative_transform['yaw']

                vehicle_transform = carla.Transform(
                    carla.Location(
                        trigger_x + offset_x,
                        trigger_y + offset_y,
                        trigger_z + offset_z
                    ),
                    carla.Rotation(
                        0,
                        trigger_yaw + offset_yaw,
                        0
                    )
                )
                vehicle = CarlaDataProvider.request_new_actor('vehicle.tesla.model3', vehicle_transform)
                self.other_actors.append(vehicle)

    def _create_behavior(self):
        """
        The scenario defined after is a "follow leading vehicle" scenario. After
        invoking this scenario, it will wait for the user controlled vehicle to
        enter the start region, then make the other actor to drive until reaching
        the next intersection. Finally, the user-controlled vehicle has to be close
        enough to the other actor to end the scenario.
        If this does not happen within 60 seconds, a timeout stops the scenario
        """

        # Sequential actions for NPCs
        npc_behaviors = py_trees.composites.Sequence("Sequential Behaviors for npcs")
        for action in self.scenario_config["behavior"]:
            try:
                module = import_module(type_map[action["type"]])
            except Exception as e:
                print(e)

            try:
                action_definition = getattr(module, action["name"])
            except Exception as e:
                print(e)

            action_params = action['params']

            if "actors" in action:
                action_params["actor_list"] = []
                for actor_idx in action["actors"]:
                    if actor_idx == "ego":
                        action_params["actor_list"].append(self.ego_vehicles[0])
                    else:
                        action_params["actor_list"].append(self.other_actors[actor_idx])

            action_instance = action_definition(action_params)
            npc_behaviors.add_child(action_instance)

        # construct scenario
        # driving_forward_and_change_lane.add_child(start_driving)
        
        # end condition
        endcondition = py_trees.composites.Parallel("Waiting for end position",
                                                    policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)
        endcondition_part = StandStill(self.ego_vehicles[0], name="StandStill", duration=15)
        endcondition.add_child(endcondition_part)

        # Build behavior tree
        sequence = py_trees.composites.Sequence("Sequence Behavior")
        # sequence.add_child(start_transform)
        # sequence.add_child(driving_forward_and_change_lane)
        sequence.add_child(npc_behaviors)
        sequence.add_child(endcondition)
        sequence.add_child(ActorDestroy(self.other_actors[0]))

        return sequence

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criteria = []

        collision_criterion = CollisionTest(self.ego_vehicles[0])

        criteria.append(collision_criterion)

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()
