"""config

Store the configuration of the A1 robot.

License: BSD 3-Clause License
Copyright (C) 2018-2019, New York University , Max Planck Gesellschaft
Copyright note valid unless otherwise stated in individual files.
All rights reserved.
"""

import numpy as np
from math import pi
from os.path import join, dirname
from os import environ
import pinocchio as se3
from pinocchio.utils import zero
from pinocchio.robot_wrapper import RobotWrapper
from robot_properties_atlas.utils import find_paths


class AtlasAbstract(object):
    """ Abstract class used for all Solo robots. """

    # PID gains
    kp = 5.0
    kd = 0.1
    ki = 0.0

    # The Kt constant of the motor [Nm/A]: tau = I * Kt.
    motor_torque_constant = 0.0

    # Control time period.
    control_period = 0.001
    dt = control_period

    # MaxCurrent = 12 # Ampers
    max_current = 0.0

    # Maximum torques.
    max_torque = 10000

    # Maximum control one can send, here the control is the current.
    max_control = max_torque

    # ctrl_manager_current_to_control_gain I am not sure what it does so 1.0.
    ctrl_manager_current_to_control_gain = 1.0

    max_qref = pi

    base_link_name = "pelvis"

    #TODO: Needs to be done
    #end_effector_names = ["HL_ANKLE", "HR_ANKLE", "FL_ANKLE", "FR_ANKLE"]

    @classmethod
    def buildRobotWrapper(cls):
        # Rebuild the robot wrapper instead of using the existing model to
        # also load the visuals.
        robot = RobotWrapper.BuildFromURDF(
            cls.urdf_path, cls.meshes_path, se3.JointModelFreeFlyer()
        )
        robot.model.rotorInertia[6:] = cls.motor_inertia
        robot.model.rotorGearRatio[6:] = cls.motor_gear_ration
        return robot

    def joint_name_in_single_string(self):
        joint_names = ""
        for name in self.robot_model.names[2:]:
            joint_names += name + " "
        return joint_names


class AtlasConfig(AtlasAbstract):
    robot_family = "boston_dynamics"
    robot_name = "atlas"

    paths = find_paths(robot_name)
    meshes_path = paths["package"]
    dgm_yaml_path = paths["dgm_yaml"]
    urdf_path = paths["urdf"]

    # The inertia of a single blmc_motor.
    motor_inertia = 0.0

    # The motor gear ratio.
    motor_gear_ration = 0.0

    # pinocchio model.
    pin_robot_wrapper = RobotWrapper.BuildFromURDF(
        urdf_path, meshes_path, se3.JointModelFreeFlyer()
    )
    pin_robot_wrapper.model.rotorInertia[6:] = motor_inertia
    pin_robot_wrapper.model.rotorGearRatio[6:] = motor_gear_ration
    pin_robot = pin_robot_wrapper

    robot_model = pin_robot_wrapper.model
    mass = np.sum([i.mass for i in robot_model.inertias])
    base_name = "pelvis"

    # End effectors informations
    shoulder_ids = []
    end_eff_ids = []
    shoulder_names = []
    end_effector_names = []

    #TODO: Needs to be redone for Atlas
    for leg in ["l_foot", "r_foot"]:
        end_eff_ids.append(robot_model.getFrameId(leg))
        end_effector_names.append(leg)

    # nb_ee = len(end_effector_names)
    # hl_index = robot_model.getFrameId("HL_ANKLE")
    # hr_index = robot_model.getFrameId("HR_ANKLE")
    # fl_index = robot_model.getFrameId("FL_ANKLE")
    # fr_index = robot_model.getFrameId("FR_ANKLE")

    # The number of motors, here they are the same as there are only revolute
    # joints.
    # nb_joints = robot_model.nv - 6

    # joint_names = [
    #     "FL_HAA",
    #     "FL_HFE",
    #     "FL_KFE",
    #     "FR_HAA",
    #     "FR_HFE",
    #     "FR_KFE",
    #     "HL_HAA",
    #     "HL_HFE",
    #     "HL_KFE",
    #     "HR_HAA",
    #     "HR_HFE",
    #     "HR_KFE",
    # ]

    # Mapping between the ctrl vector in the device and the urdf indexes.
    urdf_to_dgm = tuple(range(robot_model.nv))

    map_joint_name_to_id = {}
    map_joint_limits = {}
    for i, (name, lb, ub) in enumerate(
        zip(
            robot_model.names[1:],
            robot_model.lowerPositionLimit,
            robot_model.upperPositionLimit,
        )
    ):
        map_joint_name_to_id[name] = i
        map_joint_limits[i] = [float(lb), float(ub)]

    # Define the initial state.
    initial_configuration = (
        [0.0, 0.0, 0.863527, 0.0, 0.0, 0.0, 1.0]
        + [0.0, 0.0, 0.0]
        + [0.0, -1.2, 0.0, 0.0, 0.0, 0.0, 0.0]
        + [0.0, 1.2, 0.0, 0.0, 0.0, 0.0, 0.0]
        + [0.0, 0.0, -0.5, 1.0, -0.5, 0.0]
        + [0.0, 0.0, -0.5, 1.0, -0.5, 0.0]
    )
    initial_velocity = zero(robot_model.nv)

    q0 = zero(robot_model.nq)
    q0[:] = initial_configuration
    v0 = zero(robot_model.nv)
    a0 = zero(robot_model.nv)

    #what does this do?
    base_p_com = [0.0, 0.0, -0.02]
