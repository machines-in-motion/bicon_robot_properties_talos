"""Atlas wrapper

Atlas pybullet interface using pinocchio's convention.

License: BSD 3-Clause License
Copyright (C) 2018-2019, New York University , Max Planck Gesellschaft
Copyright note valid unless otherwise stated in individual files.
All rights reserved.
"""

import pybullet
from bullet_utils.wrapper import PinBulletWrapper
from robot_properties_atlas.config import AtlasConfig

dt = 1e-3


class AtlasRobot(PinBulletWrapper):
    def __init__(
        self,
        pos=None,
        orn=None,
        useFixedBase=False,
        init_sliders_pose=4
        * [
            0,
        ],
    ):

        # Load the robot
        if pos is None:
            pos = [0.0, 0, 0.95]
        if orn is None:
            orn = pybullet.getQuaternionFromEuler([0, 0, 0])

        pybullet.setAdditionalSearchPath(AtlasConfig.meshes_path)
        self.urdf_path = AtlasConfig.urdf_path
        self.robotId = pybullet.loadURDF(
            self.urdf_path,
            pos,
            orn,
            flags=pybullet.URDF_USE_INERTIA_FROM_FILE,
            useFixedBase=useFixedBase,
        )
        pybullet.getBasePositionAndOrientation(self.robotId)

        # Create the robot wrapper in pinocchio.
        self.pin_robot = AtlasConfig.buildRobotWrapper()

        # Query all the joints.
        num_joints = pybullet.getNumJoints(self.robotId)

        for ji in range(num_joints):
            pybullet.changeDynamics(
                self.robotId,
                ji,
                linearDamping=0.04,
                angularDamping=0.04,
                restitution=0.0,
                lateralFriction=0.5,
            )

        self.base_link_name = "pelvis"
        self.end_eff_ids = []
        self.end_effector_names = []
        controlled_joints = []

        # TODO: This parts need to be redone
        for name in self.pin_robot.model.names[2:]:
            controlled_joints.append(name)
        # for ee in eff_names:
            # controlled_joints += [ee + "_HAA", ee + "_HFE", ee + "_KFE"]
        #     self.end_effector_names.append(ee + "_FOOT")
        print(controlled_joints)
        self.joint_names = controlled_joints
        self.nb_ee = len(self.end_effector_names)

        # self.hl_index = self.pin_robot.model.getFrameId("HL_ANKLE")
        # self.hr_index = self.pin_robot.model.getFrameId("HR_ANKLE")
        # self.fl_index = self.pin_robot.model.getFrameId("FL_ANKLE")
        # self.fr_index = self.pin_robot.model.getFrameId("FR_ANKLE")

        # Creates the wrapper by calling the super.__init__.

        # 
        super(AtlasRobot, self).__init__(
            self.robotId,
            self.pin_robot,
            controlled_joints,
            self.end_effector_names,
        )

    def forward_robot(self, q=None, dq=None):
        if not q:
            q, dq = self.get_state()
        elif not dq:
            raise ValueError("Need to provide q and dq or non of them.")

        self.pin_robot.forwardKinematics(q, dq)
        self.pin_robot.computeJointJacobians(q)
        self.pin_robot.framesForwardKinematics(q)
        self.pin_robot.centroidalMomentum(q, dq)

    def get_slider_position(self, letter):
        if letter == "a":
            return pybullet.readUserDebugParameter(self.slider_a)
        if letter == "b":
            return pybullet.readUserDebugParameter(self.slider_b)
        if letter == "c":
            return pybullet.readUserDebugParameter(self.slider_c)
        if letter == "d":
            return pybullet.readUserDebugParameter(self.slider_d)
