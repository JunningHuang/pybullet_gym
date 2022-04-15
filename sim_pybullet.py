import pybullet as p
import time
import pybullet_data
import numpy as np
import os
from utils import *
from loguru import logger

class bullet_sim(object):
    def __init__(self, robot_name, config):
        # create two directories: on for storing temporary images to generate video
        # another for store videos 
        data_path = config["data_path"]
        self.img_dir = img_dir = data_path["images"]
        self.video_dir = video_dir = data_path["videos"]
        self.robot_data_dir = robot_data_dir = data_path["robot_data"]
        
        cur_dirs = os.listdir()
        if img_dir in cur_dirs:
            imgs = os.listdir(img_dir)
            if len(imgs) != 0:
                os.popen(f'rm {img_dir}/*.png')
            logger.info(f"Directory for storing images already exists")
        else:
            os.mkdir(img_dir)
        if video_dir in cur_dirs:
            logger.info(f"Directory for storing videos already exists")
        else:
            os.mkdir(video_dir)

        # retrieve flags and configurations
        startPos, startOrientation = config["startPos"], config["startOrientation"] 
        self.gui = config["gui"] 
        self.fps = config["fps"] 
        self.robot_name = robot_name 
        startOrientation = p.getQuaternionFromEuler(startOrientation)

        # locate model file
        urdf_filename = robot_name + ".urdf"
        urdf_filename = find_path(urdf_filename, f"./{robot_data_dir}") 

        # connect to pybullet server
        if self.gui:
            gui_flag = p.GUI # graphical version
        else:
            gui_flag = p.DIRECT # non-graphical version
        self.physicsClient = p.connect(gui_flag)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        planeId = p.loadURDF("plane.urdf")
        self.objId = p.loadURDF(urdf_filename, startPos, startOrientation, useFixedBase=1)

        # remove fixed joints
        tot_njoints = p.getNumJoints(self.objId)
        tot_jointIdxs = [i for i in range(tot_njoints)]
        jointTypes, jointLowerLimits, jointUpperLimits, jointMaxForces, \
            jointMaxVelocities, jointAxiss, jointDampings = retrieve_info(self.objId, tot_jointIdxs)
        self.jointIdxs = []
        self.jointTypes, self.jointLowerLimits, self.jointUpperLimits, self.jointMaxForces, \
            self.jointMaxVelocities, self.jointAxiss, self.jointDampings = [], [], [], [], [], [], []
        for i in range(len(jointTypes)):
            if jointTypes[i] == p.JOINT_FIXED:
                continue
            else:
                self.jointIdxs.append(i)
                self.jointLowerLimits.append(jointLowerLimits[i])
                self.jointUpperLimits.append(jointUpperLimits[i])
                self.jointMaxForces.append(jointMaxForces[i])
                self.jointMaxVelocities.append(jointMaxVelocities[i])
                self.jointAxiss.append(jointAxiss[i])
                self.jointDampings.append(jointDampings[i])
        self.njoints = len(self.jointIdxs)
        logger.info(f"Successfully init {robot_name}, number of joints: {self.njoints}")
        logger.info(f"JointLowerLimits: {self.jointLowerLimits}")
        logger.info(f"JointUpperLimits: {self.jointUpperLimits}")
        logger.info(f"JointMaxVelocities: {self.jointMaxVelocities}")
        logger.info(f"JointMaxForces: {self.jointMaxForces}")
        logger.info(f"JointDampings: {self.jointDampings}")

        # set initial joint position and camera
        init_state_config = config["init_state"]
        self.initJointPos, self.initJointVel = init_state_config["jointPos"], init_state_config["jointVel"]
        self.initState = np.hstack([self.initJointPos, self.initJointVel])
        self.resetSim()

        cam_config = config["cam_config"]
        cam_dist, cam_yaw, cam_pitch, cam_target = cam_config["cam_dist"], cam_config["cam_yaw"], \
                                                   cam_config["cam_pitch"], cam_config["cam_target"]
        self.setCamera(cam_dist, cam_yaw, cam_pitch, cam_target)

        ## disable default motors
        #self._disable_default_motor()
    
    def _disable_default_motor(self):
        maxForce = 0 
        mode = p.VELOCITY_CONTROL
        for jointId in self.jointIdxs:
            p.setJointMotorControl2(self.objId, jointIndex=jointId, controlMode=mode, force=maxForce)

    def setCamera(self, cam_dist:float, cam_yaw:float, cam_pitch:float, cam_target:np.array=np.array([0., 0., 0.])):
        p.resetDebugVisualizerCamera(cam_dist, cam_yaw, cam_pitch, cam_target)

    def setJointState(self, jointPos, jointVel):
        assert len(jointPos) == self.njoints and len(jointVel) == self.njoints, "joint positions and velocities should have the same dimension with the number of joints"
        for i in range(len(jointPos)): 
            pos = jointPos[i]
            vel = jointVel[i]
            jointId = self.jointIdxs[i]
            p.resetJointState(self.objId, jointId, pos, vel)

    def resetSim(self):
        self.setJointState(self.initJointPos, self.initJointVel)
        return self.initState

    def posVelControl(self, targetPoss:np.array, targetVels:np.array, posGains:np.array=None, velGains:np.array=None):
        """
        Param:
            controlMode:      int
            targetPositions:  list of float
            targetVelocities: list of float
            forces:           list of float
            positionGains:    list of float
            velocityGains:    list of float
        """
        assert len(targetPoss) == len(targetVels) == len(self.jointIdxs), "target positions and velocities should have the same dimension with the number of moveable joints"
        if posGains is None and velGains is None:
            p.setJointMotorControlArray(bodyUniqueId=self.objId, jointIndices=self.jointIdxs, controlMode=p.POSITION_CONTROL, \
                                        targetPositions=targetPoss, targetVelocities=targetVels)
        else:
            p.setJointMotorControlArray(bodyUniqueId=self.objId, jointIndices=self.jointIdxs, controlMode=p.POSITION_CONTROL, \
                                        targetPositions=targetPoss, targetVelocities=targetVels, positionGains=posGains, velocityGains=velGains)

    def velControl(self, targetVels:np.array, velGains=None):
        assert len(targetVels) == len(self.jointIdxs), "target velocities should has the same dimension with the number of oveable joints"
        if velGains is None:
            p.setJointMotorControlArray(bodyUniqueId=self.objId, jointIndices=self.jointIdxs, controlMode=p.VELOCITY_CONTROL, \
                                        targetVelocities=targetVels)
        else:
            p.setJointMotorControlArray(bodyUniqueId=self.objId, jointIndices=self.jointIdxs, controlMode=p.VELOCITY_CONTROL, \
                                        targetVelocities=targetVels, velocityGains=velGains)
    
    def torqueControl(self, forces:np.array):
        p.setJointMotorControlArray(bodyUniqueId=self.objId, jointIndices=self.jointIdxs, controlMode=p.TORQUE_CONTROL, \
                                    forces=forces)

    def sim_step(self):
        sim_start_time = time.time()

        p.stepSimulation()

        # TODO: should return sth, state, action, rew etc...
        jointStates = p.getJointStates(self.objId, self.jointIdxs)
        q, qd, reactionForces, appliedTorques = retrieve_state(jointStates)

        self.wait_fps(sim_start_time)
        qqd = np.hstack([q, qd]) 
        return qqd, appliedTorques 

    def wait_fps(self, sim_start_time:float):
        deltat = 1./self.fps
        time_duration = time.time() - sim_start_time

        ## use time.sleep here to guarantee we meet the given frequency
        if deltat > time_duration:
            elapse_time = deltat - time_duration
            time.sleep(elapse_time)
        else:
            raise Exception(f"Current frame rate: {time_duration}, but require a {deltat} frame rate for simulation, you should decrease the require fps")

    def startVideoRecording(self):
        self.loggingID = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, f"{self.video_dir}/{self.robot_name}.mp4") 
        
    def stopVideoRecording(self, upload:bool=False):
        p.stopStateLogging(self.loggingID)
        if upload:
            upload_video(self.robot_name, self.video_dir, self.fps, video_type="mp4")

if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description='Simulator')
    parser.add_argument('--robot', type=str, default='furuta_pendulum')
    args = parser.parse_args()

    # load yaml config
    robot_name = args.robot
    config = load_config(robot_name)
    
    # set start positions and camera
    sim = bullet_sim(robot_name, config)

    # rollout
    sim_config = config["sim"]
    horizon, video_recording = sim_config["horizon"], sim_config["video_recording"]
    if video_recording:
        sim.startVideoRecording()
    for i in range(horizon):
        sim.sim_step()
    if video_recording:
        sim.stopVideoRecording()
