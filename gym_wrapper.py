from sim_pybullet import bullet_sim
import gym
from utils import *
import yaml
from gym import spaces
import pybullet as p

class bullet_gym(gym.Env):
    def __init__(self, robot_name: str):
        config = load_config(robot_name)
        sim_config = config["sim"]
    
        # set start positions and camera
        self.sim = bullet_sim(robot_name, config)
        self.video_recording = sim_config["video_recording"]
        if self.video_recording:
            self.sim.startVideoRecording()
        self.gui = self.sim.gui

        # observation space and action space
        jointUpperLimits = np.array(self.sim.jointUpperLimits)
        jointLowerLimits = np.array(self.sim.jointLowerLimits)
        jointMaxVelocites = np.array(self.sim.jointMaxVelocities)
        jointMinVelocities = -jointMaxVelocites
        self.jointMaxForces = jointMaxForces = np.array(self.sim.jointMaxForces)
        self.jointMinForces = jointMinForces = -jointMaxForces
        
        observation_space_upper = np.hstack([jointUpperLimits, jointMaxVelocites])
        observation_space_lower = np.hstack([jointLowerLimits, jointMinVelocities])
        action_space_upper = jointMaxForces 
        action_space_lower = jointMinForces

        self.observation_space = spaces.Box(
            observation_space_lower.astype(np.float32),
            observation_space_upper.astype(np.float32) 
        )

        self.action_space = spaces.Box(
            action_space_lower.astype(np.float32),
            action_space_upper.astype(np.float32) 
        )
        
    def step_tau(self, taus: np.array):
        # excute the torque
        self.sim.torqueControl(taus)        
        return self.sim.sim_step()

    def step_qqd(self, targetPositions: np.array, targetVelocities: np.array):
        self.sim.posVelControl(targetPositions, targetVelocities)
        return self.sim.sim_step()

    def step_qd(self, targetVelocities: np.array):
        self.sim.velControl(targetVelocities)
        return self.sim.sim_step()
    
    def reset(self):
        return self.sim.resetSim()

    def close(self):
        if self.video_recording:
            self.sim.stopVideoRecording() 
        p.disconnect(self.sim.physicsClient)

if __name__ == '__main__':
    import argparse
    
    parser = argparse.ArgumentParser(description='Simulator')                       
    parser.add_argument('--robot', type=str, default='furuta_pendulum')
    args = parser.parse_args()   
                                     
    # load yaml config
    robot_name = args.robot
    sim_gym = bullet_gym(robot_name)
    sim_config = load_config(robot_name)["sim"]
    taus = np.zeros_like(sim_gym.action_space.low)
    sim_gym.reset()
    horizon = sim_config["horizon"]
    for i in range(horizon):
        sim_gym.step_tau(taus)
