import os
import pybullet as p
from PIL import Image
import wandb        
import imageio
import numpy as np
import yaml
import matplotlib.pyplot as plt
import pinocchio as pin
from copy import copy
from loguru import logger
from scipy.signal import savgol_filter

def find_path(name, path):
    for root, dirs, files in os.walk(path):
        if name in files:
            return os.path.join(root, name)
    raise Exception(f"Can't find {name} in directory {path}!")

def save_png(frameId, img_dir):
    width, height, rgbPixels, depthPixels, _ = p.getCameraImage(width=800, height=600)
    imgArray = Image.fromarray(rgbPixels, 'RGBA')
    imgName = f"./{img_dir}/{frameId}.png" 
    imgArray.save(imgName)
    return imgName

def save_gif(img_dir, video_dir, robot_name):
    imgNames = os.listdir(img_dir)
    f = lambda x: int(x[:-4])
    imgNames = sorted(imgNames, key=f)

    imgIoBuffer = []
    for imgName in imgNames:
        imgName = f"./{img_dir}/{imgName}"
        imgIoBuffer.append(imageio.imread(imgName))
    imageio.mimsave(f"./{video_dir}/{robot_name}.gif", imgIoBuffer)

    # delete all image files
    os.popen(f'rm {img_dir}/*.png')

def upload_video(robot_name, video_dir, fps, video_type="gif"):
    # upload to weights and biases
    wandb.init(
        project="sysid_pin", 
        entity="junning", 
        name=robot_name
    )
    wandb.log(
            {"video": wandb.Video(f"./{video_dir}/{robot_name}.{video_type}", fps=fps, format=f"{video_type}")}
            )

def upload_img(robot_name, img_name, img_type="png"):
    # upload to weights and biases
    wandb.init(
        project="sysid_pin", 
        entity="junning", 
        name=robot_name
    )
    wandb.log(
            {"image": wandb.Image(img_name)}
            )

# --------------------------
# util functions for pybulet
def retrieve_state(states):
    qs = []
    qds = []
    reactionForces = []
    appliedTorques = []
    for state in states:
        qs.append(state[0])
        qds.append(state[1])
        reactionForces.append(state[2])
        appliedTorques.append(state[3])
    return np.array(qs), np.array(qds), np.array(reactionForces), np.array(appliedTorques)

def retrieve_info(objId, jointIdxs):
    jointLowerLimits = []
    jointUpperLimits = []
    jointMaxForces = []
    jointMaxVelocities = []
    jointAxiss = []
    jointDampings = []
    jointTypes = []
    for jointIndex in jointIdxs:
        info = p.getJointInfo(objId, jointIndex)
        jointIndex, jointName, jointType, qIndex, uIndex, flags, \
        jointDamping, jointFriction, jointLowerLimit, jointUpperLimit, \
        jointMaxForce, jointMaxVelocity, linkName, jointAxis, \
        parentFramePos, parentFrameOrn, parentIndex = info    
        
        jointTypes.append(jointType)
        jointLowerLimits.append(jointLowerLimit) 
        jointUpperLimits.append(jointUpperLimit) 
        jointMaxForces.append(jointMaxForce)
        jointMaxVelocities.append(jointMaxVelocity)
        jointAxiss.append(jointAxis)
        jointDampings.append(jointDamping)
    return jointTypes, jointLowerLimits, jointUpperLimits, jointMaxForces, jointMaxVelocities, jointAxiss, jointDampings

# --------------------------

def load_config(file_name):
    config_file = f"{file_name}.yml"
    config_file = find_path(config_file, "./configs")
    with open(f"{config_file}") as file:
        config = yaml.safe_load(file)
    return config

