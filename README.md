# Pybullet Gym Wrapper
## Installation
### Operating Systems:
The wrapper has been tested on
1. Ubuntu 20.04
### Conda Installation:
Download anaconda or miniconda from https://docs.conda.io/en/latest/miniconda.html and install it follow the online instructions 
### Create an virtual environment with anaconda:
```
conda create --name pybullet python=3.8 --file requirements.txt
```
## Active the virtua environment and run the script
```
conda active pybullet
```
```
python gym_wrapper.py
```
You can set the robot you want with a flag ```--robot```, the available choices include:

1. furuta_pendulum
2. wam_4dof
3. iiwas
4. cartpole_short_downward
5. a1
