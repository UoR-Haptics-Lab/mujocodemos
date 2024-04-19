"""
Run this file with -m and then, in matlab, view with
>> load data
>> figure;plot(ts,qpos,ts,qvel,ts,joint_pos,ts,joint_vel);shg

Based on Pranav Bhounsule (PB) 3_control_pendulum.py
https://www.youtube.com/watch?v=SJQZKZsvRRE
MuJoCoPy Lec 7: Position/Velocity/Torque Servo
"""

# from mujoco.glfw import glfw
from scipy.io import savemat # needed for matlab style matrices
import argparse
import mujoco as mj
import mujoco.viewer
import numpy as np
#import os
#import sys
import time

# ---- Global and default variables ----------
matlabdatafile="data.mat"

# log via simandcollect 
q_pos = np.empty((0,1),float) # position state size=1
q_vel = np.empty((0,1),float) # velocity state size=1
ts = np.empty((0,1),float)    # time 
sensdata0 = np.empty((0,1),float)
sensdata1 = np.empty((0,1),float)

# MuJoCo data structures
#model = mj.MjModel.from_xml_path(xml_path)  # MuJoCo model
#data = mj.MjData(model)                # MuJoCo data
#cam = mj.MjvCamera()                        # Abstract camera
#opt = mj.MjvOption()                        # visualization options


# ---- Process command line arguments -------------------------------------------------------

parser = argparse.ArgumentParser(description='Load a mujoco file, run a simulation and save data for matlab.')

parser.add_argument('-v', '--viewer',action='store_true',help='use the mjuoco viewer')      # viewer
parser.add_argument('-x', '--mjmodel',type=str,default='simplecontrolonelink.xml',help='Will use named file as the mjmodel.If not given uses a default file')      # model file
parser.add_argument('-r', '--runtime',type=float,default=5,help='set the runtime in seconds')  # runtime
parser.add_argument('-l', '--log',action='store_true', help='log the data into csv files')     # csv log
parser.add_argument('-m', '--dotmat',action='store_true', help='log the data into a matlab matrix')  # matlab logs may be faster
parser.add_argument('-s', '--numsensors',type=int, default=-1, help='t.b.d. save the sensor data. This number must be less than or equal to the number of sensors.') # t.b.d.  
args=parser.parse_args()

# ---- Command line done -------------------------------------------------------


# ---- Functionse -----------------------------

def torquecontroller(model, data):
    """
    Based on PB 3_control_pendulum
    """
    refpos=4 # radians
    Kp=-10
    Kd=-1
    model.actuator_gainprm[0, 0] = 1
    data.ctrl[0] = Kp * (data.sensordata[0] - refpos) + Kd * (data.sensordata[1] - 0.0)

def pdcontroller(model, data):
        kp = 10.0
        model.actuator_gainprm[1, 0] = kp
        model.actuator_biasprm[1, 1] = -kp
        data.ctrl[1] = -0.5

        kv = 1.0
        model.actuator_gainprm[2, 0] = kv
        model.actuator_biasprm[2, 2] = -kv
        data.ctrl[2] = 0.0

# called each time around the loop
def simandcollect(dum):
    global ts
    global q_pos
    global q_vel
    global sensdata0
    global sensdata1
    mj.mj_step(model, data)
    q_pos=np.append(q_pos,[data.qpos],axis=0)
    q_vel=np.append(q_vel,[data.qvel],axis=0)
    ts=np.append(ts,data.time)
    sensdata0=np.append(sensdata0,[data.sensor(0).data],axis=0)
    sensdata1=np.append(sensdata1,[data.sensor(1).data],axis=0)

        
# ---- Main program starts here -----------------------------------------
        
# Load the mujoco model
model = mj.MjModel.from_xml_path(args.mjmodel) # Model loaded from xml, that can inturn call on URDF etc
data = mj.MjData(model)                # MuJoCo data

#set the controller
mj.set_mjcb_control(torquecontroller)


if(args.viewer): # viewer is the -m option
  with mj.viewer.launch_passive(model, data) as viewer:
    start = time.time()
    while viewer.is_running() and (time.time() - start) < args.runtime: # 
      simandcollect(0)
      viewer.sync()
      elapsedtime=time.time()-start
      if data.time > elapsedtime : # comment out next two lines to not attempt real time 
          time.sleep(data.time-elapsedtime)
    print(f'model time=%0.2f, actual time=%0.2f'%(data.time, time.time()-start))
    #      time_until_next_step = model.opt.timestep - (time.time() - step_start)
else: # no viewer
    while data.time < args.runtime:
        simandcollect(0)



# ---- save data to simpendata.mat files --
if args.dotmat >0 :
    mdict={'ts': ts, 'qpos': q_pos, 'qvel':q_vel, data.sensor(0).name.replace(" ", "_"):sensdata0,data.sensor(1).name.replace(" ", "_"):sensdata1}
    savemat(matlabdatafile, mdict)
    # print(f'not saving  %s and %s'%(data.sensor(0).name, data.sensor(1).name))
    # mdict={'ts': ts, 'qpos': q_pos, 'qvel':q_vel}
