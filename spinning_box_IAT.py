## Demonstrates the dzhanibekov effect/ intermediate axis theorem / tennis racket theorem
## The intermediate axis in this example is z 
## Either run the python directly or from matlab
## Static world
#    !python .\spinning_box_IAT.py  -r 25 -v -kf 0 -m
## Spinning around z at 20 deg/sec with no other spin direction (spin is stable)
#    !python .\spinning_box_IAT.py  -r 25 -v -kf 1 -m
## Spinning mostly around z at about 5 deg/sec (flips every 4.3 seconds )
#    !python .\spinning_box_IAT.py  -r 25 -v -kf 4 -m
## Spinning mostly around z at about 50 deg/sec (flips every 0.62 seconds )
#    !python .\spinning_box_IAT.py  -r 10 -v -kf 7 -m
## To see results in matlab
#    >> load dbdata.mat
#    >> figure(1);plot(ts,qvel);grid on;figure(2);plot(ts,qpos);grid on;

## This may be a solution to the problem of crashes
# The GLFW library is not initialized
# https://stackoverflow.com/questions/17381811/glfw-3-initialized-yet-not
# https://www.glfw.org/download.html


import argparse
import mujoco as mj
import mujoco.viewer
import numpy as np
import time
from inspect import currentframe, getframeinfo
from scipy.io import savemat # needed for matlab style matrices

Box = """
<mujoco model="IAT">
  <option gravity="0 0 0" timestep="0.001" integrator="RK4" />
  <asset>
    <texture name="grid" type="2d" builtin="checker" width="512" height="512" rgb1=".1 .2 .3" rgb2=".2 .3 .4"/>
    <material name="grid" texture="grid" texrepeat="1 1" texuniform="true" reflectance=".2"/>
  </asset>

  <worldbody>
    <light diffuse=".9 .9 .9" pos="0 0 10" dir="0 0 -1"/>
    <geom type="plane" size="3 3 .01" material="grid"/>
    <body name="boxA" pos="0 0 .05">
      <freejoint/>
      <geom name="chasis" type="box" size=".01 .03 .02" /> <!-- axisangle="1 0 0 20" -->
      <!--geom name="front wheel" pos=".08 0 -.015" type="sphere" size=".015" condim="1" priority="1"/-->
    </body>

  </worldbody>

  <keyframe>
    <key name="spinning0"  qvel="0 0 0 0 0 0" />
    <key name="spinning1"  qvel="0 0 0 0 0 20" />
    <key name="spinning2"  qvel="0 0 0 0 20 0" />
    <key name="spinning3"  qvel="0 0 0 20 0 0" />
    <key name="spinning4"  qvel="0 0 0 0 0.1 5" />
    <key name="spinning5"  qvel="0 0 0 0.1 5 0" />
    <key name="spinning6"  qvel="0 0 0 5 0 0.1" />
    <key name="spinning7"  qvel="0 0 0 0 0.1 50" />
  </keyframe>
</mujoco>
"""

# ---- Global and default variables ----------

matlabdatafile="dbdata.mat"
# log variables
timevals = []
angular_velocity = []
stem_height = []
q_pos = []
q_vel = []

mdict={'ts': timevals, 'angvel': angular_velocity, 'qpos':q_pos, 'qvel':q_vel}



# ---- Command line -------------------------------------------------------
parser = argparse.ArgumentParser(description='Load a mujoco file, run a simulation and save data for matlab.')

parser.add_argument('-v', '--viewer',action='store_true',help='use the mjuoco viewer')      # viewer
parser.add_argument('-vrt', '--rtviewer',action='store_true',help='to be done. View in or out of real time')      # viewer
# parser.add_argument('-x', '--mjmodel',type=str,default='simplecontrolonelink.xml',help='Will use named file as the mjmodel.If not given uses a default file')      # model file
parser.add_argument('-X','--xml', nargs='?', const='default.xml', help='Use the default or this xml file') # nargs='?' implies zero or one argument
parser.add_argument('-kf', '--keyframe',type=int,default=0,help='Set the keyframe if it exists')  # key frame
parser.add_argument('-r', '--runtime',type=float,default=5,help='set the runtime in seconds')  # runtime
parser.add_argument('-l', '--log',action='store_true', help='log the data into csv files')     # csv log
parser.add_argument('-m', '--mathworks',action='store_true', help='log the data into a matlab matrix')  # matlab logs may be faster
parser.add_argument('-s', '--numsensors',type=int, default=-1, help='t.b.d. save the sensor data. This number must be less than or equal to the number of sensors.') # t.b.d.  
args=parser.parse_args()

# ---- Functions -------------------------------------------------------

# called each time around the loop
def simandcollect():
    mj.mj_step(model, data)
    timevals.append(data.time)
    #angular_velocity.append(data.qvel[3:6].copy())
    #stem_height.append(data.geom_xpos[2,2])
    q_pos.append(data.qpos[0:7].copy())
    q_vel.append(data.qvel[0:6].copy())

def get_linenumber(): # needs inspect
    cf = currentframe()
    return cf.f_back.f_lineno
# __main__
model = mj.MjModel.from_xml_string(Box)
data = mj.MjData(model)                # MuJoCo data
renderer = mj.Renderer(model)

kf=args.keyframe
mj.mj_resetDataKeyframe(model, data, kf)  # Reset the state to keyframe 0

#model.opt.timestep = .001
    
if(args.viewer): # viewer is the -v option
    with mj.viewer.launch_passive(model, data) as viewer:
        viewer.cam.distance=.5;
        start = time.time()
        while viewer.is_running() and (time.time() - start) < args.runtime: # 
            simandcollect()
            viewer.sync() # passive viewer updates to model and data
            elapsedtime=time.time()-start # comment out this and the next two lines to not attempt real time 
            if data.time > elapsedtime : 
                time.sleep(data.time-elapsedtime)
        print(f'model time=%0.2f, actual time=%0.2f'%(data.time, time.time()-start))
        viewer.close()
        print(f"Debug: Viewer crash issue, reached line %d "% get_linenumber())

else: # no viewer
    while data.time < args.runtime:
        simandcollect()

# ---- save data ----------
# uses mdict to determine the variables to save

if args.mathworks >0 : # save as a matlab data
    savemat(matlabdatafile, mdict)

if(args.viewer): # 
    print(f"Debug: Viewer crash issue, reached line %d (if -l or -m then data was saved)"% get_linenumber())
