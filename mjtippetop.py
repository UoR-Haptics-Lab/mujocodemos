# Tippytop from colab myjoco python tutorial
# Note, the xml is embedded in this file.
# This code can run headless or with graphics (-v)
#
# Based on 
# https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/tutorial.ipynb

from scipy.io import savemat # needed for matlab style matrices
from inspect import currentframe, getframeinfo
import argparse
import mujoco as mj
import mujoco.viewer
import numpy as np
import time
#import faulthandler
#from icecream import ic
import itertools
import os


# ballast has contype=conaffinity=0 so only inertia is considered
# https://mujoco.readthedocs.io/en/stable/XMLreference.html#body-geom
tippe_top = """
<mujoco model="tippe top">
  <option integrator="RK4"/>

  <asset>
    <texture name="grid" type="2d" builtin="checker" rgb1=".1 .2 .3"
     rgb2=".2 .3 .4" width="300" height="300"/>
    <material name="grid" texture="grid" texrepeat="8 8" reflectance=".2"/>
  </asset>

  <worldbody>
    <geom size=".2 .2 .001" type="plane" material="grid"/>
    <light pos="0 0 .6"/>
    <camera name="closeup" pos="0 -.1 .07" xyaxes="1 0 0 0 1 2"/>
    <body name="top" pos="0 0 .02">
      <freejoint/>
      <geom name="ball" type="sphere" size=".02" />
      <geom name="stem" type="cylinder" pos="0 0 .02" size="0.004 .008"/>
      <geom name="ballast" type="box" size=".023 .023 0.005"  pos="0 0 -.015"
       contype="0" conaffinity="0" group="3"/>
    </body>
  </worldbody>

  <keyframe>
    <key name="spinning" qpos="0 0 0.02 1 0 0 0" qvel="0 0 0 0 1 200" />
    <key name="spinning198" qpos="0 0 0.02 1 0 0 0" qvel="0 0 0 0 1 198" />
    <key name="spinning90" qpos="0 0 0.02 1 0 0 0" qvel="0 0 0 0 1 10" />
  </keyframe>
</mujoco>
"""

# ---- Global and default variables ----------

matlabdatafile="ttdata.mat"
# log variables
timevals = []
angular_velocity = []
stem_height = []
q_pos = []
q_vel = []

mdict={'ts': timevals, 'angvel': angular_velocity, 'stem_height':stem_height, 'qpos':q_pos, 'qvel':q_vel}





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

# ---- Command line done -------------------------------------------------------


# ---- Functions -------------------------------------------------------

# called each time around the loop
def simandcollect():
    mj.mj_step(model, data)
    timevals.append(data.time)
    angular_velocity.append(data.qvel[3:6].copy())
    stem_height.append(data.geom_xpos[2,2])
    q_pos.append(data.qpos[0:7].copy())
    q_vel.append(data.qvel[0:6].copy())

def get_linenumber(): # needs inspect
    cf = currentframe()
    return cf.f_back.f_lineno


    
# ---- Functions done -------------------------------------------------------

# ---- main programme ----------

# Either of the following 2 lines should be used to load the model (from string or file)
# 
if args.xml == None:
    model = mj.MjModel.from_xml_string(tippe_top)
else:
    model = mj.MjModel.from_xml_path(args.xml) # Model loaded from xml, that can inturn call on URDF etc    
data = mj.MjData(model)                # MuJoCo data
# renderer = mj.Renderer(model)



#mj.mj_forward(model, data)
# renderer.update_scene(data, camera="closeup")
# media.show_image(renderer.render())

# faulthandler.enable() # Seems to be a SIGSEGV 

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
    mdict = {**mdict,'mass':model.body_mass, 'inertia':model.body_inertia}
    savemat(matlabdatafile, mdict)

if args.log : # save as csv files
    if not os.path.isdir("Data"):
        os.mkdir('Data')
        print("A Data folder/directory has been created in ",os.getcwd())
    for k, v in mdict.items():
        np.savetxt(os.path.join('Data',k),v,delimiter=',')



if(args.viewer): # 
    print(f"Debug: Viewer crash issue, reached line %d (if -l or -m then data was saved)"% get_linenumber())

# ---- print out some data ----
# print(model.body('top'))

