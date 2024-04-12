# Based on https://colab.research.google.com/github/deepmind/dm_control/blob/main/dm_control/mujoco/tutorial.ipynb
from dm_control import mujoco

# Access to enums and MuJoCo library functions.
from dm_control.mujoco.wrapper.mjbindings import enums
from dm_control.mujoco.wrapper.mjbindings import mjlib

import numpy as np
import time
import os
import PIL.Image

from scipy.io import savemat # needed for matlab style matrices

swinging_body = """
<mujoco>
  <worldbody>
    <light name="top" pos="0 0 1"/>
    <body name="box_and_sphere" euler="0 0 -30">
      <joint name="swing" type="hinge" axis="1 -1 0" pos="-.2 -.2 -.2"/>
      <geom name="red_box" type="box" size=".2 .2 .2" rgba="1 0 0 1"/>
      <geom name="green_sphere" pos=".2 .2 .2" size=".1" rgba="0 1 0 1"/>
    </body>
  </worldbody>
</mujoco>
"""

defaultduration = 3    # (seconds)
framerate = 30  # (Hz)

import argparse

# command line 
parser = argparse.ArgumentParser(description='Load a mujoco file, run a simulation and save data for matlab.')

parser.add_argument('-v', '--viewer',action='store_true',help='use the mjuoco viewer')      # viewer
parser.add_argument('-vrt', '--rtviewer',action='store_true',help='to be done. View in or out of real time')      # viewer
# parser.add_argument('-x', '--mjmodel',type=str,default='simplecontrolonelink.xml',help='Will use named file as the mjmodel.If not given uses a default file')      # model file
parser.add_argument('-X','--xml', nargs='?', const='default.xml', help='Use the default or this xml file') # nargs='?' implies zero or one argument
parser.add_argument('-kf', '--keyframe',type=int,default=0,help='Set the mujoco keyframe if it exists')  # key frame
parser.add_argument('-r', '--runtime',type=float,default=defaultduration,help='set the runtime in seconds')  # runtime
parser.add_argument('-l', '--log',action='store_true', help='log the data into csv files')     # csv log
parser.add_argument('-m', '--mathworks',action='store_true', help='log the data into a matlab matrix')  # matlab logs may be faster
parser.add_argument('-s', '--numsensors',type=int, default=-1, help='t.b.d. save the sensor data. This number must be less than or equal to the number of sensors.') # t.b.d.  
args=parser.parse_args()



#------------
physics = mujoco.Physics.from_xml_string(swinging_body)
#pixels = physics.render()


duration = 3    # (seconds)
framerate = 30  # (Hz)

# Visualize the joint axis
scene_option = mujoco.wrapper.core.MjvOption()
scene_option.flags[enums.mjtVisFlag.mjVIS_JOINT] = True

# Simulate and display video.
frames = []
physics.reset()  # Reset state and time
mycnt=1
while physics.data.time < args.runtime:
  physics.step()
  if len(frames) < physics.data.time * framerate:
      pixels = physics.render(scene_option=scene_option)
      frames.append(pixels)
      #myimage=PIL.Image.fromarray(pixels)
      #myimage.save('tst%02d.png'%mycnt)
      mycnt=mycnt+1

#display_video(frames, framerate)
savemat('somedata.mat',{'frames':frames})
filepath='data.raw'
with open (filepath,'wb') as FileToWrite:
  np.asarray(frames,dtype=np.uint8).tofile(FileToWrite)
print(FileToWrite.closed)
