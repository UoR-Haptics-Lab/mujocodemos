# Make a video using mujoco (vanilla)
#
# Based on the glfw code written by Pranav Bhounsule (U. Illinois Chicago)
# using -H and -W will likely cause a crash

import mujoco as mj
from mujoco.glfw import glfw
import argparse

import PIL.Image
import numpy as np
import os
import time
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

# --- GLFW functions based on Pranav Bhounsule's code -----------------------------------------------

button_left = False
button_middle = False
button_right = False
lastx = 0
lasty = 0


def keyboard(window, key, scancode, act, mods):
    if act == glfw.PRESS and key == glfw.KEY_BACKSPACE:
        mj.mj_resetData(model, data)
        mj.mj_forward(model, data)

def mouse_button(window, button, act, mods):
        # update button state
        button_left = (glfw.get_mouse_button(
            window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS)
        button_middle = (glfw.get_mouse_button(
            window, glfw.MOUSE_BUTTON_MIDDLE) == glfw.PRESS)
        button_right = (glfw.get_mouse_button(
            window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS)

        # update mouse position
        glfw.get_cursor_pos(window)

def mouse_move(window, xpos, ypos):
    # compute mouse displacement, save
    global lastx
    global lasty
    dx = xpos - lastx
    dy = ypos - lasty
    lastx = xpos
    lasty = ypos

    # no buttons down: nothing to do
    if (not button_left) and (not button_middle) and (not button_right):
        return

    # get current window size
    width, height = glfw.get_window_size(window)

    # get shift key state
    PRESS_LEFT_SHIFT = glfw.get_key(
        window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS
    PRESS_RIGHT_SHIFT = glfw.get_key(
        window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS
    mod_shift = (PRESS_LEFT_SHIFT or PRESS_RIGHT_SHIFT)

    # determine action based on mouse button
    if button_right:
        if mod_shift:
            action = mj.mjtMouse.mjMOUSE_MOVE_H
        else:
            action = mj.mjtMouse.mjMOUSE_MOVE_V
    elif button_left:
        if mod_shift:
            action = mj.mjtMouse.mjMOUSE_ROTATE_H
        else:
            action = mj.mjtMouse.mjMOUSE_ROTATE_V
    else:
        action = mj.mjtMouse.mjMOUSE_ZOOM

    mj.mjv_moveCamera(model, action, dx/height,
                      dy/height, scene, abscam)

def scroll(window, xoffset, yoffset):
    action = mj.mjtMouse.mjMOUSE_ZOOM
    mj.mjv_moveCamera(model, action, 0.0, -0.05 *
                      yoffset, scene, abscam)
## --
def pb_openwindow(width:int, height:int):
        # Init GLFW, create window, make OpenGL context current, request v-sync
        glfw.init()

        window = glfw.create_window(width, height, "Demo", None, None)
        glfw.make_context_current(window)
        glfw.swap_interval(1)

        # initialize visualization data structures
        mj.mjv_defaultCamera(abscam)
        mj.mjv_defaultOption(optvid)
        scene = mj.MjvScene(model, maxgeom=10000)
        context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)

        # install GLFW mouse and keyboard callbacks
        glfw.set_key_callback(window, keyboard)
        glfw.set_cursor_pos_callback(window, mouse_move)
        glfw.set_mouse_button_callback(window, mouse_button)
        glfw.set_scroll_callback(window, scroll)

        return window,scene,context
    
# ---- Other Functions ------------------------------------------------

# -- Parameters and defaults (needs to be before the command line parser --

defruntime=3
defwinheight=240
defwinwidth=320
defframerate=30

# ---- Command line parser -----------------------------------------------
parser = argparse.ArgumentParser(description='Load a mujoco file, run a simulation and save data for matlab.')
parser.add_argument('-X','--xml', nargs='?', const='default.xml', help='Use the default or this xml file') # nargs='?' implies zero or one argument
parser.add_argument('-r', '--runtime',type=float,default=defruntime,help='set the runtime in seconds')  # runtime
parser.add_argument('-m', '--savematlab',action='store_true', help='Save the frames into a matlab matrix')  
parser.add_argument('-s', '--savepngs',action='store_true', help='Save the frames as numbered png images.') #
parser.add_argument('-fr','--framerate',type=int, default=defframerate, help='Set the framerate.') # 
parser.add_argument('-H', '--height',type=int, default=defwinheight, help='Set the window height.') # 
parser.add_argument('-W', '--width',type=int, default=defwinwidth, help='Set the window width.') # 

args=parser.parse_args()

# ---- Command line done -------------------------------------------------------


if args.xml == None:
    model = mj.MjModel.from_xml_string(swinging_body)
else:
    model = mj.MjModel.from_xml_path(args.xml) # Model loaded from xml
    
data = mj.MjData(model)                # MuJoCo data
abscam = mj.MjvCamera()                        # Abstract camera
optvid = mj.MjvOption()                        # visualization options


# Simulate and display video.
frames = [] # will be a list of numpy.ndarray s

mycnt=1

window,scene,context=pb_openwindow(320,240)


dbuf=np.ndarray([240,320], dtype=np.float32) # need space to pass into glfw.readpixels

starttime=time.time()
while not glfw.window_should_close(window) and (time.time() - starttime) < args.runtime: # :
    mj.mj_step(model, data)

    viewport_width, viewport_height = glfw.get_framebuffer_size(window)
    viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)

    # Update scene and render
    mj.mjv_updateScene(model, data, optvid, None, abscam, mj.mjtCatBit.mjCAT_ALL.value, scene)
    mj.mjr_render(viewport, scene, context)
    if len(frames) < data.time * args.framerate:
        pixels=np.ndarray([240,320,3], dtype=np.uint8) # need space to pass into glfw.readpixels
        mj.mjr_readPixels(pixels,dbuf,viewport,context)
        # pixels = physics.render(scene_option=scene_option)
        frames.append(pixels)
        print("recorded %d frames %f"%(mycnt,data.time))
        if args.savepngs:
            myimage=PIL.Image.fromarray(pixels)
            myimage.save('tst%02d.png'%mycnt)
        mycnt=mycnt+1

                    
    glfw.swap_buffers(window) # swap OpenGL buffers (blocking call due to v-sync)
    glfw.poll_events() # process pending GUI events, call GLFW callbacks

print("total recorded %d frames"%mycnt-1)

if args.savematlab:
    savemat('framesdata.mat',{'frames':frames})


# ---- Junk DNA -------------------------------------------------------
# screendata = glReadPixels(0, 0, viewport_width, viewport_height, GL_RGBA, GL_UNSIGNED_BYTE,None)
# pix=mj.mjr_readPixels(rgb=pixels,depth=24,viewport=viewport,con=context)
# ---
# <class 'numpy.ndarray'>
#print("pixels")
#print(type(pixels))
#print(pixels.shape)
#print(pixels.dtype)
#print(len(pixels))

#print("frame")
#print(len(frames))
#print(type(frames))
