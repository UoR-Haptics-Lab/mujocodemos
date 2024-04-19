# mujocodemos



These files are intended to be short demonstrations of mujoco used for dynamic simulation and control.

This is very much **work in progress** at the moment so feedback is welcome.

The examples draw on the classes and examples taught by Pranav Bhounsule (University of Illinois Chicago but see [RM](https://pab47.github.io/teaching.html)) as well as examples provided my Deep Mind and Mujoco including their colab tutorials [mj](https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/tutorial.ipynb) and [dm](https://colab.research.google.com/github/deepmind/dm_control/blob/main/dm_control/mujoco/tutorial.ipynb)


Programs are intended to produce data that can be analysed elsewhere, e.g. Matlab, Julia, Python, etc. hence the inclusion of several matlab files in this archive.

Example programs for Mujoco for dynamic simulation


 - README.md
 - framestomovie.m
 - makevideodm.py
 - makevideopb.py
 - mjtippetop.m
 - mjtippetop.py
 - mjtippetop_csv.m
 - simplecontrolonelink.xml
  -simplecontrol.py




### Recording video

The two versions of 'make a video' are either mujoco as demonstated in Pranav Bhounsule's lectures (pb) or as used by dm_control (dm)

At the moment the converstion to an established video format is via matlab, but eventually it should be possible to use ffmpeg

	ffmpeg -f rawvideo -pixel_format rgb24 -video_size 320x240 -framerate 30 -i data.raw myvideo.mp4
	ffmpeg -f rawvideo -pixel_format rgb24 -video_size 320x240 -framerate 30 -i data.raw -vf "vflip" myvideo.mp4


### 
