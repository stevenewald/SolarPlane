# Autonomous solar plane
Description is a bit outdated, now using C++ based stabilization and using GCS rather than python/c++ multithreading airplane visualization. I added the SSH server reliant orientation renderer/visualization on the GCS because I had to do a final CS project for school and didn't want to stop working on this project. This description is mainly relevant for the renderer, rather than the actual plane pathfinding algorithms for that reason.
I have tested the stabilization, and it worked perfectly. It worked for about 6 minutes (the planned time) but the code stopped running because I forgot to use TMUX to stop the SSH disconnection from causing shell termination so the plane hit the trees but since the code worked and I was switching to an alternate plane model anyway (this one didn't have ailerons and was a prototype), no progress was lost. I also got the flight data logs over an FTP connection while the plane was in the trees, lmao

## Introduction

> I am making a solar rc plane (~8.5ft wingspan, 7lbs, electric) and am using a navio2 and raspberry pi 3 B+ to have it fly autonomously. This involves taking the Euler angles (pitch, roll, heading) to stabilize the plane as well as change the heading, altitude, and throttle to navigate between waypoints. The point of this project was to create a visualization for the orientation of the plane. Because the solar plane will go 10-50 miles away at any time, it is useful to be able to see the orientation of the plane. Right now the visualization is just a cube, but it will be a plane once I model it (which isn't programming).

## Project summary
### 1. Calculating Euler angles from the raw accelerometer, gyroscope, and magnetometer data
>> I had to rewrite this three times, twice in Python and once in C++, to get it to work correctly. This involves a lot of extremely complicated math and geometry that would definitely require a math degree to actually understand, so I just implemented berryIMU's code (different IMU so I had to change a lot, including translating it into C++).

> ## 2. Transmitting Euler angles over an SSH connection
>> Once the Euler angles are calculated, the server side (navio) C++ program prints the three angles using the "cout" function. On the client side (my laptop), the python program receives the lines of bytes using the paramiko SSH package. It assigns each of the three lines to the x-axis, y-axis, and z-axis variables.

> ## 3. Visualizes the orientation
>> Using panda3d, a cube is rendered with the X, Y, and Z orientation given by the previously mentioned x-axis, y-axis, and z-axis variables. There are two processes that run on the client side program:
>>> 1. Rendering
>>>> This process renders the cube at the HPR orientation.
>>> 2. HPR assigning
>>>> This process assigns the SSH data to the Euler/HPR angles. Because it runs at the same time as rendering, it runs using threading. Because Panda3D is based on C++, it has to use a special threading package in order to interface with the C++ based Panda3D and the python-based client-side program. The render window pops up as a new pygame window and renders a cube with colored vertices on a grey background.

## Code Samples

> ## 1: from direct.stdpy import threading
>> This package was needed to run the SSH data processing on a separate thread from the rendering process, because Panda3d (the rendering engine) is C++ based and the client-side programming is python based.


> ## 2: def rotatemycube(task):
  ##   global cube
  ##   cube.setHpr(float(xaxis)*-1, float(yaxis)*-1, float(zaxis))
  ##   return task.again
>> This is the second threading example (although not coded in manually) that is done automatically by the panda3d engine to be able to change the orientation of the cube while also rendering the cube.

## Installation

### Client side packages required:
> 1. PyPanda3D (Panda3D for Python) - Visualization renderer
2. Paramiko - SSH client


## Network setup
> This is the difficult part. I'll set the board up to the wifi network wanted (it's too complicated to describe here). The board should be plugged into the battery supplied, and the computer should be on the same wifi network that the board was set up to be on. Run the python program about 20-30 seconds after plugging in the navio, and the visualization should run.

### There are two files relevant to this project, all of the rest are either test files or package files. The two relevant files are:
> ### 1. /ClientSideVisualization.py
>> This program receives the Euler angles from the flight control board (the navio) and renders a visualization of the orientation.

> ### 2. /C++/ProjectFiles/AHRS
>> This program is server side and is not neccesary to run. It will not run on a computer (it uses navio packages that only work on a navio board). This program calculates and transmits the Euler angles.
