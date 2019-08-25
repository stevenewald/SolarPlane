from paramiko import transport
from paramiko import client
import time
from direct.showbase.ShowBase import ShowBase
from direct.showbase.DirectObject import DirectObject
from direct.gui.DirectGui import *
from direct.interval.IntervalGlobal import *
from panda3d.core import lookAt
from panda3d.core import GeomVertexFormat, GeomVertexData
from panda3d.core import Geom, GeomTriangles, GeomVertexWriter
from panda3d.core import Texture, GeomNode
from panda3d.core import PerspectiveLens
from panda3d.core import CardMaker
from panda3d.core import Light, Spotlight
from panda3d.core import TextNode
from panda3d.core import LVector3
from direct.task import Task
from panda3d.core import loadPrcFileData 

loadPrcFileData('', 'win-size 3840 2160') #sets the window size as fullscreen borderless
import sys
import os
from direct.stdpy import threading

xaxis = float(0) #establishing axis variables
yaxis = float(0)
zaxis = float(0)


#made own class to make it easier to work with instead of raw paramiko data
class ssh:
    client = None
 
    #connects to the ssh server
    def __init__(self, address, username, password):
        print("Connecting to server.")
        self.client = client.SSHClient()
        #self.transport = self.transport.Transport(1)
        self.client.set_missing_host_key_policy(client.AutoAddPolicy())
        self.client.connect(address, username=username, password=password, look_for_keys=False)
 
    #continually sends and receives the datastream in the form of yaxis, xaxis, zaxis and appends the data to it
    def sendCommand(self, command):
        if(self.client):
            print("now sending")
            stdin, stdout, stderr = self.client.exec_command(command) #creates channel plus stdin, stdout, stderr
            stdin.close() #doesnt take in any more commands
            global xaxis #program prints xaxis, then yaxis, then zaxis in sequential lines, so this function sets them
            global yaxis 
            global zaxis
            global cube
            which = 1 #assigns each stdout to the x, y, or z axis
            iteration = 0 #basically a tick function to prevent the xaxis yaxis and zaxis from printing every tick
            for line in iter(lambda: stdout.readline(2048), ""): #please dont fail me for this, it works and thats all that matters
                if not((line[0] == "0") or (line[0] == "1") or (line[0] == "2") or (line[0] == "3") or (line[0] == "4") or (line[0] == "5") or (line[0] == "6") or (line[0] == "7") or (line[0] == "8") or (line[0] == "9") or (line[0] == "-") or (line[0] == "+")):
                    xaxis = xaxis #basically an ignore clause
                elif which == 1:
                    zaxis = (line)
                    which = which + 1
                elif which == 2:
                    yaxis = (line)
                    which = which + 1
                elif which == 3:
                    xaxis = (line)
                    which = 1
                else:
                    print("ERROR!")
                    break
                iteration = iteration + 1
                if (iteration % 30) == 0: #prevent overflow by limiting stdout
                    print("XAXIS, YAXIS, ZAXIS: " + str(xaxis) + " " + str(yaxis) + " " + str(zaxis))
            else:
                print("Connection not opened.") #error clause


print("connecting to server")
connection = ssh("10.0.0.60", "pi", "raspberry") #will change if network is changed but is static on my hotspot
#so not changing it for now


#can't send multiple commands without closing channel, so one long one instead

########################## Start of renderer ######################################
#sets up the renderer
base = ShowBase()
base.disableMouse()

#this position and orientation puts the camera at the back of the plane, facing the plane
base.camera.setPos(0, 11000, 800)
base.camera.setHpr(180, 0, 0)

#"testingmats2.x" is the plane model
#its named like that because I had to try many times before I got the mats to work with the .mtl file
#it was originally named cube because it was an actual cube, and now theres no point in changing it
cube = loader.loadModel("Testingmats2.x")
#setscale is self explanatory
cube.setScale(1)
#attaches the plane to the renderer
cube.reparentTo(base.render)


#shows the "plane visualizer" text onscreen
title = OnscreenText(text="Plane Visualizer",
                     style=1, fg=(1, 1, 1, 1), pos=(-0.1, 0.1), scale=.07,
                     parent=base.a2dBottomRight, align=TextNode.ARight)


#cant normalize inline so this is a helper function
def normalized(*args):
    myVec = LVector3(*args)
    myVec.normalize()
    return myVec

#helper function to make a square given the lower left hand and upper right hand corners
#because I now use the plane, this function is a bit useless
#but I won't delete it because it shows how the cube was made
def makeSquare(x1, y1, z1, x2, y2, z2):
    format = GeomVertexFormat.getV3n3cpt2()
    vdata = GeomVertexData('square', format, Geom.UHDynamic)

    #initializing variables
    vertex = GeomVertexWriter(vdata, 'vertex')
    normal = GeomVertexWriter(vdata, 'normal')
    color = GeomVertexWriter(vdata, 'color')
    texcoord = GeomVertexWriter(vdata, 'texcoord')

    #gotta draw the sqaure in the right plane
    #aligning vertexes to the right planes
    if x1 != x2:
        vertex.addData3(x1, y1, z1)
        vertex.addData3(x2, y1, z1)
        vertex.addData3(x2, y2, z2)
        vertex.addData3(x1, y2, z2)

        normal.addData3(normalized(2 * x1 - 1, 2 * y1 - 1, 2 * z1 - 1))
        normal.addData3(normalized(2 * x2 - 1, 2 * y1 - 1, 2 * z1 - 1))
        normal.addData3(normalized(2 * x2 - 1, 2 * y2 - 1, 2 * z2 - 1))
        normal.addData3(normalized(2 * x1 - 1, 2 * y2 - 1, 2 * z2 - 1))

    else:
        #adding vertexes
        vertex.addData3(x1, y1, z1)
        vertex.addData3(x2, y2, z1)
        vertex.addData3(x2, y2, z2)
        vertex.addData3(x1, y1, z2)

        normal.addData3(normalized(2 * x1 - 1, 2 * y1 - 1, 2 * z1 - 1))
        normal.addData3(normalized(2 * x2 - 1, 2 * y2 - 1, 2 * z1 - 1))
        normal.addData3(normalized(2 * x2 - 1, 2 * y2 - 1, 2 * z2 - 1))
        normal.addData3(normalized(2 * x1 - 1, 2 * y1 - 1, 2 * z2 - 1))

    #adding different colors to the vertex for visibility
    color.addData4f(1.0, 0.0, 0.0, 1.0)
    color.addData4f(0.0, 1.0, 0.0, 1.0)
    color.addData4f(0.0, 0.0, 1.0, 1.0)
    color.addData4f(1.0, 0.0, 1.0, 1.0)

    texcoord.addData2f(0.0, 1.0)
    texcoord.addData2f(0.0, 0.0)
    texcoord.addData2f(1.0, 0.0)
    texcoord.addData2f(1.0, 1.0)

    #quads aren't directly supported by the geom interface
    tris = GeomTriangles(Geom.UHDynamic)
    tris.addVertices(0, 1, 3)
    tris.addVertices(1, 2, 3)

    #initialize surface
    square = Geom(vdata)
    square.addPrimitive(tris)
    return square

#it isnt efficient to make every face as a separate geom, it would be better to create one geom holding all of the faces - maybe later?
#this creates the vertices and surfaces of the square
square0 = makeSquare(-1, -1, -1, 1, -1, 1)
square1 = makeSquare(-1, 1, -1, 1, 1, 1)
square2 = makeSquare(-1, 1, 1, 1, -1, 1)
square3 = makeSquare(-1, 1, -1, 1, -1, -1)
square4 = makeSquare(-1, -1, -1, -1, 1, 1)
square5 = makeSquare(1, -1, -1, 1, 1, 1)
snode = GeomNode('square')
snode.addGeom(square0)
snode.addGeom(square1)
snode.addGeom(square2)
snode.addGeom(square3)
snode.addGeom(square4)
snode.addGeom(square5)

#cube = render.attachNewNode(plane)
rotation = cube.setHpr(0, 0, 0) #does nothing

#opengl by default only draws front faces (polygons whose vertices are specified ccw)
cube.setTwoSided(True)


caliboffset = 0
#when this is called, it changes the orientation of the cube to the global x y and z axes
#caliboffset is the offset to make sure that the heading will always be 0 at the start heading of the navio
def rotatemycube(task):
  global cube
  global caliboffset
  cube.setHpr((float(xaxis) - float(caliboffset))*-1, (float(zaxis))*-1, float(yaxis)*-1)
  return task.again

#sets the offset so that plane will always be at heading 0 when visualization starts
#issue with AHRS program but this fix works alright
offsettick = 0

#Render class
class MyTapper(DirectObject):

    def __init__(self):
        self.accept("1", self.breakProgram)

        #lighting
        slight = Spotlight('slight') 
        slight.setColor((1, 1, 1, 1))
        lens = PerspectiveLens()
        slight.setLens(lens)
        #attaching lighting
        self.slnp = render.attachNewNode(slight)
        self.slnp1 = render.attachNewNode(slight)

        #runs as a thread to change the HPR of the plane
        taskMgr.add(rotatemycube)

        #adds the offset

    #Can't find exit clause so this works
    #Activates if you press "1"
    def breakProgram(self):
        str = "break"
        return int(str)

######################### End of renderer ##########################

#Connects to the navio and sends the commands outlined above
class ConnectionThread(threading.Thread):

    def __init__(self):
        threading.Thread.__init__(self)

    def run(self):
        connection.sendCommand(command1) #Sends the command and receives the input, changes the global x y and z axis variables as data comes in
        #and the renderer receives this data and renders the cube to the orientation

#Calculates the compass offset so that heading starts at zero
#Algorithm can't do this currently so this is a temporary fix
#Could later serve as a changed visualization clause, like "30 seconds in change the orientation to the target orientation"
class OffsetThread(threading.Thread):

    def __init__(self):
        threading.Thread.__init__(self)

    def run(self):
        OffsetCalculatedYet = True
        while OffsetCalculatedYet:
            global offsettick
            global xaxis
            global caliboffset
            if offsettick == 20:
                caliboffset = xaxis
                OffsetCalculatedYet = False
            offsettick = offsettick + 1
            time.sleep(1)


#t2 = thread(connection.sendCommand(command1))
#connection.sendCommand(command1)


#if code has been changed, type Y
#otherwise type N
#setup = input("First time running program or program has been updated? (Y/N)")
setup = "N"
if setup == "Y":
    print("Running first-time setup")
    command1 = '''
    git pull origin master
    cd C++/ProjectFiles/AHRS
    make
    sudo ./AHRS -i mpu'''
else:
    print("Not running first-time setup")
    command1 = '''
    cd C++/ProjectFiles/AHRS
    make
    sudo ./AHRS -i mpu'''

thread = ConnectionThread()
thread.start()

offsetThreadd = OffsetThread()
offsetThreadd.start()
#runs the SSH connection send/receive thread

#runs the renderer
t = MyTapper()
base.run()

