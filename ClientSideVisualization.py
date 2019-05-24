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

loadPrcFileData('', 'win-size 3840 2160') 
import sys
import os
from direct.stdpy import threading

xaxis = float(0) #establishing axis variables
yaxis = float(0)
zaxis = float(0)

#setup = input("First time running program or program has been updated? (Y/N)")
setup = "N"
if setup == "Y":
    print("Running first-time setup")
    command1 = '''
    git pull origin master
    cd C++/ProjectFiles/AHRS
    make
    ./AHRS -i mpu'''
else:
    print("Not running first-time setup")
    command1 = '''
    cd C++/ProjectFiles/AHRS
    make
    ./AHRS -i mpu'''


#made own class to make it easier to work with instead of raw paramiko data
class ssh:
    client = None
 
    def __init__(self, address, username, password):
        print("Connecting to server.")
        self.client = client.SSHClient()
        #self.transport = self.transport.Transport(1)
        self.client.set_missing_host_key_policy(client.AutoAddPolicy())
        self.client.connect(address, username=username, password=password, look_for_keys=False)
 
    def sendCommand(self, command):
        if(self.client):
            print("now sending")
            stdin, stdout, stderr = self.client.exec_command(command) #creates channel plus stdin, stdout, stderr
            stdin.close()
            global xaxis #program prints xaxis, then yaxis, then zaxis in sequential lines, so this function sets them
            global yaxis 
            global zaxis
            global cube
            which = 1
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
                if (iteration % 30) == 0:
                    print("XAXIS, YAXIS, ZAXIS: " + str(xaxis) + " " + str(yaxis) + " " + str(zaxis))
            else:
                print("Connection not opened.") #error clause


print("connecting to server")
connection = ssh("192.168.43.88", "pi", "raspberry") #will change if network is changed but is static on my hotspot
#so not changing it for now


#can't send multiple commands without closing channel, so one long one instead

########################## Start of renderer ######################################
base = ShowBase()
base.disableMouse()
base.camera.setPos(0, 9000, 800)
base.camera.setHpr(180, 0, 0)

class MyRenderer(ShowBase):
    def __init__(self):
        ShowBase.__init__(self)
 
        # Load the environment model.
        self.scene = self.loader.loadModel("models/environment")
        # Reparent the model to render.
        self.scene.reparentTo(self.render)
        # Apply scale and position transforms on the model.
        self.scene.setScale(0.25, 0.25, 0.25)
        self.scene.setPos(-8, 42, 0)
 
        # Add the spinCameraTask procedure to the task manager.
        self.taskMgr.add(self.spinCameraTask, "SpinCameraTask")
 
        # Load and transform the panda actor.
        self.pandaActor = Actor("models/panda-model",
                                {"walk": "models/panda-walk4"})
        self.pandaActor.setScale(0.005, 0.005, 0.005)
        self.pandaActor.reparentTo(self.render)
        # Loop its animation.
        self.pandaActor.loop("walk")

cube = loader.loadModel("Testingmats2.x")
#tex = loader.loadTexture("maps/planetex.png")
#cube.setTexture(tex, 1)
cube.setScale(1)
cube.reparentTo(base.render)


title = OnscreenText(text="Plane Visualizer",
                     style=1, fg=(1, 1, 1, 1), pos=(-0.1, 0.1), scale=.07,
                     parent=base.a2dBottomRight, align=TextNode.ARight)


#cant normalize inline so this is a helper function
def normalized(*args):
    myVec = LVector3(*args)
    myVec.normalize()
    return myVec

#helper function to make a square given the lower left hand and upper right hand corners
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
def rotatemycube(task):
  global cube
  global caliboffset
  cube.setHpr((float(xaxis) - caliboffset)*-1, (float(zaxis)), float(yaxis))
  return task.again


offsettick = 0
def offset(task):
    global offsettick
    global xaxis
    if offsettick == 20:
        caliboffset = xaxis
    offsettick = offsettick + 1





class MyTapper(DirectObject):

    def __init__(self):
        self.testTexture = loader.loadTexture("maps/envir-reeds.png")
        self.accept("1", self.breakProgram)

        slight = Spotlight('slight')
        slight.setColor((1, 1, 1, 1))
        lens = PerspectiveLens()
        slight.setLens(lens)
        self.slnp = render.attachNewNode(slight)
        self.slnp1 = render.attachNewNode(slight)
        taskMgr.add(rotatemycube)
        taskMgr.add(offset)

    def breakProgram(self):
        str = "test"
        return int(str)

######################### End of renderer ##########################

class TestThread(threading.Thread):

    def __init__(self):
        threading.Thread.__init__(self)

    def run(self):
        connection.sendCommand(command1) #Sends the command and receives the input, changes the global x y and z axis variables as data comes in
        #and the renderer receives this data and renders the cube to the orientation

#t2 = thread(connection.sendCommand(command1))
#connection.sendCommand(command1)

thread = TestThread()
thread.start()
#thread.join()

time.sleep(1) #the make command messes it up
t = MyTapper()
base.run()
#base.run()

#time.sleep(5)

