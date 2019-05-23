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
import sys
import os
from direct.stdpy import threading
#from direct.stdpy import threading2
#from direct.stdpy.file import *

xaxis = float(0)
yaxis = float(0)
zaxis = float(0)


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
            stdin, stdout, stderr = self.client.exec_command(command)
            stdin.close()
            global xaxis #program prints xaxis, then yaxis, then zaxis in sequential lines, so this function sets them
            global yaxis 
            global zaxis
            global cube
            which = 1
            iteration = 0
            for line in iter(lambda: stdout.readline(2048), ""):
                if not((line[0] == "0") or (line[0] == "1") or (line[0] == "2") or (line[0] == "3") or (line[0] == "4") or (line[0] == "5") or (line[0] == "6") or (line[0] == "7") or (line[0] == "8") or (line[0] == "9") or (line[0] == "-") or (line[0] == "+")):
                    xaxis = xaxis #basically an ignore clause because the iter function adds newlines (doesnt work without iter, so not removing it for now)
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
                if (iteration % 30) == 0: #for some reason xaxis is compass, yaxis is xaxis, and zaxis is xaxis
                    print("XAXIS, YAXIS, ZAXIS: " + str(xaxis) + " " + str(yaxis) + " " + str(zaxis))
            else:
                print("Connection not opened.")


print("connecting")
connection = ssh("192.168.43.88", "pi", "raspberry")


#can't send multiple commands without closing channel, so one long one instead
print("starting command sequence")
command1='''
cd C++/ProjectFiles/AHRS
make
./AHRS -i mpu
'''
base = ShowBase()
base.disableMouse()
base.camera.setPos(0, -10, 0)

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

    vertex = GeomVertexWriter(vdata, 'vertex')
    normal = GeomVertexWriter(vdata, 'normal')
    color = GeomVertexWriter(vdata, 'color')
    texcoord = GeomVertexWriter(vdata, 'texcoord')

    #gotta draw the sqaure in the right plane
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

    square = Geom(vdata)
    square.addPrimitive(tris)
    return square

#it isnt efficient to make every face as a separate geom, it would be better to create one geom holding all of the faces - maybe later?
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

cube = render.attachNewNode(snode)
rotation = cube.setHpr(0, 0, 0) #does nothing

#opengl by default only draws front faces (polygons whose vertices are specified ccw)
cube.setTwoSided(True)

def rotatemycube(task):
  global cube
  cube.setHpr(float(xaxis)*-1, float(yaxis)*-1, float(zaxis))
  return task.again



class MyTapper(DirectObject):

    def __init__(self):
        self.testTexture = loader.loadTexture("maps/envir-reeds.png")

        slight = Spotlight('slight')
        slight.setColor((1, 1, 1, 1))
        lens = PerspectiveLens()
        slight.setLens(lens)
        self.slnp = render.attachNewNode(slight)
        self.slnp1 = render.attachNewNode(slight)
        taskMgr.add(rotatemycube)

class TestThread(threading.Thread):

    def __init__(self):
        threading.Thread.__init__(self)

    def run(self):
        connection.sendCommand(command1)

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

