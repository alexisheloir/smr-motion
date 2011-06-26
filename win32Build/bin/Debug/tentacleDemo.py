# Author: Alexis Heloir
# Models and Textures by: Alexis Heloir
# Last Updated: 5/08/08
#

import direct.directbase.DirectStart
import SMRPy
from pandac.PandaModules import AmbientLight,DirectionalLight
from pandac.PandaModules import TextNode,NodePath,LightAttrib
from pandac.PandaModules import Vec3,Vec4
from pandac.PandaModules import Quat
from direct.actor.Actor import Actor
from direct.task.Task import Task
from direct.gui.OnscreenText import OnscreenText
from direct.showbase.DirectObject import DirectObject
import sys

#A simple function to make sure a value is in a given range, -1 to 1 by default
def restrain(i, mn = -1, mx = 1): return min(max(i, mn), mx)

class World(DirectObject):
  def __init__(self):
    #This code puts the standard title and instuction text on screen
    self.title = OnscreenText(text="Panda3D: Embots - Joint Manipulation",
                              style=1, fg=(1,1,1,1),
                              pos=(0.8,-0.95), scale = .07)            
    #setup key input
    self.accept('escape', sys.exit)
                              
    #base.disableMouse() #Disable mouse-based camera-control
    camera.setPos(0,-20,0)              #Position the camera

    self.teapot = loader.loadModel('models/teapot')
    self.teapot.reparentTo(render)
    self.teapot.setScale(0.2)

    self.tentacle = Actor("models/tentacle") #Load our animated charachter
    self.tentacle.reparentTo(render)          #Put it in the scene
    self.tentacle.setScale(1.0)

    self.setupLights()                        #Put in some default lighting

    self.tentacleBase = self.tentacle.controlJoint(None, 'modelRoot', 'Bone')

    self.tentacleIK = SMRPy.SmrKinematicChain(True,True,'tentacle');

    self.jointList = []
    self.constraintList = []

    self.createKinematicChain(self.tentacle, 'Bone', '', 0)

    myJoint = self.tentacleIK.getJointByName("Bone.006")

    self.myConstraint = SMRPy.SmrIKConstraint()
    self.myConstraint.setRelatedJointptr(myJoint)
    myConstraintPosition = SMRPy.SmrVector3(0.0,0.0,0.0)
    self.teapot.setPos(0.0,0.0,0.0)
    self.myConstraint.setPosition(myConstraintPosition)

    myConstraintPosition = SMRPy.SmrVector3(0.0,0.0,0.0)
    self.myConstraint.setOffset(myConstraintPosition)

    self.myKinematicSolver = SMRPy.SmrGSMMSolver(self.tentacleIK)
    self.myKinematicSolver.addConstraintPtr(self.myConstraint)

    taskMgr.add(self.ikStep, "ikStep")
    
  def ikStep(self, task):
    if base.mouseWatcherNode.hasMouse():
      mpos = base.mouseWatcherNode.getMouse()
      self.myConstraint.setPosition(SMRPy.SmrVector3(restrain(mpos.getX()) * 10, 0.0 , restrain(mpos.getY()) * 10 ))
      #self.tentacleBase.setR (restrain(mpos.getX()) * 20)
      self.teapot.setPos(restrain(mpos.getX()) * 10, 0.0 , restrain(mpos.getY()) * 10 )
    self.myKinematicSolver.process()
    self.updatePandaSkeleton(self.tentacle, self.tentacleIK, 'Bone')
    return Task.cont

  def createKinematicChain(self, _tentacle, _initialJointName, _parentName, _weight):
    #get the tentacle's currentJoint
    currentPandaJoint = _tentacle.getJoints(_initialJointName)
    currentPandaCJoint = self.tentacle.controlJoint(None, 'modelRoot', _initialJointName)
    #get the first joint's position
    position = currentPandaCJoint.getPos()

    if (currentPandaJoint[0].getNumChildren() == 0):
      newJoint = SMRPy.SmrKinematicJoint(True)
      newJoint.setEndVect(position.getX(),position.getY(),position.getZ());
    else:  
      newJoint = SMRPy.SmrKinematicJoint(False)

    newJoint.setPos(position.getX(),position.getY(),position.getZ())
    print "position", position.getX(), position.getY(), position.getZ()
    #newJoint.setRot(0,0,0)
    newJoint.setParentName(_parentName)
    newJoint.setName(_initialJointName)
    
    rotZ = (currentPandaCJoint.getH()/180.0)*3.14159;
    rotX = (currentPandaCJoint.getP()/180.0)*3.14159;
    rotY = (currentPandaCJoint.getR()/180.0)*3.14159;

    print rotX, rotY, rotZ, currentPandaCJoint.getName()
    # remove the setRot function veveal in C++, error prone

    newJoint.addDof(SMRPy.SmrVector3(0.0,0.0,1.0), rotZ - 1.0, rotZ + 1.0, rotZ, 0.01*_weight);
    newJoint.addDof(SMRPy.SmrVector3(1.0,0.0,0.0), rotX - 1.0, rotX + 1.0, rotX, 0.01*_weight);
    newJoint.addDof(SMRPy.SmrVector3(0.0,1.0,0.0), rotY, rotY, rotY, 0);

    _weight += 0.5


    print newJoint.getName()
    print _initialJointName, 'numchildren : ', currentPandaJoint[0].getNumChildren()

    self.jointList.append(newJoint)
    self.tentacleIK.insertJoint(newJoint)
    for i in range(currentPandaJoint[0].getNumChildren()):
      childJoint = currentPandaJoint[0].getChild(i)
      childName = childJoint.getName()
      print(childName)
      self.createKinematicChain(_tentacle, childName, _initialJointName, _weight)


  def updatePandaSkeleton(self, _pandaSkeleton, _smrSkeleton, _startJointName):
    currentPandaJoint =  _pandaSkeleton.getJoints(_startJointName)
    currentPandaCJoint = _pandaSkeleton.controlJoint(None, 'modelRoot', _startJointName)
    currentSmrJoint = _smrSkeleton.getJointByName(_startJointName)

    self.synchronize(currentPandaCJoint, currentSmrJoint)

    for i in range(currentPandaJoint[0].getNumChildren()):
      childJoint = currentPandaJoint[0].getChild(i)
      childName = childJoint.getName()
      self.updatePandaSkeleton(_pandaSkeleton, _smrSkeleton, childName)

  def synchronize(self, _pandaCJoint, _smrJoint):
    #_smrJoint.setRot(3.14/20.0,0,0);
    smrQuaternion = _smrJoint.getRot()
    pandaQuaternion = Quat()
    pandaQuaternion.setI(smrQuaternion.getX())
    pandaQuaternion.setJ(smrQuaternion.getY())
    pandaQuaternion.setK(smrQuaternion.getZ())
    pandaQuaternion.setR(smrQuaternion.getW())
    _pandaCJoint.setQuat(pandaQuaternion)

#  def turnTentacle(self, task):
    #Check to make sure the mouse is readable
#    if base.mouseWatcherNode.hasMouse():
      #get the mouse position as a Vec2. The values for each axis are from -1 to
      #1. The top-left is (-1,-1), the bottom rightis (1,1)
#      mpos = base.mouseWatcherNode.getMouse()
      #Here we multiply the values to get the amount of degrees to turn
      #Restrain is used to make sure the values returned by getMouse are in the
      #valid range. If this particular model were to turn more than this,
      #significant tearing would be visable
      #self.tentacleBase.setP(restrain(mpos.getY()) * -50)
#      self.tentacleBase.setR (restrain(mpos.getX()) * 20)
#    return Task.cont                        #Task continues infinitely


  def setupLights(self):                    #Sets up some default lighting
    lAttrib = LightAttrib.makeAllOff()
    ambientLight = AmbientLight( "ambientLight" )
    ambientLight.setColor( Vec4(.4, .4, .35, 1) )
    lAttrib = lAttrib.addLight( ambientLight )
    directionalLight = DirectionalLight( "directionalLight" )
    directionalLight.setDirection( Vec3( 0, 8, -2.5 ) )
    directionalLight.setColor( Vec4( 0.9, 0.8, 0.9, 1 ) )
    lAttrib = lAttrib.addLight( directionalLight )
    render.attachNewNode( directionalLight.upcastToPandaNode() ) 
    render.attachNewNode( ambientLight.upcastToPandaNode() ) 
    render.node().setAttrib( lAttrib )


w = World()        #Create an instance of our class
run()              #Run the simulation

