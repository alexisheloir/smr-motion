# Author: Alexis Heloir
# Models and Textures by: Alexis Heloir
# Last Updated: 12/08/08
#
import direct.directbase.DirectStart
import SMRPy
from pandac.PandaModules import AmbientLight,DirectionalLight
from pandac.PandaModules import TextNode,NodePath,LightAttrib
from pandac.PandaModules import Vec3,Vec4
from pandac.PandaModules import Quat,Mat4
from direct.actor.Actor import Actor
from direct.task.Task import Task
from direct.gui.OnscreenText import OnscreenText
from direct.showbase.DirectObject import DirectObject
from pandac.PandaModules import Lens
import sys


#A simple function to make sure a value is in a given range, -1 to 1 by default
def restrain(i, mn = -1, mx = 1): return min(max(i, mn), mx)

tPosX = 0.0
tPosY = 0.0
tPosZ = 0.0

class World(DirectObject):
    def __init__(self):
        #This code puts the standard title and instuction text on screen
        self.title = OnscreenText(text="Panda3D: Embots - Joint Manipulation",
                              style=1, fg=(1,1,1,1),
                              pos=(0.8,-0.95), scale = .07)            
        #setup key input
        self.accept('escape', sys.exit)

        #base.disableMouse() #Disable mouse-based camera-control
        #camera.setPos(0,0,-40)              #Position the camera
        #camera.setHpr(0,90,0)
        
        #load the agent and parent it to the world 
        self.agent = Actor("models/agent")
        self.agent.reparentTo(render)
        self.agent.setScale(1)

        #load the teapot (representing the IK constraint)
        #to be parented later on
        self.teapot = loader.loadModel('models/teapot')
        self.teapot.setScale(0.2)

        #instanciate a list in order to keep track of kinematic joints joints 
        #in python runtime
        #if nothing points towards those joints, they get flushed by
        #python's garbage collector
        self.jointList = []

        #create the SMR skeleton (IK version of agent skeleton)
        self.agentSMRIK = SMRPy.SmrKinematicChain(True,True,'agent')
        self.createKinematicChain(self.agent, self.agentSMRIK, 'rclavicle', '', 0)
        #expose the beginning of the kinematic chain in order to 
        #reparent the kinematic cosntraint (teapot) 
        rclavicleJoint = self.agent.exposeJoint(None,"modelRoot","spine4")
        self.teapot.reparentTo(rclavicleJoint)

        #get a reference towards the end of the kinematic chain to be created
        myJoint = self.agentSMRIK.getJointByName("rhand")        
        
        # create one constraint (will be graphicaly represented by a teapot)
        self.myConstraint = SMRPy.SmrIKConstraint()
        #attach a joint from which the distance contraint-joint 
        #will be calculated
        self.myConstraint.setRelatedJointptr(myJoint)
        #in order to take into account skin layer, an offset 
        #may be specified on the constraint
        myConstraintOffset = SMRPy.SmrVector3(0.0,0.0,0.0)
        self.myConstraint.setOffset(myConstraintOffset)

        #create the kinematic solver and bind the constaint to it
        self.myKinematicSolver = SMRPy.SmrGSMMSolver(self.agentSMRIK)
        self.myKinematicSolver.addConstraintPtr(self.myConstraint)

        self.accept('arrow_up', self.moveTargetUp )
        self.accept('arrow_down', self.moveTargetDown )
        self.accept('arrow_left', self.moveTargetLeft )
        self.accept('arrow_right', self.moveTargetRight )
        self.accept('page_up', self.moveTargetToward )
        self.accept('page_down', self.moveTargetForward )

        #let's rock ! (one IK step every tenth of a second)
        taskMgr.doMethodLater(0.1, self.ikStep, "ikStep")

#----------------------------------------------------------------------------#
#                      and the rest is litterature...                        #
#----------------------------------------------------------------------------#

    def moveTargetUp(self):
        tPosX += 0.1
    def moveTargetDown(self):
        tPosX -= 0.1
    def moveTargetRight(self):
        tPosZ += 0.1
    def moveTargetLeft(self):
        tPosZ -= 0.1
    def moveTargetToward(self):
        tPosY += 0.1
    def moveTargetForward(self):
        tPosY -= 0.1


    def ikStep(self, task):
        if base.mouseWatcherNode.hasMouse():
            mpos = base.mouseWatcherNode.getMouse()
            self.myConstraint.setPosition(tPosX , tPosY , tPosZ)
            self.teapot.setPos(restrain(tPosX,tPosY,tPosZ))
        self.myKinematicSolver.process()
        self.updatePandaSkeleton(self.agent, self.agentSMRIK, 'rclavicle')
        return Task.again

      
        self.setupLights()

    def setupLights(self):
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

    def createKinematicChain(self, _pandaAgent, _smrIKSkel, _initialJointName, _parentName, _weight):
        #get the agent's currentJoint
        currentPandaJoint = _pandaAgent.getJoints(_initialJointName)
        currentPandaCJoint = _pandaAgent.controlJoint(None, 'modelRoot', _initialJointName)
        #get the first joint's position
        position = currentPandaCJoint.getPos()

        if (currentPandaJoint[0].getNumChildren() == 0):
            newJoint = SMRPy.SmrKinematicJoint(True)
            newJoint.setEndVect(position.getX(),position.getY(),position.getZ());
        else:  
            newJoint = SMRPy.SmrKinematicJoint(False)

        newJoint.setPos(position.getX(),position.getY(),position.getZ())
        #print "position", position.getX(), position.getY(), position.getZ()
        #newJoint.setRot(0,0,0)
        newJoint.setParentName(_parentName)
        newJoint.setName(_initialJointName)
    
        rotZ = (currentPandaCJoint.getH()/180.0)*3.14159;
        rotX = (currentPandaCJoint.getP()/180.0)*3.14159;
        rotY = (currentPandaCJoint.getR()/180.0)*3.14159;

        print rotX, rotY, rotZ, currentPandaCJoint.getName()
        # remove the setRot function reveal in C++, error prone

        newJoint.addDof(SMRPy.SmrVector3(0.0,0.0,1.0), rotZ - 1.0, rotZ + 1.0, rotZ, 0.05*_weight);
        newJoint.addDof(SMRPy.SmrVector3(1.0,0.0,0.0), rotX - 1.0, rotX + 1.0, rotX, 0.05*_weight);
        newJoint.addDof(SMRPy.SmrVector3(0.0,1.0,0.0), rotY, rotY, rotY, 0);

        _weight += 0.5


        #print newJoint.getName()
        print _initialJointName, 'numchildren : ', currentPandaJoint[0].getNumChildren()

        self.jointList.append(newJoint)
        _smrIKSkel.insertJoint(newJoint)
        for i in range(currentPandaJoint[0].getNumChildren()):
            childJoint = currentPandaJoint[0].getChild(i)
            childName = childJoint.getName()
            #print(childName)
            self.createKinematicChain(_pandaAgent, _smrIKSkel, childName, _initialJointName, _weight)

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

w = World()        #Create an instance of our class
run()              #Run the simulation
