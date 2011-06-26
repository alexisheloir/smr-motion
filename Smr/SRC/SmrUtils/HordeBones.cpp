#include "HordeBones.h"
#include "SmrUtils.h"

HordeBones::HordeBones()
{
}

HordeBones::~HordeBones()
{
  this->clear();
}

void HordeBones::clear()
{
  //if (!m_shared) 
  for_each( m_bones.begin(), m_bones.end(), DeleteObjectPtr());
  m_bones.clear();
  m_chain = NULL;
  this->freeResource();
}

void HordeBones::freeResource()
{
  m_boneRes.free();
}

void HordeBones::loadResource()
{
  //bone
  m_boneRes.load("models/bone/bone.scene.xml");
}

void HordeBones::create(SMRKinematicChain *chain, float scale)
{
  //set the chain for later update
  m_chain=chain;

  //prepare variables
  H3DNode previousBone = NULL;

  //go through all joints of the kinematic chain and create bones between joint and parent joint
  for(unsigned int i=0;i<chain->getNumJoints();i++)
  {
    //get the current joint in the kinematic chain
    SMRKinematicJoint* kinJointSPtr = chain->getJoint(i); 
    SMRJoint* jointPtr = static_cast<SMRJoint*>(kinJointSPtr);
    //int* referenceCounter = kinJointSPtr.getReferenceCounter();
    SMRJoint* currentJoint = (jointPtr);

    //check if the joint has a parent joint and get it
    SMRJoint* parentJoint;
    SMRKinematicJoint* kinParentJointSPtr = chain->getJointByName(currentJoint->getParentName());
    SMRJoint* parentJointPtr = static_cast<SMRJoint*>(kinParentJointSPtr);
    if (currentJoint->hasParent()) parentJoint = parentJointPtr;
    //else parentJoint = NULL;

    //if the joint has no children skip it
    //if(chain->hasJointChildren(i)==false) continue;

    //retrieve the relative position of the current joint
    SMRVector3 position(0.0,0.0,0.0);
    position = currentJoint->getPosition();

    //create a new horde bone
    HordeBone *bone=new(HordeBone);

    //create the bone parented or unparented and add it to the bone list
    m_bones.push_back(bone);
    if (previousBone)
     bone->node.create(m_boneRes, "joint"+i,previousBone);
    else
      bone->node.create(m_boneRes, "joint"+i);

    //set transformation and scale
    bone->node.setTransform((float)position.X(),(float)position.Y(),(float)position.Z(),0,0,0,1,1,1);
    bone->node.setScale(scale,scale,scale);
    bone->jointIndex=i;
    previousBone = bone->node.getHandle();
  }
}

void HordeBones::create(SMRSkeleton *skeleton, float scale)
{
  //prepare variables
  H3DNode parentBone = NULL;

  HordeNode rootNode;
  rootNode.create("root");
  rootNode.setTransform(0,0,0,0,0,0,scale,scale,scale);//setScale(scale,scale,scale);

  //go through all joints of the kinematic skeleton and create bones between joint and parent joint
  for(unsigned int i=0;i<skeleton->getNumJoints();i++)
  {
    //get the current joint in the kinematic skeleton
    SMRJoint* currentJoint(skeleton->getJoint(i));

    //check if the joint has a parent joint and get it
    SMRJoint* parentJoint;
    if (currentJoint->hasParent())
    {
      //get parent joint
      parentJoint=skeleton->getJointByName(currentJoint->getParentName());

      //get parent bone
      for(unsigned int j=0;j<m_bones.size();j++)
      {
        HordeBone *curBone=m_bones[j];
        if(curBone->name==currentJoint->getParentName())
        {
          //get Horde3D node handle
          parentBone=curBone->node.getHandle();
          break;
        }
      }
    }
    //else
    //{
    //  //no parent bone/joint
    //  parentJoint = NULL;
    //  parentBone=NULL;
    //}

    //retrieve the relative position of the current joint
    SMRVector3 position(0.0,0.0,0.0);
    position = currentJoint->getPosition();

    //create bone
    HordeBone *bone=new(HordeBone);
    bone->name=currentJoint->getName();

    //create bone parented or unparented
    m_bones.push_back(bone);
    if (parentBone)
      bone->node.create("joint_"+i,parentBone);
    else
      bone->node.create("joint_"+i,rootNode.getHandle());

    //check if the joint has children
    if(skeleton->hasJointChildren(i))
    {
      //get a vector with all child joints
      vector<unsigned int> children=skeleton->getJointChildren(i);

      //create bones for all child joints
      for(unsigned int j=0;j<children.size();j++)
      {
        SMRJoint* childJoint(skeleton->getJoint(children[j]));

        SMRVector3 childPosition=childJoint->getPosition();
        SMRVector3 boneDisplayVector(0,1,0);

        //compute the bone length
        float displayLength=7.0f;
        float boneLength=static_cast<float>(childPosition.norm())/displayLength;

        //normalize the position for axis/angle computations
        childPosition.normalize();

        //compute the axis/angle rotation between the position vector and display bone vector in order to orient the visual representation correctly
        SMRVector3 boneAxis=CrossProduct(boneDisplayVector,childPosition);
        float dp=static_cast<float>(DotProduct(boneDisplayVector,childPosition));
        float boneAngle=acosf(dp);

        //convert the axis/angle rotation into euler angles
        double boneRrx,boneRry,boneRrz;
        SMRQuaternion boneRotation(boneAxis,boneAngle);
        boneRotation.toEulerAnglesYXZ(boneRrx,boneRry,boneRrz);

        //create bone horde node and attach it to joint node
        HordeNode *newChildNode=new(HordeNode);//is added to child node list
        bone->childNodes.push_back(newChildNode);

        newChildNode->create(m_boneRes,"joint_bone_"+childJoint->getName(),bone->node.getHandle());
        newChildNode->setTransform(0,0,0,SMRUtils::radToDeg(((float)(boneRrx))),
                                SMRUtils::radToDeg(((float)(boneRry))),SMRUtils::radToDeg(((float)(boneRrz))),boneLength,boneLength,boneLength);
      }
    }

    //convert the joint rotation to euler angles
    double rrx,rry,rrz;
    SMRQuaternion rotation=currentJoint->getOrientation();
    rotation.toEulerAnglesYXZ(rrx,rry,rrz);

    //set joint node transformation
    bone->node.setTransform((float)position.X(),(float)position.Y(),(float)position.Z(),SMRUtils::radToDeg(((float)(rrx))),
                                  SMRUtils::radToDeg(((float)(rry))),SMRUtils::radToDeg(((float)(rrz))),1,1,1);
    //bone->node.setScale(scale,scale,scale);
    bone->jointIndex=i;
  }
}

void HordeBones::update()
{
  double rrx,rry,rrz;
  for(unsigned int i=0;i<m_bones.size();i++)
  {
    HordeBone *currentBone=m_bones[i];
    SMRKinematicJoint* currentJoint=m_chain->getJoint(currentBone->jointIndex);

    //get position and rotation
    SMRVector3 position=currentJoint->getPosition();
    SMRQuaternion rotation=currentJoint->getOrientation();
    rotation.toEulerAnglesYXZ(rrx,rry,rrz);

    //set transformation
    currentBone->node.setTransform((float)position.X(),(float)position.Y(),(float)position.Z(),SMRUtils::radToDeg(((float)(rrx))),
                                  SMRUtils::radToDeg(((float)(rry))),SMRUtils::radToDeg(((float)(rrz))));
  }
}

void HordeBones::update(SMRSkeleton *skeleton)
{
  double rrx,rry,rrz;
  for(unsigned int i=0;i<m_bones.size();i++)
  {
    HordeBone *currentBone=m_bones[i];
    SMRJoint* currentJoint=skeleton->getJoint(currentBone->jointIndex);

    //get position and rotation
    SMRVector3 position=currentJoint->getPosition();
    SMRQuaternion rotation=currentJoint->getOrientation();
    rotation.toEulerAnglesYXZ(rrx,rry,rrz);

    //set transformation
    currentBone->node.setTransform((float)position.X(),(float)position.Y(),(float)position.Z(),SMRUtils::radToDeg(((float)(rrx))),
                                  SMRUtils::radToDeg(((float)(rry))),SMRUtils::radToDeg(((float)(rrz))));
  }
}
