/*
 *  SmrExporterAcclaim.cpp
 *  SmrIO
 */

#include "SmrExporter.h"

void absolutize(SMRSkeleton &bindPose )
{
  for(unsigned int i = 0; i < bindPose.getNumJoints(); ++i)
  {
    SMRJoint* currentBindJoint = bindPose.getJoint(i);
    {
      SMRQuaternion currentBindOrientation=currentBindJoint->getOrientation();
      if (currentBindJoint->hasParent())
      {
        SMRJoint* parentBindJoint = bindPose.getJointByName( currentBindJoint->getParentName() );
      
        SMRQuaternion parentBindOrientation=parentBindJoint->getOrientation();

        currentBindOrientation = parentBindOrientation * currentBindOrientation;
        currentBindJoint->setOrientation(currentBindOrientation);
      }
      SMRVector3 currentPosition=currentBindJoint->getPosition();
      currentBindOrientation=currentBindJoint->getOrientation();
      currentBindOrientation.rotate(currentPosition);
      currentBindJoint->setPosition(currentPosition); 
    }
  }
}

void ASFWriteHeader(ofstream& file)
{
  //# AST/ASF file generated using VICON BodyLanguage
  //# -----------------------------------------------
  //:version 1.10
  //:name VICON
  //:units
  //  mass 1.0
  //  length 0.45
  //  angle deg
  //:documentation
  //   .ast/.asf automatically generated from VICON data using
  //   VICON BodyBuilder and BodyLanguage model FoxedUp or BRILLIANT.MOD

  file<<"# ASF file generated using SMR IO library"<<endl;
  file<<"# ---------------------------------------"<<endl;
  file<<":version 1.0"<<endl;
  file<<":name SMR"<<endl;
  file<<":units"<<endl;
  file<<"  mass 1.0"<<endl;
  file<<"  length 1.0 (meter)"<<endl;
  file<<"  angle deg"<<endl;
  file<<":documentation"<<endl;
  file<<"  Generated with SMR IO library"<<endl;
}

void ASFWriteRoot(ofstream& file)
{
  //:root
  //   order TX TY TZ RX RY RZ
  //   axis XYZ
  //   position 0 0 0  
  //   orientation 0 0 0 

  file<<":root"<<endl;
  file<<"   order TX TY TZ RX RY RZ"<<endl;
  file<<"   axis XYZ"<<endl;
  file<<"   position 0 0 0  "<<endl;
  file<<"   orientation 0 0 0"<<endl;
}

void ASFWriteBoneDataBegin(ofstream& file,int ID,string Name,double DirectionX,double DirectionY,double DirectionZ,
  double Length,double AxisX,double AxisY,double AxisZ)
{
  //  begin
  //   id 1 
  //   name examplename
  //   direction 0.00498555 0.997467 -0.0709511   
  //   length 2.01085   
  //   axis 0 0 0   XYZ

  file<<"  begin"<<endl;
  file<<"     id "<<ID<<endl;
  file<<"     name "<<Name<<endl;
  file<<"     direction "<<DirectionX<<" "<<DirectionY<<" "<<DirectionZ<<" "<<endl;
  file<<"     length "<<Length<<endl;
  file<<"     axis "<<AxisX<<" "<<AxisY<<" "<<AxisZ<<" XYZ"<<endl;
}

void ASFWriteBoneDataDOF(ofstream& file,bool Rx,bool Ry,bool Rz,double XMin,double XMax,
  double YMin,double YMax,double ZMin,double ZMax)
{
  //  dof rx ry rz
  //  limits (-20.0 45.0)
  //       (-30.0 30.0)
  //       (-30.0 30.0)

  bool Indent = false;

  file<<"    dof ";

  if(Rx == true)
  {
    file<<"rx ";
  }

  if(Ry == true)
  {
    file<<"ry ";
  }

  if(Rz == true)
  {
    file<<"rz ";
  }

  file<<endl;

  file<<"    limits ";

  if(Rx == true)
  {
    if(Indent == true)
    {
      file<<"           ";
    }

    file<<"("<<XMin<<" "<<XMax<<")"<<endl;

    Indent = true;
  }

  if(Ry == true)
  {
    if(Indent == true)
    {
      file<<"           ";
    }

    file<<"("<<YMin<<" "<<YMax<<")"<<endl;

    Indent = true;
  }

  if(Rz == true)
  {
    if(Indent == true)
    {
      file<<"           ";
    }

    file<<"("<<ZMin<<" "<<ZMax<<")"<<endl;

    Indent = true;
  }
}

void ASFWriteBoneData(ofstream& file,SMRSkeleton& Skeleton)
{
  //:bonedata
  //  begin
  //   id 1 
  //   name examplename
  //   direction 0.00498555 0.997467 -0.0709511   
  //   length 2.01085   
  //   axis 0 0 0   XYZ
  //  dof rx ry rz
  //  limits (-20.0 45.0)
  //       (-30.0 30.0)
  //       (-30.0 30.0)
  //  end

  file<<":bonedata"<<endl;

  for(unsigned int i=1; i<Skeleton.getNumJoints(); i++)
  {
    SMRJoint* CurrentJoint;
    double      Length      = 0.0;
    SMRVector3    Position;
    SMRVector3    Direction;
    
    CurrentJoint = Skeleton.getJoint(i);
    if(CurrentJoint == NULL) return;

    Position = CurrentJoint->getPosition();
    Length = Position.norm();
    Direction = Position;
    Direction.normalize();

    //Get sure that Direction has a norm of 1
    if (Direction == SMRVector3(0.0,0.0,0.0))
    {
      Direction = SMRVector3(1.0,0.0,0.0);
    }
    //get sure that length is mot null
    if (Length == 0.0)
    {
      Length = 0.001;
    }

    double rotx,roty,rotz;
    SMRQuaternion Orientation;
    Orientation = CurrentJoint->getOrientation();
    Orientation.toEulerAngles(rotx,roty,rotz);

    rotx = rotx * 180.0 / M_PI;
    roty = roty * 180.0 / M_PI;
    rotz = rotz * 180.0 / M_PI;

    ASFWriteBoneDataBegin(file,i,CurrentJoint->getName(),Direction.X(),Direction.Y(),Direction.Z(),
                 Length,rotx,roty,rotz);

    ASFWriteBoneDataDOF(file,true,true,true,-360.0,360.0,-360.0,360.0,-360.0,360.0);

    file<<"  end"<<endl;
  }
}

void ASFWriteJoint(ofstream& file,const SMRJoint* Joint,SMRSkeleton& Skeleton,bool IsRoot)
{
  vector <unsigned int> Children;
  Children = Skeleton.getJointChildren(Joint->getName());
  if(Children.size() == 0) return;

  if(IsRoot == true)
  {
    file<<"    root";
  }
  else
  {
    file<<"    "<<Joint->getName();
  }

  for(unsigned int i=0; i<Children.size(); i++)
  {
    SMRJoint* CurrentChild;

    CurrentChild = Skeleton.getJoint(Children[i]);
    if(CurrentChild == NULL) return;

    file<<" "<<CurrentChild->getName();
  }

  file<<endl;

  for(unsigned int i=0; i<Children.size(); i++)
  {
    SMRJoint* CurrentChild;

    CurrentChild = Skeleton.getJoint(Children[i]);
    if(CurrentChild == NULL) return;

    ASFWriteJoint(file,CurrentChild,Skeleton,false);
  }
}

void ASFWriteHierarchy(ofstream& file,SMRSkeleton& Skeleton)
{
  //:hierarchy
  //  begin
  //root lhipjoint rhipjoint lowerback
  //lhipjoint lfemur
  //  end

  SMRJoint* RootJoint;

  file<<":hierarchy"<<endl;

  file<<"  begin"<<endl;

  RootJoint = Skeleton.getRootJoint();
  if(RootJoint == NULL) return;

  ASFWriteJoint(file,RootJoint,Skeleton,true);

  file<<"  end"<<endl;
}

void exportSkeletonToAcclaim(string fileName,const SMRSkeleton& _skeleton)
{
  SMRSkeleton skeleton = _skeleton;
  ofstream file(fileName.c_str());
  skeleton.setRotationOrder(ROTATIONFIRST);
  ASFWriteHeader(file);
  ASFWriteRoot(file);
  absolutize(skeleton);
  ASFWriteBoneData(file,skeleton);
  ASFWriteHierarchy(file,skeleton);
}

//acclaim motion capture file export

void writeFrame(ofstream& file,SMRSkeleton& bindPose,SMRSkeleton& frame)
{
  //absolutize(frame,bindPose);

  for(unsigned int i=0;i<frame.getNumJoints();i++)
  {
    SMRJoint* currentJoint = frame.getJoint(i);
    SMRJoint* currentBindJoint = bindPose.getJointByName(currentJoint->getName());

    SMRQuaternion currentRot = currentJoint->getOrientation();
    const SMRQuaternion bindRot = currentBindJoint->getOrientation();

    const SMRQuaternion invBindRot = Inverse(bindRot);

    currentRot = invBindRot * currentRot;

    //if (currentJoint->getName() == "rhumerus" || currentJoint->getName() == "lhumerus")
    //  currentJoint->setOrientation(Inverse(currentRot));

    //currentRot.identity();
    currentJoint->setOrientation(currentRot);

    if (currentJoint->hasParent())
      file<<currentJoint->getName();
    else
      file<<"root";

    if(i==0) // root joint
    {
      const SMRVector3 position=currentJoint->getPosition();
      file<<" "<<position.m_x<<" "<<position.m_y<<" "<<position.m_z;
    }

    double rotx,roty,rotz;
    SMRQuaternion orientation=currentJoint->getOrientation();
    orientation.toEulerAnglesZYX(rotx,roty,rotz);
    rotx= rotx*180/M_PI;
    roty= roty*180/M_PI;
    rotz= rotz*180/M_PI;
    file<<" "<<rotx<<" "<<roty<<" "<<rotz;

    file<<endl;
  }
}

void exportMotionToAcclaim(string fileName,SMRMotion& motion)
{
  ofstream file(fileName.c_str());
  file<<"#exported with SMR IO library"<<endl;
  
  SMRQuaternion identity;
  identity.identity();
  SMRSkeleton bindPose = motion.getSkeleton(0);
  for (unsigned int i = 0; i< bindPose.getNumJoints(); i++)
  {
    bindPose.getJoint(i)->setOrientation(identity);
  }

  bindPose.setRotationOrder(ROTATIONFIRST);

  for(unsigned int i=0;i<motion.getNumFrames();i++)
  {
    SMRSkeleton frame = motion.getSkeleton(i);
    frame.setRotationOrder(ROTATIONFIRST);
    file<<(i+1)<<endl;
    writeFrame(file,bindPose,frame);
  }
}

void writeOBJJoint(ofstream& file,string name,double x,double y,double z)
{
  file<<"g "<<name<<endl;
  file<<"v "<<x<<" "<<y<<" "<<z<<endl;
}

void writeOBJLine(ofstream& file,int childIndex,int parentIndex)
{
  file<<"l "<<(childIndex+1)<<" "<<(parentIndex+1)<<endl;
}

void dumpSkeletonOBJ(string fileName,SMRSkeleton& skeleton)
{
  ofstream file(fileName.c_str());
  
  skeleton.setMode(ABSOLUTEMODE);

  for(unsigned int i=0;i<skeleton.getNumJoints();i++)
  {
    SMRJoint* currentJoint=skeleton.getJoint(i);
    if(!currentJoint)return;
    SMRVector3 position=currentJoint->getPosition();
    string name=currentJoint->getName();
    if(currentJoint->hasParent())
    {
      string parentName=currentJoint->getParentName();
      int parentIndex=skeleton.getJointIndex(parentName);
      writeOBJLine(file,i,parentIndex);
    }
    writeOBJJoint(file,name,position.X(),position.Y(),position.Z());
  }

//  file<<"l 1 2"<<endl;
}