#include "SmrLoader.h"

void relativize(SMRSkeleton &bindPose);
void nullRot(SMRSkeleton &bindPose );

/*------------------------------------------------------------------*/
/*  Function:  Load Skeleton From Acclaim              */
/*------------------------------------------------------------------*/

SMRSkeleton loadSkeletonFromAcclaim( string filename )
{

  double scale = (1.0/0.45)*2.54/100.0;
  scale = 1;

  //  Prepare variables
  vector<string>  dofVect;
  vector<string>  parentStack;

  //  defines a stream to be read from
  ifstream skelfile( filename.c_str() );

  //  a new skeleton to be constructed
  SMRSkeleton bindPose( RELATIVEMODE, ROTATIONFIRST );

  //  rotations will re relative
  bindPose.setMode( RELATIVEMODE );

  //  parse root infos
  expect( ":root", skelfile );

  //  push root onto stack
  dofVect.push_back( "root+tx" );
  dofVect.push_back( "root+ty" );
  dofVect.push_back( "root+tz" );
  dofVect.push_back( "root+rx" );
  dofVect.push_back( "root+ry" );
  dofVect.push_back( "root+rz" );

  //  instanciate root joint
  SMRJoint* currentJoint(new SMRJoint);
  currentJoint->setName( "root" );
  currentJoint->setParentName( "" );
  currentJoint->setPosition( SMRVector3( 0.0f, 0.0f, 0.0f ) );

  SMRQuaternion id;
  id.identity();
  currentJoint->setOrientation( id );
  bindPose.insertJoint( currentJoint );

  //  parse following joints
  expect( ":bonedata", skelfile );
  string nextToken( "" );

  do
  {
    expect( "name", skelfile );
    string nodeName;
    skelfile >> nodeName;
    //cout << nodeName << endl;

    //  just a little bit of trigo, in order to convert a length/axis to absolute cartesian coordinates.
    double x,y,z,length;
    double lX,lY,lZ;//,hyp;

    expect( "direction", skelfile );

    skelfile >> x;
    skelfile >> y;
    skelfile >> z;

    expect( "length", skelfile );

    skelfile >> length;
    length = length * scale;

    expect( "axis",skelfile );
    double a1,a2,a3;
    string axisOrder;

    skelfile >> a1;
    skelfile >> a2;
    skelfile >> a3;

    skelfile >> axisOrder;

    //orientation axis (absolute )
    SMRQuaternion orientation;
    orientation.fromEulerAngles(a1/180.0*M_PI,a2/180.0*M_PI,a3/180.0*M_PI);

    orientation.normalize();

    lX = length * x;
    lY = length * y;
    lZ = length * z;
    SMRVector3 pos(lX,lY,lZ);
    SMRJoint* currentJoint(new SMRJoint);
    currentJoint->setName(nodeName );

    currentJoint->setPosition(pos);
    currentJoint->setOrientation((orientation));
    bindPose.insertJoint(currentJoint);

    skelfile >> nextToken;
    if (nextToken != "end" )
    {
      char str[2048];
      skelfile.getline(str, 2048);
      string dofType;
      stringstream dofStream(str,stringstream::in);
      while(dofStream >> dofType )
      {
        dofVect.push_back(nodeName + "+" + dofType );
      }
      expect( "end",skelfile );
      skelfile >> nextToken;
    }
    else
      skelfile >> nextToken;
  }
  while (nextToken != ":hierarchy" );

  expect( "begin",skelfile );
  char str[2048];
  skelfile.getline(str, 2048);
  string parentJointName;
  skelfile >> parentJointName;

  while(parentJointName != "end" )
  {
    skelfile.getline(str, 2048);
    stringstream jointStream(str,stringstream::in);
    string childJoint;
    while(jointStream >> childJoint)
    {
      currentJoint = bindPose.getJointByName( childJoint );
      //SMRJoint* parentJoint = bindPose.getJointByName( parentJointName );
      currentJoint->setParentName( parentJointName );
      //cout << childJoint << ">>" << parentJointName << endl;
    }
    skelfile >> parentJointName;
  }

  bindPose.checkEndJoints();
  bindPose.setMode(RELATIVEMODE);
  bindPose.setRotationOrder(TRANSLATIONFIRST);
  bindPose.setRotationOrder(ROTATIONFIRST);

  relativize(bindPose);
  skelfile.close();
  return bindPose;
}

/*------------------------------------------------------------------*/
/*  Function:  Load Motion From Acclaim              */
/*------------------------------------------------------------------*/

SMRMotion loadMotionFromAcclaim(string _skelfilename, string _motionfilename )
{

  double scale = (1.0/0.45)*2.54/100.0;
  scale = 1.0;

  //this vector stores each joint DOF : "jointName+DOFtype
  //where DOFtype is of
  //tx
  //ty
  //tz
  //rx
  //ry
  //rz
  vector<string> dofVect;

  //cout << "Loading skeleton from file : " << _skelfilename << endl;
  //  defines a stream to be read from
  ifstream skelfile(_skelfilename.c_str());
  if (skelfile.fail())
  {
    cout << "file " +  _skelfilename + " not found" << endl;
    exit(1);
  }
  //  a new skeleton to be constructed
  SMRSkeleton  bindPose(RELATIVEMODE,ROTATIONFIRST);
  //  rotations will re relative
  bindPose.setMode(RELATIVEMODE);
  //  a stack of articulation names
  vector<string> parentStack;
  //  parse root infos
  expect( ":root", skelfile );
  //push root onto stack
  dofVect.push_back( "root+tx" ); // we are here feeding each joint's DOF
  dofVect.push_back( "root+ty" );
  dofVect.push_back( "root+tz" );
  dofVect.push_back( "root+rx" );
  dofVect.push_back( "root+ry" );
  dofVect.push_back( "root+rz" );

  //instanciate root joint
  SMRJoint* currentJoint(new SMRJoint);
  currentJoint->setName( "root" );
  currentJoint->setParentName( "" );
  currentJoint->setPosition(SMRVector3( 0.0f, 0.0f, 0.0f ));
  SMRQuaternion id;
  id.identity();
  currentJoint->setOrientation(id);
  bindPose.insertJoint(currentJoint);

  //  parse following joints
  expect( ":bonedata", skelfile );
  string nextToken( "" );

  do
  {
    expect( "name", skelfile );
    string nodeName;
    skelfile >> nodeName;
    //cout << nodeName << endl;

    /*
    * just a little bit of trigo, in order to convert a length/axis to absolute cartesian coordinates.
    */

    double x,y,z,length;
    double lX,lY,lZ;//,hyp;

    expect( "direction", skelfile );

    skelfile >> x;
    skelfile >> y;
    skelfile >> z;

    expect( "length", skelfile );

    skelfile >> length;

    length *= scale;

    expect( "axis",skelfile );
    double a1,a2,a3;
    string axisOrder;

    skelfile >> a1;
    skelfile >> a2;
    skelfile >> a3;

    skelfile >> axisOrder;

    //orientation axis (absolute )
    SMRQuaternion orientation;
    orientation.fromEulerAngles(a1/180.0*M_PI,a2/180.0*M_PI,a3/180.0*M_PI);

    orientation.normalize();

    lX = length * x;
    lY = length * y;
    lZ = length * z;
    SMRVector3 pos(lX,lY,lZ);

    //SMRQuaternion inv = Inverse(orientation);
    //inv.rotate(pos);

    SMRJoint* currentJoint(new SMRJoint);
    currentJoint->setName(nodeName );

    currentJoint->setPosition(pos);
    currentJoint->setOrientation((orientation));
    bindPose.insertJoint(currentJoint);

    skelfile >> nextToken;
    //cout << nextToken << endl;
    if (nextToken != "end" )
    {
      char str[2048];
      skelfile.getline(str, 2048);
      string dofType;
      stringstream dofStream(str,stringstream::in);
      while(dofStream >> dofType )
      {
        //cout << nodeName + "+" + dofType << endl;
        dofVect.push_back(nodeName + "+" + dofType ); //and we are here feeding each joint's DOF
      }
      expect( "end",skelfile );
      skelfile >> nextToken;
    }
    else
      skelfile >> nextToken;
  }
  while (nextToken != ":hierarchy" );

  //  cout << "end" << endl;
  expect( "begin",skelfile );
  char str[2048];
  skelfile.getline(str, 2048);
  string parentJointName;
  skelfile >> parentJointName;
  //cout << parentJointName << endl;

  while(parentJointName != "end" )
  {
    skelfile.getline(str, 2048);
    stringstream jointStream(str,stringstream::in);
    string childJoint;
    while(jointStream >> childJoint)
    {
      currentJoint = bindPose.getJointByName( childJoint );
      //SMRJoint* parentJoint = bindPose.getJointByName( parentJointName );
      currentJoint->setParentName( parentJointName );
      //cout << childJoint << ">>" << parentJointName << endl;
    }
    skelfile >> parentJointName;
  }

  bindPose.setMode(RELATIVEMODE);

  bindPose.checkEndJoints();
  bindPose.setMode(RELATIVEMODE);
  bindPose.setRotationOrder(TRANSLATIONFIRST);
  bindPose.setRotationOrder(ROTATIONFIRST);

  relativize(bindPose);

  SMRSkeleton refPose  = bindPose;
  SMRSkeleton nullPose = bindPose;
  nullRot(nullPose ); // set each joint orientation to 0
  SMRSkeleton skeleton = nullPose;
  skelfile.close();
  SMRMotion motion; // instanciate the motion we will feed with motion data

/**************************************************************************
                     Reading the motion from here
**************************************************************************/

  //  defines a stream to be read from
  ifstream motionfile(_motionfilename.c_str());
  if (motionfile.fail())
  {
    cout << "file " +  _motionfilename + " not found" << endl;
    exit(1);
  }
  string dofIterator;
  int nframes = 1;
  string jointName;
  
  expect( "1",motionfile );
  motionfile.getline(str, 2048);
  do
  {
    if (!motionfile.getline(str, 2048)) break; // exit loop when EOF reached
    stringstream valueStream(str,stringstream::in);
    valueStream >> jointName ;
    char * pEnd;
    if ( strtol(jointName.c_str(),&pEnd,0) == nframes+1 ) // frame number
    {
      nframes++;
       
      skeleton.checkEndJoints();
      motion.insertSkeleton( skeleton );
      //motion.insertSkeleton( refPose );
      skeleton = nullPose;
    }
    else // joint rotation information
    {
      currentJoint = skeleton.getJointByName( jointName ); // JOINT and DOF information (skeleton has been instanciated)
      int jointIndex = -1;
      int rank = 0;

      // find the DOF information relative to the parsed joint in the DOF vector
      vector<string>::iterator dofit;
      for (dofit = dofVect.begin() ; dofit < dofVect.end() ; dofit++)
      {
        if (static_cast<string>(*dofit).find(jointName ) != string::npos)
        {
          jointIndex = rank; 
          break;
        }
        rank++;
      }
      
      // get the next value, have to figure out which DOF it represents (check this in the DOF vectro)
      double nextValue;
      int dofInd = 0;
      while (valueStream >> nextValue )
      {
        dofIterator = dofVect.at(rank + dofInd); // this tells us which kind of DOF it is
        int npos = static_cast<int>((dofIterator).find('+',0)); // split the "jointname+doftype"
        string jointName = (dofIterator).substr( 0,npos); // jointname
        string dofType = (dofIterator).substr(npos+1); // doftype

        currentJoint = skeleton.getJointByName(jointName );

        dofInd++;

        if(dofType == "tx" ) // update the rotation according to doftype
        {
          SMRVector3 pos = currentJoint->getPosition();
          nextValue *= scale;
          currentJoint->setPosition( nextValue, pos.Y(), pos.Z() );
        }
        else if(dofType == "ty" ) // update the rotation according to doftype
        {
          SMRVector3 pos = currentJoint->getPosition();
          nextValue *= scale;
          currentJoint->setPosition( pos.X(), nextValue, pos.Z() );
        }
        else if(dofType == "tz" ) // update the rotation according to doftype
        {
          SMRVector3 pos = currentJoint->getPosition();
          nextValue *= scale;
          currentJoint->setPosition( pos.X(), pos.Y(), nextValue );
        }
        else if(dofType == "rx" ) // update the rotation according to doftype
        {
          SMRQuaternion quat = currentJoint->getOrientation();
          SMRVector3 axis(1.0,0.0,0.0);
          SMRQuaternion rot(axis,nextValue/180.0*M_PI);
          //if (nframes < 3) cout << jointName << " rx " << nextValue << endl;
          currentJoint->setOrientation( rot*quat );
        }
        else if(dofType == "ry" ) // update the rotation according to doftype
        {
          SMRQuaternion quat = currentJoint->getOrientation();
          SMRVector3 axis( 0.0,1.0,0.0);
          SMRQuaternion rot(axis,nextValue/180.0*M_PI);
          //if (nframes < 3) cout << jointName << " ry " << nextValue << endl;
          currentJoint->setOrientation( rot*quat );
        }
        else if(dofType == "rz" ) // update the rotation according to doftype
        {
          SMRQuaternion quat = currentJoint->getOrientation();
          SMRVector3 axis( 0.0,0.0,1.0);
          SMRQuaternion rot(axis,nextValue/180.0*M_PI);
          //if (nframes < 3) cout << jointName << " rz " << nextValue << endl;
          currentJoint->setOrientation( rot*quat );
          //SMRQuaternion testOrient;
          //testOrient.fromEulerAngles(10.0/180.0*M_PI,20.0/180.0*M_PI,30.0/180.0*M_PI);
          //currentJoint->setOrientation(testOrient);
          double x,y,z;
          currentJoint->getOrientation().toEulerAngles(x,y,z);
          //cout << x*180.0/M_PI<< " " << y*180.0/M_PI << " " << z*180.0/M_PI << endl;
        }

        if(jointName == "root" )
        {
          //currentJoint->setOrientation( currentBindOrientation);
        }
      }

      if(jointName != "" ) // apply the bindPose orientation
      {
        SMRQuaternion currentOrientation = currentJoint->getOrientation();
        SMRQuaternion currentBindOrientation = bindPose.getJointByName(jointName)->getOrientation();
        SMRQuaternion currentBindOrientationInv = Inverse(currentBindOrientation);
        currentJoint->setOrientation(currentBindOrientation * currentOrientation);
        //currentJoint->setOrientation( currentBindOrientation);
        //SMRQuaternion rot1 = currentJoint->getOrientation();
        //SMRVector3 p;
        //p = currentJoint->getPosition();
        //rot1.rotate(p);
        //currentJoint->setPosition(p);
      }
    }
  }
  while(true );
  motionfile.close();
  //cout << "nb frames" << motion.getNumFrames() << endl;
  motion.setTimeStep( 0.01);
  return motion;
}

/*------------------------------------------------------------------*/
/*  Function:  Relativize                      */
/*------------------------------------------------------------------*/

/**
 *  This funtion takes as argument two skeletons : the skeleton that has been
 *  obtained by parsing the motion data AMC file (currentPose) and the reference skeleton that is
 *  represented in the ASF file (bindPose)
 *  rotations in AMC file are expressed according to the reference skeleton joints local frames.
 *  in order to have a consistent skeleton for SMR library, it is necessary to express parsed skeleton joints rotations
 *  according to the reference joint parents local frames 
 *
 *   bindPose         currentPose before             currentPose after
 *      * Ref0          * R0/Ref0                       *R0/World
 *      |               |                               |
 *      * Ref1          * R1/Ref1                       * R1/R0
 *      |               |                               |
 *      * Ref2          * R2/Ref2                       * R2/R1
 *      |               |                               |
 */


void relativize(SMRSkeleton &bindPose )
{
  for(int i=bindPose.getNumJoints()-1;i>=0;--i)
  {
    SMRJoint* currentJoint=bindPose.getJoint(i);                                       // get joint in currentPose

    SMRJoint* bindJoint=bindPose.getJointByName( currentJoint->getName() );             // get joint in binPose
    SMRQuaternion bindOrientation=bindJoint->getOrientation();                          // get bindJoint orientation

    SMRVector3 currentPosition = currentJoint->getPosition();
    SMRQuaternion inverseBindOrientation=Inverse(bindOrientation);
    inverseBindOrientation.rotate(currentPosition);                                     // 
    currentJoint->setPosition(currentPosition); 

    if(currentJoint->hasParent())
    {
      SMRJoint* parentBindJoint=bindPose.getJointByName( currentJoint->getParentName() ); // get Parentjoint in bindPose      
      SMRQuaternion parentBindOrientation=parentBindJoint->getOrientation();              // get parentBindJoint orientation

      SMRQuaternion inverseParentBindOrientation=Inverse(parentBindOrientation);
      SMRQuaternion relativeOrientation=inverseParentBindOrientation*bindOrientation; 
      currentJoint->setOrientation(relativeOrientation);
    }

  }
}

void nullRot(SMRSkeleton &bindPose )
{
  /************************************************************************/
  //  switch to relative
  SMRQuaternion id;
  id.identity();
  for( int i = bindPose.getNumJoints() - 1 ; i > 0; --i )
  {
    SMRJoint*joint(bindPose.getJoint(i));
    joint->setOrientation(id);
  }
}

/*------------------------------------------------------------------*/
/*  End of file                            */
/*------------------------------------------------------------------*/