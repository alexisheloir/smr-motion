/*
*  SmrLoader.cpp
*  SmrCore
*
*  Created by Alexis Heloir/Nicolas Courty on 18/10/05.
*  Copyright 2005 __MyCompanyName__. All rights reserved.
*
*/

#pragma warning( disable : 4996 )

#include "SmrLoader.h"
#include <libxml/xmlreader.h>

//double hexToDouble(char* doubleAsHex)
//{
//  unsigned long long l,ll,m,e;
//  double returnValue=0,mantisse=0;
//
//  l = (unsigned long long)doubleAsHex[7];
//  l = l << 56;
//  ll = (unsigned long)doubleAsHex[6];
//  l = l | (ll << 48);
//  ll = (unsigned long)doubleAsHex[5];
//  l = l | (ll << 40);
//  ll = (unsigned long)doubleAsHex[4];
//  l = l | (ll << 32);
//  ll = (unsigned long)doubleAsHex[3];
//  l = l | (ll << 24);
//  ll = (unsigned long)doubleAsHex[2];
//  l = l | (ll << 16);
//  ll = (unsigned long)doubleAsHex[1];
//  l = l | (ll << 8);
//  ll = (unsigned long)doubleAsHex[0];
//  l = l | (ll);
//
//  //get the exponent
//  e=l&0x7ff0000000000000;
//  e>>=53;
//
//  m=l&0x000fffffffffffff;
//
//  for(int i=0; i<=53;i++)
//  {
//    unsigned long long powerOfTwo = unsigned long long(ldexp(1.0,i));
//    if( ( m & powerOfTwo ) == powerOfTwo )  
//    {
//      mantisse+=double((ldexp(1.0, i-53)));
//    }
//  }
//  mantisse+=1.0;
//
//  returnValue=double(ldexp(mantisse, e-1023.0));
//
//  char isNegative = doubleAsHex[7] & 0x8;
//  if (isNegative) returnValue = - returnValue;
//
//  return returnValue;
//}

//float hexToFloat(unsigned char byte1, unsigned char byte2, unsigned char byte3, unsigned char byte4)
//{
//  unsigned long l,ll,m,e;
//  float returnvalue=0,mantisse=0;
//
//  // feed the bit array (32 bits)
//  l = (unsigned long)byte1;
//  l = l << 24;
//  ll = (unsigned long)byte2;
//  l = l | (ll << 16);
//  ll = (unsigned long)byte3;
//  l = l | (ll << 8);
//  ll = (unsigned long)byte4;
//  l = l | ll;
//
//  //get the exponent
//  e=l&0x7f800000;
//  e>>=23;
//  //get the mantisse
//  m=l&0x007fffff;
//
//  //denormalize
//
//
//  for(int i=0; i<=23;i++)
//  {
//    long powerOfTwo = unsigned long(ldexp(1.0f,i));
//    if( ( m & powerOfTwo ) == powerOfTwo )  
//    {
//      mantisse+=float((ldexp(1.0f, i-23)));
//    }
//  }
//  mantisse+=1.0f;
//
//  returnvalue=float(ldexp(mantisse, e-127));
//
//  char isNegative = byte1 & 0x8;
//  if (isNegative) returnvalue = - returnvalue;
//
//  return returnvalue;
//}

long parseHex(char* hex, int size)
{
  unsigned long res = 0;

  for (int i=0; i < size; i++)
  {
    int intermediate = (unsigned char)hex[i];
    for (int j=0; j<i; j++)
    {
      intermediate *= 256;
    }
    res += intermediate;
  }


  return res;
}

SMRJoint* SmrLoaderVskCreateJoint(SMRSkeleton &_skeletton, xmlNodePtr _currentNode, bool _isRoot, map<string, double> *_parametersMap)
{

  xmlAttr *currentAttribute = _currentNode->properties;

  string currentJointName((char*)(currentAttribute->children->content));

  cout << endl << currentJointName << " name" << endl;

  SMRJoint* currentJoint(new SMRJoint());
  currentJoint->setName(currentJointName);
  currentJoint->setEndJoint(false);
  currentJoint->setPosition(0.0e0,0.0e0,0.0e0);

  char* position;
  currentAttribute = currentAttribute->next;
  position = ((char*)(currentAttribute->children->content));
  cout<< endl << position << " position" << endl;

  //   position = _currentNode->_xmlAttr
  char* token;
  token = strtok(position, " ");
  char alphabet[] = "abcdefghijklmnopqrstuvwxyz";
  double offset[3] = {0.00,0.00,0.00};
  int i=0;
  while (token != NULL)
  {
    if (strcspn (token,alphabet) != strlen(token))
    {
      //if negative segment value
      if(token[0] == '-')
      {
        char newtoken[256] = "";
        for (unsigned int j=1; j<strlen(token); j++)
        {
          newtoken[j-1] = token[j];
        }
        offset[i]=-(*_parametersMap)[newtoken];
      }
      //if positive segment value
      else
      {
        offset[i]=(*_parametersMap)[token];
      }
    }
    else
    {
      offset[i] = atof(token);
    }
    token = strtok (NULL, " ");
    i++;
  }

  //position is set, update joint
  currentJoint->setPosition(offset[0],offset[1],offset[2]);
  cout << offset[0] << " " << offset[1] << " " << offset[2] << " offset values" << endl;
  _skeletton.insertJoint(currentJoint);

  xmlNode * childNode;

  childNode = _currentNode->children;
  do
  {
    cout << (childNode->name) << endl;
    if (! strcmp((char*)(childNode->name), "JointFree"))
    {
      // this os a 6 dof joint, probably root
      //currentJoint->addDOF(SMRDOF::XAXIS);
      //currentJoint->addDOF(SMRDOF::YAXIS);
      //currentJoint->addDOF(SMRDOF::ZAXIS);
    }
    if (! strcmp((char*)(childNode->name), "JointBall"))
    {
      // this is a 3 dof joint, like shoulder joint
      //currentJoint->addDOF(SMRDOF::XAXIS);
      //currentJoint->addDOF(SMRDOF::YAXIS);
      //currentJoint->addDOF(SMRDOF::ZAXIS);
    }
    if (! strcmp((char*)(childNode->name), "JointHardySpicer"))
    {
      //get AXIS_PAIR
      istringstream axis_stream(string((char*)xmlGetProp(childNode, (xmlChar*)"AXIS-PAIR")));
      SMRVector3 firstAxis,secondAxis;
      if (!(axis_stream >> firstAxis.m_x >> firstAxis.m_y >> firstAxis.m_z >> secondAxis.m_x >> secondAxis.m_y >> secondAxis.m_z)) 
      {
        cout << "Cannot read axes for joint!" << endl;
        assert(0);
      }
      firstAxis.normalize();
      secondAxis.normalize();
      // this is a 2 dof joint
      //currentJoint->addDOF(firstAxis);
      //currentJoint->addDOF(secondAxis);
    }
    if (! strcmp((char*)(childNode->name), "JointHinge"))
    {
      //get AXIS
      istringstream axis_stream(string((char*)xmlGetProp(childNode, (xmlChar*)"AXIS")));
      SMRVector3 firstAxis;
      if (!(axis_stream >> firstAxis.m_x >> firstAxis.m_y >> firstAxis.m_z)) 
      {
        cout << "Cannot read axes for joint!" << endl;
        assert(0);
      }
      firstAxis.normalize();
      //currentJoint->addDOF(firstAxis);
    }

    // child is a segment, reursively create it
    if (! strcmp((char*)(childNode->name), "Segment"))
    {
      SMRJoint* childJoint = SmrLoaderVskCreateJoint(_skeletton, childNode, false, _parametersMap);
      childJoint->setParentName(currentJoint->getName());
    }
    childNode = childNode->next;
  }
  while(childNode != NULL);
  return currentJoint;
}

map<string, double> SmrLoaderVskStoreVskParameters(xmlTextReaderPtr _reader)
{
  map<string, double> parametersMap;
  int ret;
  const xmlChar *name;
  ret = xmlTextReaderRead(_reader);
  xmlChar* skeleton = xmlCharStrdup("Skeleton");
  do
  {
    name = xmlTextReaderConstName(_reader);
    xmlChar* parameter = xmlCharStrdup("Parameter");
    if(!xmlStrcmp(name, parameter))
    {
      string paramName((char*)(xmlTextReaderGetAttribute(_reader, xmlCharStrdup("NAME"))));
      double paramValue(atof((char*)(xmlTextReaderGetAttribute(_reader, xmlCharStrdup("VALUE")))));
      parametersMap.insert(pair<string,double>(paramName, paramValue));
    }
    ret = xmlTextReaderRead(_reader);
  }
  while (ret == 1 && xmlStrcmp(name,skeleton));
  return parametersMap;
}

SMRSkeleton loadSkeletonFromVSK(string _filePath)
{

  map<string, double> parametersMap;

  xmlTextReaderPtr reader;
  xmlDoc * doc;
  reader = xmlReaderForFile(_filePath.c_str(), NULL, 0);
  doc = xmlReadFile(_filePath.c_str(), NULL, 0);

  SMRSkeleton  bindPose(RELATIVEMODE,TRANSLATIONFIRST,"vskImport");

  if (reader != NULL)
  {

    xmlNode *root_element = xmlDocGetRootElement(doc);

    parametersMap = SmrLoaderVskStoreVskParameters(reader);

    xmlTextReaderRead(reader);

    xmlNode *squelettonRootNode = root_element->children;
    while(strcmp((char*)(squelettonRootNode->name),"Skeleton"))
    {
      squelettonRootNode=squelettonRootNode->next;
    }
    squelettonRootNode=squelettonRootNode->children;
    while(strcmp((char*)(squelettonRootNode->name),"Segment"))
    {
      squelettonRootNode=squelettonRootNode->next;
    }

    SmrLoaderVskCreateJoint(bindPose, squelettonRootNode, true, &parametersMap);
    //_skeletton.addrootJoint(createJoint(_skeletton, squelettonRootNode, true));
    //squeletton.addrootJoint(createJoint(_skeletton, squelettonRootNode, false));

    map<string, double>::iterator pos;

    //     for (pos = m_parametersMap.begin(); pos != m_parametersMap.end(); pos++ )
    //     {
    //       cout << (*pos).first << " " << (*pos).second << endl;
    //     }

  }
  else
  {
    fprintf(stderr, "Unable to open %s\n", _filePath.c_str());
  }

  bindPose.checkEndJoints();
  xmlFreeTextReader(reader);
  xmlFreeDoc(doc);

  //scale everything from millimeter to meter
  for (unsigned int i = 0; i < bindPose.getNumJoints(); i++)
  {
    bindPose.getJoint(i)->setPosition(bindPose.getJoint(i)->getPosition()*0.001);
    bindPose.getJoint(i)->setEndLength(bindPose.getJoint(i)->getEndLength()*0.001);
  }

  return bindPose;
}

/**
* /brief
* Load a vicon .V motion file  into an SMRMotion
*/
void loadMotionFromVSK(string _skelFileName, string _motionfilename, SMRMotion &_motion)
{

  SMRSkeleton bindPose = loadSkeletonFromVSK(_skelFileName);
  bindPose.setMode(ABSOLUTEMODE);
  //bindPose.setRotationOrder(TRANSLATIONFIRST);
  SMRSkeleton bindPose1 = bindPose;
  bindPose1.setMode(RELATIVEMODE);
  /*******************************************************************/

  std::ifstream file(_motionfilename.c_str(), ios::in | ios::binary);
  //char *token, line[1500];
  //double joint[6] = {0.0,0.0,0.0,0.0,0.0,0.0};

  std::vector<double> values;

  char buffer[500];
  //lets parse the binary V file
  //first, get rif of the file header (4 bytes)
  file.read(buffer,4);
  //second, get rid of the static data area : a succession of sections
  //a section has a header (32 bytes long)
  //file.read(buffer,4);
  //buffer[4]='\0';
  //char* biffer = "264";
  //buffer[0] = biffer[0];
  //buffer[1] = biffer[1];
  //buffer[2] = biffer[2];
  //buffer[3] = biffer[3];
  long sectionLength;
  int bodyValueStart = 0;
  int bodyValueSize = 0;
  int numBodies = 0;
  int globalBodyGroupID = 0;

  // Ugly temporary hack because I don't have time to figure out how to get sampling frequency in .V files
  // the vicon motion capture files people usually owrk with have indeed a sampling frequency of 120Hz
  _motion.setTimeStep(1.0/120.0);


  do {
    file.read(buffer,4);
    buffer[4]='\0';
    // get the section content's length
    sectionLength = parseHex(buffer, 4);
    file.read(buffer,32-4);
    // if we encounter a datagroup section, we must parse it
    if ( !strcmp(buffer,"DATAGROUP") )
    {
      int parsedSectionChars = 0;
      while(parsedSectionChars < sectionLength)
      {
        //get the length of the record (2 bytes)
        //! it seems recLength do not take into account the trailing
        //'\0' at the end of description !
        file.read(buffer,2);
        unsigned int recLength = parseHex(buffer,2) + 1;

        //skip group ID (2 bytes)
        file.read(buffer,2);
        int groupID = parseHex(buffer,2);

        //get description length
        file.read(buffer,1);
        int dl = (unsigned char)buffer[0];

        //get description (there is a '\0' character after description)
        file.read(buffer,dl);
        char description[256];
        strcpy(description,buffer);

        //get type
        file.read(buffer,1);
        //int storageType = (unsigned char)buffer[0];

        //get width (redundant)
        file.read(buffer,1);
        int storageSize = (unsigned char)buffer[0];
        //get frame rate
        //file.read(buffer,4);
        //buffer[4]='\0';
        //float frameRate = hexToFloat(buffer[3],buffer[2],buffer[1],buffer[0]);
        float frameRate;
        file.read((char*)(&frameRate),sizeof(float));

        //get number of DOF
        file.read(buffer,1);
        int nDof = parseHex(buffer,1);        

        //assume data type is always double and that we work on x86 32 bits, double size is then 8 bytes
        if( strcmp(description,"Global Bodies"))
        {
          bodyValueStart += nDof * storageSize;
        }else
        {
          bodyValueSize = nDof * storageSize;
          numBodies = nDof;
          globalBodyGroupID = groupID;
          _motion.setTimeStep(1.0 / frameRate );
        }

        //read end of record
        file.seekg((recLength - 2 - 1 - dl -1 -1 -1 -4 -1 ), ios_base::cur);

        parsedSectionChars += (2 + 1 + recLength);
      }
    }else
    {
      // if unrelevant, finish parsing the section content
      file.seekg(sectionLength, ios_base::cur);
    }
  }while(sectionLength > 0);
  // dont know why two bits are missing
  file.read(buffer,2);

  //here is the dynamic section
  //get the fist dynamic data group
  file.read(buffer,2);
  unsigned int dataGroupLength = parseHex(buffer,2);

  while( !(file.fail()) && !(file.eof()) )
  {
    //bool eof = file.eof();
    int dataGroupIndex = 0;
    // dont know why two bits are missing
    //file.read(buffer,2);

    //get the group index
    file.read(buffer,2);
    dataGroupIndex = parseHex(buffer,2);

    //get frameNumber... 
    file.read(buffer,4);
    //long anotherSomething = parseHex(buffer,4);

    //if data do not concern body, skip it
    if (dataGroupIndex != globalBodyGroupID)
    {
      // check this out !
      file.seekg(dataGroupLength - 2 - 4, ios_base::cur);
    }else
    {
      //data should appear here (as double)
      SMRSkeleton newPose = bindPose;
      // - 2 : discard kitchen and lamp root nodes (should use a vector for this)
      for (int i=0; i < (numBodies / 6); i++)
      {
        SMRQuaternion globalRot;
        double dofValue;
        //read three translations
        //read three rotations
        file.read((char*)(&dofValue),sizeof(double));
        double axisX = dofValue;
        //SMRQuaternion rotX(SMRVector3(1.0f,0.0f,0.0f),(float)dofValue);
        //rotX.normalize();
        file.read((char*)(&dofValue),sizeof(double));
        //SMRQuaternion rotY(SMRVector3(0.0f,1.0f,0.0f),(float)dofValue);
        //rotY.normalize();
        double axisY = dofValue;
        file.read((char*)(&dofValue),sizeof(double));
        //SMRQuaternion rotZ(SMRVector3(0.0f,0.0f,1.0f),(float)dofValue);
        //rotZ.normalize();
        double axisZ = dofValue;
        //globalRot.identity();
        //globalRot*=rotX;
        //globalRot*=rotZ;
        //globalRot*=rotY;
        SMRVector3 axis(axisX,axisY,axisZ);
        double value = axis.norm();
        globalRot = SMRQuaternion(axis,value);
        double Tx,Ty,Tz;
        file.read((char*)(&Tx),sizeof(double));
        file.read((char*)(&Ty),sizeof(double));
        file.read((char*)(&Tz),sizeof(double));

        Tx *= 0.001;
        Ty *= 0.001;
        Tz *= 0.001;

        //feed the skeleton (only take rotational info, exept for root)
        if (i < (int)(newPose.getNumJoints()))
        {
          newPose.getJoint(i)->setOrientation(globalRot);
#ifdef false
          if (newPose.getJoint(i)->getParentName() != "")
          {
            SMRVector3 localPos             =  bindPose1.getJoint(i)->getPosition();
            SMRVector3 parentAbsolutePos    =  newPose.getJointByName(newPose.getJoint(i)->getParentName())->getPosition();
            SMRQuaternion parentAbsoluteRot =  newPose.getJointByName(newPose.getJoint(i)->getParentName())->getOrientation();
            parentAbsoluteRot.rotate(localPos);
            newPose.getJoint(i)->setPosition( parentAbsolutePos + localPos );
          }else
          {
#endif

            if (newPose.getJoint(i)->isEndJoint())
            {
              newPose.getJoint(i)->setEndLength(SMRVector3(Tx,Ty,Tz));
              newPose.getJoint(i)->setPosition( SMRVector3(Tx,Ty,Tz) ); 
            }else
            {
              newPose.getJoint(i)->setPosition( SMRVector3(Tx,Ty,Tz) );
            }
#ifdef false
          }
#endif
        }else
        {
          cout << ".v motion files declares more joints than .vsk skeleton template" << endl;
          exit(1);
        }
      }
      newPose.checkEndJoints();
      newPose.setMode(RELATIVEMODE);
      //SMRQuaternion identity;
      //identity.identity();
      //for (unsigned int i = 0; i < newPose.getNumJoints(); i++ )
      //{
      //  newPose.getJoint(i)->setOrientation(identity);
      //}
      newPose.setRotationOrder(ROTATIONFIRST);
      _motion.insertSkeleton(newPose);
    }
    file.read(buffer,2);
    dataGroupLength = parseHex(buffer,2);
  }
  // finished parsing the file
}

SMRMotion loadMotionFromVSK(string _skelFileName, string _motionfilename)
{
  SMRMotion myMotion;
  loadMotionFromVSK(_skelFileName, _motionfilename, myMotion);
  return myMotion;
}
