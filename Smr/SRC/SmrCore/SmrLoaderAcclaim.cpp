#include "SmrLoader.h"

void relativize(SMRSkeleton &bindPose, SMRSkeleton &refPose);
void nullRot(SMRSkeleton &bindPose);

SMRSkeleton loadSkeletonFromAcclaim(string _filename)
{
	vector<string> dofVect;

	//cout << "Loading skeleton from file : " << _filename << endl;
	// defines a stream to be read from
	ifstream skelfile(_filename.c_str());
	// a new skeleton to be constructed
	SMRSkeleton  bindPose(RELATIVEMODE,ROTATIONFIRST);
	// rotations will re relative
	bindPose.setMode(RELATIVEMODE);
	// a stack of articulation names
	vector<string> parentStack;
	// parse root infos
	expect(":root", skelfile);
	//push root onto stack
	dofVect.push_back("root+tx");
	dofVect.push_back("root+ty");
	dofVect.push_back("root+tz");
	dofVect.push_back("root+rx");
	dofVect.push_back("root+ry");
	dofVect.push_back("root+rz");

	//instanciate root joint
	SMRJoint* currentJoint = new SMRJoint;
	currentJoint->setName("root");
	currentJoint->setParentName("");
	currentJoint->setPosition(SMRVector3(0.0f,0.0f,0.0f));
	SmrQuaternion id;
	id.identity();
	currentJoint->setOrientation(id);
	bindPose.insertJoint(currentJoint);

	// parse following joints
	expect(":bonedata", skelfile);
	string nextToken("");

	do
	{
		expect("name", skelfile);
		string nodeName;
		skelfile >> nodeName;
		//cout << nodeName << endl;

		/*
		* just a little bit of trigo, in order to convert a length/axis to absolute cartesian coordinates.
		*/

		double x,y,z,length;
		double lX,lY,lZ;//,hyp;

		expect("direction", skelfile);

		skelfile >> x;
		skelfile >> y;
		skelfile >> z;

		expect("length", skelfile);

		skelfile >> length;

		expect("axis",skelfile);
		double a1,a2,a3;
		string axisOrder;

		skelfile >> a1;
		skelfile >> a2;
		skelfile >> a3;

		skelfile >> axisOrder;

		//orientation axis (absolute)
		SmrQuaternion orientation;
		orientation.fromEulerAngles(a1/180.0*M_PI,a2/180.0*M_PI,a3/180.0*M_PI);

		orientation.normalize();

		lX = length * x;
		lY = length * y;
		lZ = length * z;
		SMRVector3 pos(lX,lY,lZ);
		SMRJoint* currentJoint = new SMRJoint;
		currentJoint->setName(nodeName);

		currentJoint->setPosition(pos);
		currentJoint->setOrientation((orientation));
		bindPose.insertJoint(currentJoint);

		skelfile >> nextToken;
		if (nextToken != "end")
		{
			char str[2048];
			skelfile.getline(str, 2048);
			string dofType;
			stringstream dofStream(str,stringstream::in);
			while(dofStream >> dofType)
			{
				dofVect.push_back(nodeName + "+" + dofType);
			}
			expect("end",skelfile);
			skelfile >> nextToken;
		}
		else
			skelfile >> nextToken;
	}
	while (nextToken != ":hierarchy");

	expect("begin",skelfile);
	char str[2048];
	skelfile.getline(str, 2048);
	string parentJointName;
	skelfile >> parentJointName;

	while(parentJointName != "end")
	{
		skelfile.getline(str, 2048);
		stringstream jointStream(str,stringstream::in);
		string childJoint;
		while(jointStream >> childJoint)
		{
			currentJoint = bindPose.getJointByName( childJoint );
			SMRJoint* parentJoint = bindPose.getJointByName( parentJointName );
			currentJoint->setParentName( parentJointName );
			//cout << childJoint << ">>" << parentJointName << endl;
		}
		skelfile >> parentJointName;
	}

	bindPose.setMode(RELATIVEMODE);
	skelfile.close();
	return bindPose;
}

SMRMotion loadMotionFromAcclaim(string _skelfilename, string _motionfilename)
{

	vector<string> dofVect;

	//cout << "Loading skeleton from file : " << _skelfilename << endl;
	// defines a stream to be read from
	ifstream skelfile(_skelfilename.c_str());
	// a new skeleton to be constructed
	SMRSkeleton  bindPose(RELATIVEMODE,ROTATIONFIRST);
	// rotations will re relative
	bindPose.setMode(RELATIVEMODE);
	// a stack of articulation names
	vector<string> parentStack;
	// parse root infos
	expect(":root", skelfile);
	//push root onto stack
	dofVect.push_back("root+tx");
	dofVect.push_back("root+ty");
	dofVect.push_back("root+tz");
	dofVect.push_back("root+rx");
	dofVect.push_back("root+ry");
	dofVect.push_back("root+rz");

	//instanciate root joint
	SMRJoint* currentJoint = new SMRJoint;
	currentJoint->setName("root");
	currentJoint->setParentName("");
	currentJoint->setPosition(SMRVector3(0.0f,0.0f,0.0f));
	SmrQuaternion id;
	id.identity();
	currentJoint->setOrientation(id);
	bindPose.insertJoint(currentJoint);

	// parse following joints
	expect(":bonedata", skelfile);
	string nextToken("");

	do
	{
		expect("name", skelfile);
		string nodeName;
		skelfile >> nodeName;
		//cout << nodeName << endl;

		/*
		* just a little bit of trigo, in order to convert a length/axis to absolute cartesian coordinates.
		*/

		double x,y,z,length;
		double lX,lY,lZ;//,hyp;

		expect("direction", skelfile);

		skelfile >> x;
		skelfile >> y;
		skelfile >> z;

		expect("length", skelfile);

		skelfile >> length;

		expect("axis",skelfile);
		double a1,a2,a3;
		string axisOrder;

		skelfile >> a1;
		skelfile >> a2;
		skelfile >> a3;

		skelfile >> axisOrder;

		//orientation axis (absolute)
		SmrQuaternion orientation;
		orientation.fromEulerAngles(a1/180.0*M_PI,a2/180.0*M_PI,a3/180.0*M_PI);

		orientation.normalize();

		lX = length * x;
		lY = length * y;
		lZ = length * z;
		SMRVector3 pos(lX,lY,lZ);

		//SmrQuaternion inv = Inverse(orientation);
		//inv.rotate(pos);

		SMRJoint* currentJoint = new SMRJoint;
		currentJoint->setName(nodeName);

		currentJoint->setPosition(pos);
		currentJoint->setOrientation((orientation));
		bindPose.insertJoint(currentJoint);

		skelfile >> nextToken;
		//cout << nextToken << endl;
		if (nextToken != "end")
		{
			char str[2048];
			skelfile.getline(str, 2048);
			string dofType;
			stringstream dofStream(str,stringstream::in);
			while(dofStream >> dofType)
			{
				//cout << nodeName + "+" + dofType << endl;
				dofVect.push_back(nodeName + "+" + dofType);
			}
			expect("end",skelfile);
			skelfile >> nextToken;
		}
		else
			skelfile >> nextToken;
	}
	while (nextToken != ":hierarchy");

	// cout << "end" << endl;
	expect("begin",skelfile);
	char str[2048];
	skelfile.getline(str, 2048);
	string parentJointName;
	skelfile >> parentJointName;
	//cout << parentJointName << endl;

	while(parentJointName != "end")
	{
		skelfile.getline(str, 2048);
		stringstream jointStream(str,stringstream::in);
		string childJoint;
		while(jointStream >> childJoint)
		{
			currentJoint = bindPose.getJointByName( childJoint );
			SMRJoint* parentJoint = bindPose.getJointByName( parentJointName );
			currentJoint->setParentName( parentJointName );
			//cout << childJoint << ">>" << parentJointName << endl;
		}
		skelfile >> parentJointName;
	}

	bindPose.setMode(RELATIVEMODE);
	SMRSkeleton refPose = bindPose;
	SMRSkeleton nullPose = refPose;
	nullRot(nullPose);
	skelfile.close();
	SMRMotion motion;

	// defines a stream to be read from
	ifstream motionfile(_motionfilename.c_str());
	string dofIterator;
	int nframes = 1;
	string jointName;
	SMRSkeleton skeleton = nullPose;

	expect("1",motionfile);
	motionfile.getline(str, 2048);
	do
	{
		if (!motionfile.getline(str, 2048)) break;
		stringstream valueStream(str,stringstream::in);
		valueStream >> jointName ;
		char * pEnd;
		if ( strtol(jointName.c_str(),&pEnd,0) == nframes+1 )
		{
			nframes++;
			skeleton.setMode( RELATIVEMODE );
			relativize(skeleton, refPose);
			skeleton.checkEndJoints();
			skeleton.setRotationOrder( ROTATIONFIRST );
			skeleton.checkEndJoints();
			motion.insertSkeleton( skeleton );
			skeleton = nullPose;
		}
		else
		{
			currentJoint = skeleton.getJointByName( jointName );
			int jointIndex = -1;
			int rank = 0;

			vector<string>::iterator dofit;
			for (dofit = dofVect.begin() ; dofit < dofVect.end() ; dofit++)
			{
				if (static_cast<string>(*dofit).find(jointName) != string::npos)
				{
					jointIndex = rank;
					break;
				}
				rank++;
			}



			double nextValue;
			int dofInd = 0;
			while (valueStream >> nextValue)
			{
				dofIterator = dofVect.at(rank + dofInd);
				int npos = static_cast<int>((dofIterator).find('+',0));
				string jointName = (dofIterator).substr(0,npos);
				string dofType = (dofIterator).substr(npos+1);

				currentJoint = skeleton.getJointByName(jointName);

				dofInd++;

				if(dofType == "tx")
				{
					SMRVector3 pos = currentJoint->getPosition();
					currentJoint->setPosition( nextValue, pos.Y(), pos.Z() );
				}
				else if(dofType == "ty")
				{
					SMRVector3 pos = currentJoint->getPosition();
					currentJoint->setPosition( pos.X(), nextValue, pos.Z() );
				}
				else if(dofType == "tz")
				{
					SMRVector3 pos = currentJoint->getPosition();
					currentJoint->setPosition( pos.X(), pos.Y(), nextValue );
				}
				else if(dofType == "rx")
				{
					SmrQuaternion quat = currentJoint->getOrientation();
					SMRVector3 axis(1.0,0.0,0.0);
					SmrQuaternion rot(axis,nextValue/180.0*M_PI);
					//if (nframes < 3) cout << jointName << " rx " << nextValue << endl;
					currentJoint->setOrientation( rot*quat );
				}
				else if(dofType == "ry")
				{
					SmrQuaternion quat = currentJoint->getOrientation();
					SMRVector3 axis(0.0,1.0,0.0);
					SmrQuaternion rot(axis,nextValue/180.0*M_PI);
					//if (nframes < 3) cout << jointName << " ry " << nextValue << endl;
					currentJoint->setOrientation( rot*quat );
				}
				else if(dofType == "rz")
				{
					SmrQuaternion quat = currentJoint->getOrientation();
					SMRVector3 axis(0.0,0.0,1.0);
					SmrQuaternion rot(axis,nextValue/180.0*M_PI);
					//if (nframes < 3) cout << jointName << " rz " << nextValue << endl;
					currentJoint->setOrientation( rot*quat );
				}

				if(jointName == "root")
				{
					//currentJoint->setOrientation(id);
					//SMRVector3 p(0.0,0.0,0.0);
					//currentJoint->setPosition(p);
				}
			}

			if(jointName != "root")
			{
				//SmrQuaternion rot1 = currentJoint->getOrientation();
				//SMRVector3 p;
				//p = currentJoint->getPosition();
				//rot1.rotate(p);
				//currentJoint->setPosition(p);
			}
		}
	}
	while(true);
	motionfile.close();
	//cout << "nb frames" << motion.getNumFrames() << endl;
	motion.setTimeStep(0.01);
	return motion;
}

void relativize(SMRSkeleton &bindPose, SMRSkeleton &refPose)
{
	/************************************************************************/
	// switch to relative
	ptrJoint joint;
	SMRVector3 pos;
	for( int i = bindPose.getNumJoints() - 1 ; i > 0; --i )
	{
		joint = bindPose.getJoint(i);
		// has parent?
		string pName = joint->getParentName();
		if( strcmp(joint->getParentName().c_str(),""))
		{

			SmrQuaternion bindQuat = joint->getOrientation();

			ptrJoint refJoint = refPose.getJointByName( joint->getName() );
			SmrQuaternion refQuat = refJoint->getOrientation();

			ptrJoint parentBindJoint = bindPose.getJointByName( joint->getParentName() );
			SmrQuaternion parentBindQuat = parentBindJoint->getOrientation();

			ptrJoint parentRefJoint = refPose.getJointByName( joint->getParentName() );
			SmrQuaternion parentRefQuat = parentRefJoint->getOrientation();

			pos = joint->getPosition();
			SmrQuaternion quat = refQuat;
			SmrQuaternion invQuat = Inverse(quat);
			invQuat.rotate(pos);
			joint->setPosition(pos);

			SmrQuaternion parentRefQuatInv = Inverse(parentRefQuat);
			SmrQuaternion rot_parent_current = parentRefQuatInv * quat;
			joint->setOrientation( rot_parent_current * bindQuat );
		}
	}
}

void nullRot(SMRSkeleton &bindPose)
{
	/************************************************************************/
	// switch to relative
	ptrJoint joint;
	SmrQuaternion id;
	id.identity();
	for( int i = bindPose.getNumJoints() - 1 ; i > 0; --i )
	{
		joint = bindPose.getJoint(i);
		joint->setOrientation(id);
	}
}
