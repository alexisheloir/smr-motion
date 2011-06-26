//add DOF along X axis
currentJoint->addDOF(SMRDOF::XAXIS,-M_PI/16.0f,M_PI/16.0f,SMRUtils::degToRad(0));
//add DOF along Y axis
currentJoint->addDOF(SMRDOF::YAXIS,-M_PI/3.0f,M_PI/3.0f,SMRUtils::degToRad(0));
//add DOF along Z axis
currentJoint->addDOF(SMRDOF::ZAXIS,-M_PI/3.0f,M_PI/3.0f,SMRUtils::degToRad(0));