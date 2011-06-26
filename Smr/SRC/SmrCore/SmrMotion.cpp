#include "SmrMotion.h"

SMRMotion::SMRMotion(const SMRMotion &_motion)
{
  m_timeStep = _motion.getTimeStep();
  for (unsigned int i = 0; i<_motion.getNumFrames(); i++)
  {
    this->insertSkeleton(_motion.getSkeleton(i));
  }
}

const SMRSkeleton &
SMRMotion::getSkeleton (unsigned int _num) const
{
  // just a simple check wether _num belongs to the motion.
  // else last frame is returned
  unsigned int frame;
  if (_num>=getNumFrames())
    frame = getNumFrames() - 1;
  else 
    frame = _num;
  vector<SMRSkeleton>::const_iterator cit = m_skeletonTab.begin();
  cit += frame;
  return *cit;
}


// this mean is somehow the most simple one can consider for a motion
// - quaternions mean are the normalized mean of the quaternions embedded in R^4
// - mean of positions
// The resulting skeleton is computed in ABSOLUTE mode (and not in relative mode)

SMRSkeleton 
SMRMotion::getMean(SMRModeType mode) const
{
  // In this function, we omit the root orientation since it is not
  // representative of the motion of the body

  SMRSkeleton meanSkeleton = *m_skeletonTab.begin();
  meanSkeleton.setMode(mode);
  // for all my postures
  vector<SMRSkeleton>::const_iterator it(m_skeletonTab.begin());
  //for (vector<SMRSkeleton>::iterator it = m_skeletonTab.begin()+1 ; 
  //  it < m_skeletonTab.end() ;
  //  it++)
  while (it < m_skeletonTab.end()-1)
  {
    it++;
    SMRSkeleton skel(*it);
    skel.setMode(mode);
    // for all the joints
    for(unsigned int i = 0; i < it->getNumJoints(); ++i )
    {
      meanSkeleton.getJoint(i)->setOrientation( meanSkeleton.getJoint(i)->getOrientation() + skel.getJoint(i)->getOrientation() );
      meanSkeleton.getJoint(i)->setPosition( meanSkeleton.getJoint(i)->getPosition() + skel.getJoint(i)->getPosition() );
    }
  }
  // compute mean + normalize
  for(unsigned int i = 0; i < meanSkeleton.getNumJoints(); ++i ) {
    SMRQuaternion q = meanSkeleton.getJoint(i)->getOrientation();// / getNumFrames(); 
    q.normalize();
    meanSkeleton.getJoint(i)->setOrientation( q ) ;
    meanSkeleton.getJoint(i)->setPosition( meanSkeleton.getJoint(i)->getPosition()/getNumFrames() );
  }

  return meanSkeleton;
}

SMRSkeleton 
SMRMotion::getApproximateQuaternionicMean()
{  
  SMRSkeleton meanSkeleton = *m_skeletonTab.begin();
  // for all the joints
  for(unsigned int j = 0; j < meanSkeleton.getNumJoints(); ++j ) 
    meanSkeleton.getJoint(j)->setOrientation( SMRQuaternion(0,0,0,0) );

  SMRVector3 v;
  // for all my postures
  for (vector<SMRSkeleton>::iterator it = m_skeletonTab.begin() ; 
    it < m_skeletonTab.end() ;
    it++)
    // for all the joints
    for(unsigned int i = 0; i < it->getNumJoints(); ++i ) 
    {
      v = Log(it->getJoint(i)->getOrientation());
      meanSkeleton.getJoint(i)->setOrientation( meanSkeleton.getJoint(i)->getOrientation() + SMRQuaternion(0,v.m_x,v.m_y,v.m_z) );
    }
    // compute mean as Exp of mean of Log 
    for(unsigned int i = 0; i < meanSkeleton.getNumJoints(); ++i ) {
      SMRQuaternion q = meanSkeleton.getJoint(i)->getOrientation();// / getNumFrames(); 
      q = Exp( q / getNumFrames() );
      meanSkeleton.getJoint(i)->setOrientation( q ) ;
    }

    return meanSkeleton;
}


SMRSkeleton 
SMRMotion::getQuaternionicMean()
{  
  double epsilon = 0.0000001;
  bool loop = true;

  unsigned int iteration = 0;

  SMRSkeleton meanSkeleton = *m_skeletonTab.begin();
  SMRSkeleton variation = meanSkeleton ;
  SMRMotion myMotionTmp = *this;

  vector<bool> loopVect (meanSkeleton.getNumJoints(), true);

  while(loop)
  {
    loop = false;

    // first step. Multiply by inverse of current mean
    // for all the postures
    for (unsigned int s = 0 ; s < getNumFrames() ; s++)
      // for all the joints
      for(unsigned int i = 0; i < getSkeleton(s).getNumJoints(); ++i ) 
      {
        if ( loopVect.at(i) ) 
          // if the quaternion has not yet reached the mean
        {
          SMRQuaternion q = Inverse(meanSkeleton.getJoint(i)->getOrientation());
          q = q * getSkeleton(s).getJoint(i)->getOrientation() ;
          myMotionTmp.getSkeleton(s).getJoint(i)->setOrientation(q);
        }
      }

      // recompute meanvariation
      variation = myMotionTmp.getApproximateQuaternionicMean();

      // compute new mean and evaluate condition
      // for all the joints
      unsigned int test = 0;
      for(unsigned int i = 0; i < meanSkeleton.getNumJoints(); ++i ) 
      {
        if ( loopVect.at(i) ) 
          // if the quaternion has not yet reached the mean
        {
          SMRQuaternion q = meanSkeleton.getJoint(i)->getOrientation();
          SMRQuaternion dq = variation.getJoint(i)->getOrientation();
          meanSkeleton.getJoint(i)->setOrientation( q * dq );
          loopVect[i] = ( Log ( dq ).norm() > epsilon ) ;
          loop = loop  || loopVect.at(i) ;
        }
        if (!loopVect[i]) test ++;
      }

      //cout << "Iteration : " << iteration << endl;
      //cout << "nombre de mean trouves : " << test << " sur " << meanSkeleton.getNumJoints() << endl;
      iteration++;
  }

  return meanSkeleton;
}

bool
SMRMotion::dumpQuaternionsToFile(const string _filename, SMRModeType _mode)
{
  // Open file for writing
  ofstream file( _filename.c_str(), ios::out );

  if( file.fail() ) {
    cerr << "Couldn't open " << _filename << endl;
    return false;
  }

  try {
    // for all my postures
    for (vector<SMRSkeleton>::iterator it = m_skeletonTab.begin() ; 
      it < m_skeletonTab.end() ;
      it++)
    {
      it->setMode(_mode);
      // for all the joints
      for(unsigned int i = 0; i < it->getNumJoints(); ++i ) {
        SMRQuaternion q = it->getJoint(i)->getOrientation();
        file << q.m_w << " " << q.m_x << " " << q.m_y << " " << q.m_z << " "; 
      }
      file << endl;
    }

    file.close();
    cerr << "* Dump to file: " << _filename << " succesful !" << endl;
    return true;
  }
  catch( ios_base::failure &err ) {
    cerr << "Couldn't write to " << _filename << endl;
    cerr << " Reason: " << err.what() << endl;
    return false;
  }
}

bool
SMRMotion::dumpPositionsToFile(const string _filename, SMRModeType _mode)
{
  // Open file for writing
  ofstream file( _filename.c_str(), ios::out );

  if( file.fail() ) {
    cerr << "Couldn't open " << _filename << endl;
    return false;
  }

  try {
    // for all my postures
    for (vector<SMRSkeleton>::iterator it = m_skeletonTab.begin() ; 
      it < m_skeletonTab.end() ;
      it++)
    {
      it->setMode(_mode);
      // for all the joints
      for(unsigned int i = 0; i < it->getNumJoints(); ++i ) {
        SMRVector3 v = it->getJoint(i)->getPosition();
        file << v.X() << " " << v.Y() << " " << v.Z() << " ";
      }
      file << endl;
    }

    file.close();
    cerr << "* Dump to file: " << _filename << " succesful !" << endl;
    return true;
  }
  catch( ios_base::failure &err ) {
    cerr << "Couldn't write to " << _filename << endl;
    cerr << " Reason: " << err.what() << endl;
    return false;
  }
}

bool 
SMRMotion::dumpEulerAnglesToFile(const string _filename , SMRModeType _mode)
{
  // Open file for writing
  ofstream file( _filename.c_str(), ios::out );

  if( file.fail() ) {
    cerr << "Couldn't open " << _filename << endl;
    return false;
  }

  try {
    // for all my postures
    for (vector<SMRSkeleton>::iterator it = m_skeletonTab.begin() ; 
      it < m_skeletonTab.end() ;
      it++)
    {
      it->setMode(_mode);
      // for all the joints
      for(unsigned int i = 0; i < it->getNumJoints(); ++i ) {
        SMRQuaternion q = it->getJoint(i)->getOrientation();
        double eulerAngles[3] = {0.0,0.0,0.0};
        q.toEulerAngles(eulerAngles[0],eulerAngles[1],eulerAngles[2]);
        file << eulerAngles[0] << " " << eulerAngles[1] << " " << eulerAngles[2] << " ";
      }
      file << endl;
    }

    file.close();
    cerr << "* Dump to file: " << _filename << " succesful !" << endl;
    return true;
  }
  catch( ios_base::failure &err ) {
    cerr << "Couldn't write to " << _filename << endl;
    cerr << " Reason: " << err.what() << endl;
    return false;
  }
}

void 
SMRMotion::hemispherize() 
{
  unsigned int cpt = 0;
  SMRSkeleton myMeanSkeleton = getQuaternionicMean();
  // for all my postures
  for (vector<SMRSkeleton>::iterator it = m_skeletonTab.begin() ; 
    it < m_skeletonTab.end() ;
    it++)
  {
    // for all the joints
    for(unsigned int i = 0; i < it->getNumJoints(); ++i ) {
      SMRQuaternion q1 = it->getJoint(i)->getOrientation();
      SMRQuaternion q_mean = myMeanSkeleton.getJoint(i)->getOrientation();

      if (DotProduct(q_mean,q1) < 0)
      {  cpt++;
      it->getJoint(i)->setOrientation(-q1);}
      // else we are on the right hemisphere
    }
  }
  cout << "Number of inversed quats : " << cpt << " on " << myMeanSkeleton.getNumJoints() << " joints " << endl;
}


SMRTimeSerie<SMRQuaternion> 
SMRMotion::getQuaternionicTimeSerie(unsigned int _numJoint) 
{
  SMRTimeSerie<SMRQuaternion> result;

  if (_numJoint<0||_numJoint>=m_skeletonTab.begin()->getNumJoints())
    // assuming the number of Joints is constant throughout the motion
    cout << "Invalid joint identifier in SMRMotion::getTimeSerie : " << _numJoint << ". Returning Null serie " << endl;
  else
  {
    // for all my postures
    for (vector<SMRSkeleton>::iterator it = m_skeletonTab.begin() ; 
      it < m_skeletonTab.end() ;
      it++)
      result.getSerie().push_back(it->getJoint(_numJoint)->getOrientation());
  }

  return result;
}

SMRTimeSerie<SMRQuaternion> 
SMRMotion::getQuaternionicTimeSerie(string _name) 
{
  SMRTimeSerie<SMRQuaternion> result;

  if (!(m_skeletonTab.begin()->getJointByName(_name)))
    // assuming the number of Joints is constant throughout the motion
    cout << "Invalid joint identifier in SMRMotion::getTimeSerie : " << _name << ". Returning Null serie " << endl;
  else
  {
    // for all my postures
    for (vector<SMRSkeleton>::iterator it = m_skeletonTab.begin() ; 
      it < m_skeletonTab.end() ;
      it++)
      result.getSerie().push_back(it->getJointByName(_name)->getOrientation());
  }

  return result;
}

SMRTimeSerie<SMRVector3> 
SMRMotion::getJointTrajectory(unsigned int _numJoint) 
{
  SMRTimeSerie<SMRVector3> result;
  if (_numJoint<0||_numJoint>=m_skeletonTab.begin()->getNumJoints())
    // assuming the number of Joints is constant throughout the motion
    cout << "Invalid joint identifier in SMRMotion::getTimeSerie : " << _numJoint << ". Returning Null serie " << endl;
  else
  {
    changeMode(ABSOLUTEMODE);
    // for all my postures
    for (vector<SMRSkeleton>::iterator it = m_skeletonTab.begin() ; 
      it < m_skeletonTab.end() ; it++)
      result.getSerie().push_back(it->getJoint(_numJoint)->getPosition());
  }
  return result;
}

SMRTimeSerie<SMRVector3> 
SMRMotion::getJointTrajectory(string _name) 
{
  SMRTimeSerie<SMRVector3> result;
  if (!m_skeletonTab.begin()->getJointByName(_name))
    // assuming the number of Joints is constant throughout the motion
    cout << "Invalid joint identifier in SMRMotion::getTimeSerie : " << _name << ". Returning Null serie " << endl;
  else
  {
    changeMode(ABSOLUTEMODE);
    // for all my postures
    for (vector<SMRSkeleton>::iterator it = m_skeletonTab.begin() ; 
      it < m_skeletonTab.end() ; it++)
      result.getSerie().push_back(it->getJointByName(_name)->getPosition());
  }
  return result;
}

void 
SMRMotion::changeMode(SMRModeType _mode)
{
  for (vector<SMRSkeleton>::iterator it = m_skeletonTab.begin() ; 
    it < m_skeletonTab.end() ;
    it++)
    it->setMode(_mode);
}



void 
SMRMotion::removeSkeleton(unsigned int _num)
{
  // simple boundary check
  if (_num>=0&&_num<getNumFrames())
    m_skeletonTab.erase(m_skeletonTab.begin()+_num);
  else 
    cout << "*** Error : Remove Skeleton frame number " << _num << " aborted [Out of bound]." << endl;
}

void
SMRMotion::warp(SMRHermitePoly<double> _poly)
{
  vector<SMRSkeleton> warpedMotion;

  double numFrames = static_cast<double>(getNumFrames());
  SMRSkeleton firstInterpolator;
  SMRSkeleton secondInterpolator;
  SMRSkeleton interpolatedSkeleton;

  for (unsigned int i=0; i<(getNumFrames()-1); i++)
  {
    double currentFrame = static_cast<double>(i) / numFrames;
    double warpedFrame = _poly.evaluate(currentFrame) * numFrames;

    firstInterpolator = getSkeleton(static_cast<int>(warpedFrame));
    secondInterpolator = getSkeleton(static_cast<int>(warpedFrame+1));

    double dt = warpedFrame - static_cast<int>(warpedFrame);

    interpolatedSkeleton = interpolateSkeletons( firstInterpolator, secondInterpolator, (float)dt );

    warpedMotion.push_back(interpolatedSkeleton);

  }


  m_skeletonTab.clear();
  m_skeletonTab = warpedMotion;
}

void
SMRMotion::resample(double _dt)
{
  // compute new number of frames
  double totalFrame = this->getNumFrames() * (this->getTimeStep() / _dt);
  // compute new skeletons vector with interpolation 
  vector<SMRSkeleton> newMotion;
  double dt = this->getTimeStep();

  SMRSkeleton interpolatedSkeleton;

  if (_dt < dt)
  {
    int j = 0;
    for ( unsigned int i=0 ; i <= this->getNumFrames() ; i++ )
    {
      while( (j * _dt) < (i * dt) )
      {
        interpolatedSkeleton = interpolateSkeletons( this->getSkeleton(i), this->getSkeleton(i+1), ((float)( ((j*_dt)-(i*dt))/dt )) );
        newMotion.push_back(interpolatedSkeleton);
        j++;
      }
    }
  }else if (_dt > dt)
  {
    int j = 0;
    for (int i=0 ; i <= totalFrame ; i++)
    {
      while( (j * dt) < (i * _dt) )
      {
        j++;
      }
      interpolatedSkeleton = interpolateSkeletons( this->getSkeleton(j), this->getSkeleton(j-1), ((float)( ((j*dt-i*_dt)/_dt) )) );
      newMotion.push_back(interpolatedSkeleton);
    }
  }
  m_skeletonTab.clear();
  m_skeletonTab = newMotion;
  this->setTimeStep(_dt);
}

void 
SMRMotion::cutMotionFromFrame(unsigned int _num)
{
  // simple boundary check
  if (_num>=0&&_num<getNumFrames())
    m_skeletonTab.erase(m_skeletonTab.begin()+_num,m_skeletonTab.end());
  else 
    cout << "*** Error : cutMotionFromFrame number " << _num << " aborted [Out of bound]." << endl;
}

void 
SMRMotion::cutMotionUntilFrame(unsigned int _num)
{  
  // simple boundary check
  if (_num>=0&&_num<getNumFrames())
    m_skeletonTab.erase(m_skeletonTab.begin(),m_skeletonTab.begin()+_num);
  else 
    cout << "*** Error : cutMotionUntilFrame number " << _num << " aborted [Out of bound]." << endl;
}

void 
SMRMotion::cutMotion(unsigned int _begin, unsigned int _duration)
{
  // simple boundary check
  if ( _begin < getNumFrames() )
  {
    if ( (_begin + _duration) > getNumFrames() )
      _duration = getNumFrames() - _begin ;
    if (_begin > 0)
      m_skeletonTab.erase(m_skeletonTab.begin(),m_skeletonTab.begin()+_begin);
    m_skeletonTab.erase(m_skeletonTab.begin()+_duration,m_skeletonTab.end());
  }
  else 
    cout << "*** Error : cutMotion [" << _begin << "," << _duration << "] aborted [Out of bound]." << endl;
}

void 
SMRMotion::cropMotion (unsigned int _begin, unsigned int _end)
{
  // simple boundary check
  if (_begin>=0&&_end<getNumFrames()&&_begin<_end)
  {  
    // compute new end index after erasing beginning of motion
    unsigned int newEnd = _end - _begin;
    m_skeletonTab.erase(m_skeletonTab.begin(),m_skeletonTab.begin()+_begin);
    m_skeletonTab.erase(m_skeletonTab.begin()+newEnd,m_skeletonTab.end());
  }
  else 
    cout << "*** Error : cropMotion [" << _begin << "," << _end << "] aborted [Out of bound]." << endl;
}

void 
SMRMotion::decimateMotion (unsigned int _n)
{
  //TODO
}

SMRMotion &  SMRMotion::operator= (const SMRMotion & _motion)
{
  m_skeletonTab.clear();
  m_timeStep = _motion.getTimeStep();
  for (unsigned int i = 0; i<_motion.getNumFrames(); i++)
  {
    this->insertSkeleton(_motion.getSkeleton(i));
  }
  return *this;
}


bool  SMRMotion::operator== (const SMRMotion & _motion)
{
  for (unsigned int i = 0; i<_motion.getNumFrames(); i++)
  {
    if (this->getSkeleton(i) != _motion.getSkeleton(i)) return false;
  }
  return true;
}

SMRMotion & 
SMRMotion::operator+ (const SMRMotion & _motion)
{
  for (unsigned int i = 0; i < _motion.getNumFrames(); i++)
  {
    this->insertSkeleton(_motion.getSkeleton(i));
  }
  return (*this);
}

void 
SMRMotion::mergeMotionAdd(const SMRMotion & _motion)
{

}

void 
SMRMotion::mergeMotionReplace(SMRMotion & _motion)
{
  SMRMotion motion;
  SMRSkeleton poseToMerge;
  motion.setTimeStep( m_timeStep );
  unsigned int i=0;
  for (vector<SMRSkeleton>::iterator it = m_skeletonTab.begin() ;\
    it < m_skeletonTab.end() ;\
    it++)
  {
    poseToMerge = _motion.getSkeleton(i);
    *it += poseToMerge;
    i++;
  }

}

SMRMotion 
SMRMotion::extractSubSkeleton(const SMRSkeleton & _subSkeleton)
{

  SMRMotion motion;
  motion.setTimeStep( m_timeStep );
  for (vector<SMRSkeleton>::iterator it = m_skeletonTab.begin() ;\
    it < m_skeletonTab.end() ;\
    it++)
  {
    motion.insertSkeleton( (*it).getSubSkeleton(_subSkeleton) );
  }

  return motion;

}

SMRMotion
SMRMotion::derivate()
{
  SMRMotion motion;
  SMRSkeleton previousSkeleton = *(m_skeletonTab.begin());
  for (unsigned int i=1; i<this->getNumFrames(); i++)
  {
    SMRSkeleton currentSkeleton = *(m_skeletonTab.begin()+i);
    for (unsigned int j=0;j<previousSkeleton.getNumJoints();j++)
    {
      SMRJoint *previousJoint = previousSkeleton.getJoint(j);
      SMRJoint *currentJoint = currentSkeleton.getJoint(j);
      previousJoint->setOrientation(currentJoint->getOrientation()*Inverse(previousJoint->getOrientation()));
    }
    motion.insertSkeleton(previousSkeleton);
    previousSkeleton=currentSkeleton;
  }
  return motion;
}
