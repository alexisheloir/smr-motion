#include "SmrMotionPlayer.h"

// an SMRMotionPlayer can not only be constructed with an existing motion
SMRMotionPlayer::SMRMotionPlayer(SMRMotion * _motion) :
  m_motion(_motion),
  m_interpolation(false),
  m_frame(0.0f)
{
  computeBoundingBox();
}

SMRMotionPlayer::SMRMotionPlayer() :
    m_interpolation(false),
  m_frame(0.0f)
{
}

void 
SMRMotionPlayer::process(float relativeTime)
{
  if (m_motion->getNumFrames() == 0)
  {
    LOG_FATAL(logger,"this motion player has an empty motion");
    return;
  }
  m_frame = relativeTime * (float)(m_motion->getNumFrames());
  unsigned int curFrame = static_cast<unsigned int>(m_frame); // floor the m_frame value
  // if interpolation, go go go !!!
  if (m_interpolation && m_motion->getNumFrames() > 1)
  {
    float alpha = m_frame - curFrame ;
    m_skeleton =  interpolateSkeletons(m_motion->getSkeleton(curFrame),
                                     m_motion->getSkeleton(curFrame+1),
                                     alpha);
  }
  else
    m_skeleton = m_motion->getSkeleton(curFrame);
}

// this set of methods allows to control the actuator

void 
SMRMotionPlayer::setFrameToDisplay(unsigned int _numFrame)
{
  m_frame = static_cast<float>(_numFrame);
}

void 
SMRMotionPlayer::setFrameToDisplay(float _numFrame)
{
  m_frame = _numFrame;
}

void 
SMRMotionPlayer::setTimeToDisplay(float _time)
{
  m_frame = (static_cast<float>(_time / m_motion->getTimeStep() / 1000.0));
}

float 
SMRMotionPlayer::getTotalTime()
{
  return (static_cast<float>(m_motion->getNumFrames() * m_motion->getTimeStep() * 1000.0));
}

void 
SMRMotionPlayer::computeBoundingBox()
{
  BoundingBox myBoundingBox;
  SMRSkeleton currentSkeleton;
  SMRJoint* currentJoint;
  SMRVector3 currentVector;

  myBoundingBox.mx = DBL_MAX;
  myBoundingBox.Mx = -DBL_MAX;
  myBoundingBox.my = DBL_MAX;
  myBoundingBox.My = -DBL_MAX;;
  myBoundingBox.mz = DBL_MAX;
  myBoundingBox.Mz = -DBL_MAX;;

  for (unsigned int i=0; i < m_motion->getNumFrames(); i = i+10)
  {
    currentSkeleton = m_motion->getSkeleton(i);
    currentSkeleton.setMode(ABSOLUTEMODE);
    for (unsigned int j=0; j < currentSkeleton.getNumJoints(); j++ )
    {
      currentJoint = currentSkeleton.getJoint(j);
     currentVector = currentJoint->getPosition();
    if ( currentVector.X() < myBoundingBox.mx ) myBoundingBox.mx = currentVector.X();
    if ( currentVector.X() > myBoundingBox.Mx ) myBoundingBox.Mx = currentVector.X();
    if ( currentVector.Y() < myBoundingBox.my ) myBoundingBox.my = currentVector.Y();
    if ( currentVector.Y() > myBoundingBox.My ) myBoundingBox.My = currentVector.Y();
    if ( currentVector.Z() < myBoundingBox.mz ) myBoundingBox.mz = currentVector.Z();
    if ( currentVector.Z() > myBoundingBox.Mz ) myBoundingBox.Mz = currentVector.Z();
    }
  }
  m_boundingBoxDiagonal = sqrt( SQR(myBoundingBox.Mx-myBoundingBox.mx)  +  SQR(myBoundingBox.My-myBoundingBox.my)  +  SQR(myBoundingBox.Mz-myBoundingBox.mz) );
  
  m_boundingBox = myBoundingBox;
}
