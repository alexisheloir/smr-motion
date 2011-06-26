#include "SmrPCA.h"

#pragma warning( disable : 4996 )

SMRPCA::SMRPCA (const SMRMotion & _motion,  SMRModeType _mode) :
m_mode(_mode), m_motion(_motion)  
{  
}

void 
SMRPCA::saveResult(string _fileprefix, unsigned int _numMotionCurves)
{
  m_eigenValues.dumpSerieToFile(_fileprefix+"_eigenValues.data");
  unsigned int num = (_numMotionCurves<m_motionCurves.size() ? _numMotionCurves : m_motionCurves.size());
  for (unsigned int i=0;i<num;i++)
  {  
    char ch[10];sprintf( ch,"%d", i);
    m_motionCurves.at(i).dumpSerieToFile(_fileprefix+"_motioncurve_"+ch+".data");
  }
}

// ------------------------ SMRPCAPOSITION ------------------------------------------------------
// ----------------------------------------------------------------------------------------------

SMRPCAPosition::SMRPCAPosition (const SMRMotion & _motion, SMRModeType _mode) 
: SMRPCA(_motion,_mode)
{
  // change mode of representation
  m_motion.changeMode(_mode);
  
  //resize data matrix
  m_positionData.resize(m_motion.getNumFrames(),m_motion.getSkeleton(0).getNumJoints()*3);
  
  // substract mean
  meanSkeleton = m_motion.getMean(_mode);
  
  SMRSkeleton skel; SMRVector3 pos ;
  for (unsigned int i = 0 ; i < m_motion.getNumFrames() ; i ++)
  {
    skel = m_motion.getSkeleton(i);
    for (unsigned int j = 0 ; j < skel.getNumJoints() ; j ++) 
      // discard root joint
    {
      pos = skel.getJoint(j)->getPosition() - meanSkeleton.getJoint(j)->getPosition();
      m_positionData(i+1, j*3+1) = pos.m_x;
      m_positionData(i+1, j*3+2) = pos.m_y;
      m_positionData(i+1, j*3+3) = pos.m_z;
    }
  }
  
}

SMRSkeleton 
SMRPCAPosition::toEigenPosture(ColumnVector _v)
{
  SMRSkeleton skel = meanSkeleton; // initialize structure
  for (unsigned int i = 0; i < skel.getNumJoints();i++)
    skel.getJoint(i)->setPosition(_v(i*3+1),_v(i*3+2),_v(i*3+3) );
  return skel;
}

SMRTimeSerie<>
SMRPCAPosition::toMotionCurve(RowVector _v)
{
  SMRTimeSerie<> s; // initialize structure
  for (int i = 1; i <= _v.ncols();i++)
    s.getSerie().push_back(_v(i));
  return s;
}

void 
SMRPCAPosition::computeEigenPostures()
{
  DiagonalMatrix diag; // matrix of eigen values
  Matrix U; // Matrix of eigenPostures 
  Matrix V; // Matrix of Motion curves
  
  // perform SVD on data
  try
  {
    SVD(m_positionData,diag,U,V);
  }
  catch(BaseException) { 
    cout << "Error in decomposition. Reason : " << endl;
    cout << BaseException::what() << endl; 
  }
  
  // output result
  //cout << "U : " << U.ncols() << " " << U.nrows() << endl; 
  //cout << "V : " << V.ncols() << " " << V.nrows() << endl; 
  //// diagonal matrix
  //cout << setprecision(3) << diag;  
  
  // get EigenValues from Matrix diag
  for (int e = 1; e <= diag.ncols() ; e++)
    m_eigenValues.getSerie().push_back(diag(e));
  
  // get Eigenpostures from Matrix V
  for (int i = 1 ; i <= V.nrows() ; i++)
    m_eigenPostures.push_back( toEigenPosture( V.column(i).as_column() ) );
  
  // get motion curves from Matrix U
  for (int mc = 1 ; mc <= U.ncols() ; mc++)
    m_motionCurves.push_back( toMotionCurve( U.column(mc).as_row() ) );
  
  
}

SMRMotion 
SMRPCAPosition::getProjectedMotion(unsigned int _num)
{
  SMRMotion result;
  SMRSkeleton skelFrame = meanSkeleton;
  SMRSkeleton ep;
  for (unsigned int i=0; i <m_motion.getNumFrames();i++)
  {
    // for each frame in the motion
    for (unsigned int j=0; j <skelFrame.getNumJoints();j++)
      for (unsigned int n=0;n<_num;n++)
      {
        ep = getEigenPosture(n);
        // add the contribution of each desired components of the motion
        skelFrame.getJoint(j)->setPosition(skelFrame.getJoint(j)->getPosition() + 
                        ep.getJoint(j)->getPosition()*(( m_motionCurves.at(n) )[i]) * m_eigenValues[n]);
      }
    result.insertSkeleton(skelFrame);
    skelFrame = meanSkeleton; // for each frame the contribution is added to the mean skeleton !
  }
  return result;
}

// ------------------------ SMRPCALINEARIZEDQUAT ------------------------------------------------------
// ----------------------------------------------------------------------------------------------

SMRPCALinearizedQuaternion::SMRPCALinearizedQuaternion (const SMRMotion & _motion,  SMRModeType  _mode) 
: SMRPCA(_motion,_mode)
{
  
  // change mode of representation
  m_motion.changeMode(_mode);
  
  //resize data matrix
  m_positionData.resize(m_motion.getNumFrames(),m_motion.getSkeleton(0).getNumJoints()*3);
  
  // substract mean
  //meanSkeleton = m_motion.getQuaternionicMean();
  meanSkeleton = m_motion.getMean(_mode);
  //meanSkeleton= m_motion.getApproximateQuaternionicMean();
  
  SMRSkeleton skel; SMRVector3 linearQuat ; 
  for (unsigned int i = 0 ; i < m_motion.getNumFrames() ; i ++)
  {
    skel = m_motion.getSkeleton(i);
    for (unsigned int j = 0 ; j < skel.getNumJoints() ; j ++) 
    {
      linearQuat = Log(skel.getJoint(j)->getOrientation()) - Log(meanSkeleton.getJoint(j)->getOrientation());
      m_positionData(i+1, j*3+1) = linearQuat.m_x;
      m_positionData(i+1, j*3+2) = linearQuat.m_y;
      m_positionData(i+1, j*3+3) = linearQuat.m_z;
    }
  }
  
}

SMRSkeleton 
SMRPCALinearizedQuaternion::toEigenPosture(ColumnVector _v)
{
  SMRSkeleton skel = meanSkeleton; // initialize structure
  for (unsigned int i = 0; i < skel.getNumJoints();i++)
    skel.getJoint(i)->setOrientation(Exp(SMRVector3(_v(i*3+1),_v(i*3+2),_v(i*3+3))));
  return skel;
}

SMRTimeSerie<>
SMRPCALinearizedQuaternion::toMotionCurve(RowVector _v)
{
  SMRTimeSerie<> s; // initialize structure
  for (int i = 1; i <= _v.ncols();i++)
    s.getSerie().push_back(_v(i));
  return s;
}

void 
SMRPCALinearizedQuaternion::computeEigenPostures()
{
  DiagonalMatrix diag; // matrix of eigen values
  Matrix U; // Matrix of eigenPostures 
  Matrix V; // Matrix of Motion curves
  
  // perform SVD on data
  try
  {
    SVD(m_positionData,diag,U,V);
  }
  catch(BaseException) { 
    cout << "Error in decomposition. Reason : " << endl;
    cout << BaseException::what() << endl; 
  }
  
  //output result
  //cout << "U : " << U.ncols() << " " << U.nrows() << endl; 
  //cout << "V : " << V.ncols() << " " << V.nrows() << endl; 
  //// diagonal matrix
  //cout << setprecision(3) << diag;  
  
  // get EigenValues from Matrix diag
  for (int e = 1; e <= diag.ncols() ; e++)
    m_eigenValues.getSerie().push_back(diag(e));
  
  // get Eigenpostures from Matrix V
  for (int i = 1 ; i <= V.nrows() ; i++)
    m_eigenPostures.push_back( toEigenPosture( V.column(i).as_column() ) );
  
  // get motion curves from Matrix U
  for (int mc = 1 ; mc <= U.ncols() ; mc++)
    m_motionCurves.push_back( toMotionCurve( U.column(mc).as_row() ) );
}

SMRSkeleton 
SMRPCALinearizedQuaternion::getProjectedPose(const vector<double> & coefs)
{
  SMRVector3 linquat,v;
  SMRSkeleton skelFrame = meanSkeleton;
  for (unsigned int i=0; i <skelFrame.getNumJoints();i++)
  {
    v = Log( meanSkeleton.getJoint(i)->getOrientation() );
    for (unsigned int n=0;n<coefs.size();n++)
    {
      SMRSkeleton & ep = getEigenPosture(n);
      // add the contribution of each desired components of the motion
      linquat = Log(ep.getJoint(i)->getOrientation());
      // add every projected components of the projection of the quaternion in tangent space
      v = v + linquat*coefs.at(n) * m_eigenValues[n];
    }
    // build final quaternions by exponentiation
    skelFrame.getJoint(i)->setOrientation( Exp( v ) );
  }
  return skelFrame;
}


SMRMotion 
SMRPCALinearizedQuaternion::getProjectedMotion(unsigned int _num)
{
  SMRMotion result;
  SMRSkeleton skelFrame = meanSkeleton;
  SMRQuaternion qi; SMRVector3 linquat,v, pos;
  for (unsigned int i=0; i <m_motion.getNumFrames();i++)
  {
    // for each frame in the motion
    for (unsigned int j=0; j <skelFrame.getNumJoints();j++)
    {
      v = Log( meanSkeleton.getJoint(j)->getOrientation() );
      for (unsigned int n=0;n<_num;n++)
      {
        SMRSkeleton & ep = getEigenPosture(n);
        // add the contribution of each desired components of the motion
        linquat = Log(ep.getJoint(j)->getOrientation());
        // add every projected components of the projection of the quaternion in tangent space
        v = v + linquat*(( m_motionCurves.at(n) )[i]) * m_eigenValues[n];
      }
      // build final quaternions by exponentiation
      skelFrame.getJoint(j)->setOrientation( Exp( v ) );
      
      //// if mode is absolute, we need to manually rotate the position of the joint
      if (!m_mode)
      {
        pos = m_motion.getSkeleton(i).getJoint(j)->getPosition();
        qi = skelFrame.getJoint(j)->getOrientation()* Inverse(m_motion.getSkeleton(i).getJoint(j)->getOrientation());
        qi.rotate(pos);
        skelFrame.getJoint(j)->setPosition(pos);
      }
    }
    result.insertSkeleton(skelFrame);
    skelFrame = meanSkeleton; // for each frame the contribution is added to the mean skeleton !
  }
  return result;
}


ColumnVector skelToVector(const SMRSkeleton & _pose){
  ColumnVector result(_pose.getNumJoints()*3);
  for (unsigned int i = 0 ; i < _pose.getNumJoints() ; i++){
    SMRVector3 qlog = Log(_pose.getJoint(i)->getOrientation());
    result(i*3+1)=qlog.m_x;
    result(i*3+2)=qlog.m_y;
    result(i*3+3)=qlog.m_z;
  }
  return result;
}

SMRSkeleton 
SMRPCALinearizedQuaternion::getProjectedPose(const SMRSkeleton & _pose, unsigned int _num, const SMRSkeleton & meanPose){
  ColumnVector vresult;
  ColumnVector vpose = skelToVector(_pose);
  ColumnVector meanvpose = skelToVector(meanSkeleton);
        ColumnVector meanvpose2 = skelToVector(meanPose);
  vpose = vpose - meanvpose;
  //vpose = vpose - meanvpose2;
  vector<double> projections;

  for (unsigned int i = 0 ; i < meanSkeleton.getNumJoints() ; i ++){
                double a = DotProduct(vpose , (skelToVector( getEigenPosture(i)) ));
                //a = a / m_eigenValues[i];
    projections.push_back(a);
  }
  vresult = meanvpose;
        //vresult = meanvpose2;
  for (unsigned int j=0; j <_num;j++)
    vresult += projections[j] * skelToVector( getEigenPosture(j) );  
  return toEigenPosture(vresult);
}

void
SMRPCALinearizedQuaternion::saveProjectedPose(const SMRMotion & _motion, const SMRSkeleton & meanPose, char* fileName)
{
    SMRTimeSerie<double> pose;
    RowVector projections(meanSkeleton.getNumJoints());
    ColumnVector vresult;
    ColumnVector meanvpose = skelToVector(meanSkeleton);

  for (unsigned int j = 0 ; j < _motion.getNumFrames(); j++)
  {
    ColumnVector vpose = skelToVector(_motion.getSkeleton(j));
    vpose = vpose - meanvpose;
  for (unsigned int i = 0 ; i < meanSkeleton.getNumJoints() ; i ++){
                double a = DotProduct(vpose , (skelToVector( getEigenPosture(i)) ));
    projections(i+1) += a*a;
  }
  }
  projections = projections / _motion.getNumFrames();

        pose = toMotionCurve(projections);
        pose.dumpSerieToFile(fileName);
}

SMRMotion 
SMRPCALinearizedQuaternion::getProjectedMotion(SMRMotion & _motion, unsigned int _num){
        _motion.changeMode(m_mode);
        SMRSkeleton newSkeleton;
        SMRSkeleton meanPose = _motion.getMean();
  SMRMotion result;
  for (unsigned int i = 0 ; i < _motion.getNumFrames() ; i++)
          {
            newSkeleton = getProjectedPose(_motion.getSkeleton(i),_num,meanPose);
            (newSkeleton.getNonConstJoint(0))->setPosition(_motion.getSkeleton(i).getJoint(0)->getPosition());
            (newSkeleton.getNonConstJoint(0))->setOrientation(_motion.getSkeleton(i).getJoint(0)->getOrientation());
            result.insertSkeleton( newSkeleton );
          }
  return result;
}

