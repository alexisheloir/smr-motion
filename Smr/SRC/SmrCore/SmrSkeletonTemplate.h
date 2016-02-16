/**
*  \ingroup SmrCore
*  \file SMRSkeletonTemplate.h
*/
#ifndef SMRSKELETONTEMPLATE_H
#define SMRSKELETONTEMPLATE_H

#include "SmrSTL.h"
#include "SmrSmartPointer.h"
#include "SmrQuaternion.h"
#include "SmrVector3.h"
#include "SmrJoint.h"
#include "SmrKinematicJoint.h"
#include "logger.h"
/**
*  \brief definition of the two possible modes of a skeleton : ABSOLUTE or RELATIVE.
*  A skeleton can be expressed in a relative hierarchical fashion or in an absolute flat fashion 
*/
typedef enum {ABSOLUTEMODE=0,RELATIVEMODE=1} SMRModeType;

/**
*  \brief Definition of the translation/rotation relative ordering for a skeleton.
*  Some skeletal structure convention assume the ROTATIONFIRST mode (asf/amc) where the rotational component of a joint is applied before the joint's offset whereas other skeletal structure conventions (bvh) assume the TRANSLATIONFIRST mode where the offset component of a joint is applied after the joint's offset.
*/
typedef enum {ROTATIONFIRST=0,TRANSLATIONFIRST=1} SMRTransformationOrderType;

/**
*  \class SMRSkeletonT
*  \brief This class provides a support for generic skeleton.
*  The generic class SMRSkeletonT provides support for handling articluated skeletal structures. This generic class is instanciated according to the type of joint it contains, this can be a standard joint with a single orientation or a kinematic joint with several degree of freedom.
*/
template <typename JOINT>
class SMRSkeletonT
{
public:
  /**
  *..\brief Default constructor.
  */
  SMRSkeletonT(string name="");

  /**
  *..\brief Constructor.
  *..\param mode should the representation be ABSOLUTE or RELATIVE
  *..\param rotationOrder should rotation order be ROTATIONFIRST or TRANSLATIONFIRST
  *..\param name name of the newly created skeleton
  */
  SMRSkeletonT(SMRModeType mode, SMRTransformationOrderType rotationOrder, string name="");

  /**
  *..\brief Constructor for python binding, without SMRModeType and SMRTransformationOrderType type definitions.
  *..\param mode should the representation be ABSOLUTE or RELATIVE
  *..\param rotationOrder should rotation order be ROTATIONFIRST or TRANSLATIONFIRST
  *..\param name name of the newly created skeleton
  */
  SMRSkeletonT(bool mode, bool rotationOrder, string name="");

  /**
  *..\brief Copy constructor.
  *..\param skeleton skeleton instance to be copied
  */
  SMRSkeletonT(const SMRSkeletonT<JOINT> & skeleton);

  /**
  *..\brief Return a copy of this skeleton.
  *  \return A copy of *this.
  */
  SMRSkeletonT<JOINT>* clone();


  /**
  *  \brief Destructor.
  */
  virtual ~SMRSkeletonT();

  /**
  *  \brief return the skeleton's name.
  *  return the skeleton's name.
  *  \return the skeleton's name.
  */
  inline string getName() const
  {
    return m_name;
  }

  /**
  *  \brief return the skeleton's name.
  *  return the skeleton's name.
  *  \return the skeleton's name.
  */
  void setName(const string _name)
  {
    m_name = _name;
  }


  /**
  *  \brief Modify skeleton's bindPose.
  *  The bindPose of am articulated structure is its actual configuration when all it's joint rotations are set to identity. By geometric analogy, it is also called T-pose.
  *  Change the bindPose according to the bind pose that is passed into parameter.
  *  \param _bindPose a skeleton whose configuration will be used as the new bindPose *this and _bindPose shall therefore share the same topology.
  */
  void changeBindPose(const SMRSkeletonT<JOINT> &_bindPose);

  /**
  *  \brief Rotate the skeleton around its root joint.
  *  Rotates the skeleton from it's root joint according to the rotation specified in the quaternion passed as parameter.
  *  \param rot a quaternion specifying the desired rotation arount skeleton's root joint.
  */
  void rotate(const SMRQuaternion &rot);

  /**
  *  \brief Get this skeleton's mode (ABSOLUTE or RELATIVE).
  *  get this skeleton's mode (ABSOLUTE or RELATIVE).
  *  \return skeleton's mode.
  */
  inline SMRModeType getMode() const
  {
    return m_mode;
  }

  /**
  *  \brief Get this skeleton's rotation order (ROTATIONFIRST or TRANSLATIONFIRST).
  *  Get this skeleton's rotation order (ROTATIONFIRST or TRANSLATIONFIRST).
  *  \return skeleton's rotation order.
  */
  SMRTransformationOrderType getRotationOrder() const
  {
    return m_rotationOrder;
  }

  /**
  *  \brief Get a joint by its index value.
  *  get a joint by its index value.
  *  \param index the index of the desired joint
  *  \return reference towards the desired joint
  */
  JOINT*  getJoint(const unsigned int index ) const
  { 
    if (index < getNumJoints())
    {
      return m_joints.at(index);
    }
    else
    {
	  //Only run logging code if code is not being compiled by Unreal 
	  #ifndef UNREAL_SMR_BINDING
      LOG_FATAL(logger,"SMRSkleton: joint index out of range");
	  #endif
      exit(1);
    }
  }

  /**
  *  \brief Get a joint by its index value (non const version)
  *  Get a joint by its index value (non const version)
  *  \param index the index of the desired joint
  *  \return reference towards the desired joint
  */
  JOINT* getNonConstJoint(const unsigned int index )
  {
    return m_joints.at(index);
  }

  /**
  * \brief Get the root joint.
  * Get the root joint (convenience function)
  * \return reference towards the root joint
  */
  JOINT*  getRootJoint() const
  {
    return m_joints.at(0);
  }

  /**
  * \brief Get the root joint (non const version).
  * Get the root joint (non const version).
  * \return reference towards the root joint
  */
  JOINT*  getNonConstRootJoint()
  {
    return m_joints.at(0);
  }

  /**
  *  \brief Get a joint by its name.
  *  Get a joint by its name.
  *  \param name name of the desired joint
  *  \return reference towards the desired joint
  */
  JOINT*  getJointByName( const string & name ) const;

  /**
  *  \brief Get the index of a joint from it's name.
  *  Get the index of a joint from it's name.
  *  \param name the name of the joint whose desired index is sought
  *  \return joint's index
  */
  int getJointIndex( const string & name ); 

  /**
  *  \brief Insert a joint at a given index.
  *  Insert a joint at a given index.
  *  \param index where to insert the joint
  *  \param joint joint reference to be inserted
  */
  void insertJoint(const unsigned int index,JOINT* joint );

  /**
  *  \brief Enqueue a joint.
  *  Enqueue a joint (end of the list)
  *  \param joint joint reference to be inserted
  */
  void insertJoint(JOINT* joint);

  /**
  *  \brief Remove a joint.
  *  Remove a joint at a given index from the beginning
  *  \param index index of joint to be removed
  */
  void removeJoint( unsigned int index);

  /**
  *  \brief Invert all joints rotations.
  */
  void inverse();

  /**
  *  \brief Get total number of joints.
  *  Gets total number of joints
  *  \return total number of joints
  */
  inline unsigned int getNumJoints() const
  {
    return static_cast<unsigned int>(m_joints.size());
  }

  /**
  *  \brief set up the flags for a skeleton's terminal joints.
  *..go through all the joints in the collection and checks which joints are end-effector joints (without children)
  */
  void checkEndJoints();

  /**
  *  \brief Set skeleton mode (ABSOLUTE or RELATIVE).
  *  Change mode
  *  \param mode desired mode (ABSOLUTEMODE or RELATIVEMODE)
  */
  void setMode(SMRModeType mode);

private:
  /**
  *  \brief Change skeleton mode to absolute mode.
  *  Change skeleton mode to absolute mode.
  */
  void switchToAbsolute();

  /**
  *  \brief Change skeleton mode to relative mode.
  *  Change skeleton mode to absolute mode.
  */
  void switchToRelative();

public:
  /**
  *  \brief Set skeleton's rotation order (TRANSLATIONFIRST OR ROTATIONFIRST)
  *  \param rotationOrder TRANSLATIONFIRST or ROTATIONFIRST
  */
  void setRotationOrder(SMRTransformationOrderType rotationOrder);

private:
  /**
  *  \brief Change Skeleton rotation order to ROTATIONFIRST.
  *  this methods converts rotation order from T->R to R->T, typically, from a bvh representation to a asf/amc like rotation
  *  after calling this method, a "end" joint is added at the end of each leaf ot the skeleton tree, to take the 
  *  initial translation into account.
  *  ater calling this methods, end joints become irrelevants, they are however kept (endLength=0) for consistency.
  *  this representation is more comfortable for handling inverse kinematics issues
  *  we assume endJoint are correctly set before calling this method
  *  we assume rotation are relatives before calling this method
  */
  void switchToRotationFirst();

  /**
  *  \brief Change Skeleton rotation order to TRANSLATIONFIRST
  *  this methods converts rotation order from R->T to T->R, typically, from a asf/amc representation to a bvh like
  *  representation
  *  after calling this method, a "end" joint is squized at the end of each leaf ot the skeleton tree
  *  ater calling this methods, end joints replace the lost leaf joint.
  *  we assume endJoint are correctly set before calling this method
  *  we assume rotation are relatives before calling this method
  */
  void switchToTranslationFirst();

public:
  /**
  *  \brief Get a collection of the childern of a given joint (index).
  *  returns the list of children of a given joint
  *  \return a list of joints (_numJoint children)
  */
  vector <unsigned int> getJointChildren( unsigned int numJoint );

  /**
  *  \brief Get a collection of the childern of a given joint (name).
  *  returns the list of children of a given joint
  *  \return a list of joints (_numJoint children)
  */
  vector <unsigned int> getJointChildren(  string _name );

  /**
  *  \brief Checks if the joint has children.
  */
  bool hasJointChildren(unsigned int jointIndex);

  /**
  *  \brief get a subskeleton of this skeleton (local copy).
  *  Returns the subskeleton described by the skeleton passed into parameter
  *  \param subSkeleton subskeleton template of the subskeleton users wants to retrieve
  *  return a local copy of the retrieved subskeleton
  */
  SMRSkeletonT<JOINT> getSubSkeleton(const SMRSkeletonT<JOINT> & subSkeleton) const;

  SMRSkeletonT<JOINT> & combine(const SMRSkeletonT<JOINT> & _skeleton, double _amount );


  /************ Operators overload ****************/

  /**
  *  \brief operator = overload (like copy constructor)
  */
  SMRSkeletonT<JOINT> & operator=(const SMRSkeletonT<JOINT> & skeleton);

  /**
  *  \brief operator += overload (splice two skeleton together).
  */
  SMRSkeletonT<JOINT> & operator+=(const SMRSkeletonT<JOINT> & _skeleton);

  /**
  *  \brief operator *= overload (splice two skeleton together).
  */
  SMRSkeletonT<JOINT> & operator*=(const SMRSkeletonT<JOINT> & _skeleton);

  /**
  *  \brief operator == overload (are skeletons equals).
  */
  bool operator==(const SMRSkeletonT<JOINT> & skeleton) const;

  /**
  *  \brief operator != overload (are skeletons differents)
  */
  bool operator!=(const SMRSkeletonT<JOINT> & skeleton) const;

  void flush();

protected:

  string m_name; /**< Detailed description after the member */

  SMRModeType m_mode; /**< ABSOLUTEMODE or RELATIVEMODE*/


  SMRTransformationOrderType m_rotationOrder; /**< ROTATIONFIRST or TRANSLATIONFIRST*/

  vector<JOINT*> m_joints; /**< a vector of joint references.*/

  map<string, JOINT*> m_jointMap; /**< a map of joint references key is joint name */

};//end of SMTSkeletonT class

/**
*  \brief Return a local instance of a slerp interpolated skeleton between _skeletonA and _skeletonB according to _alpha.
*..Return a local instance of a slerp interpolated skeleton between 
*.._skeletonA and _skeletonB according to _alpha (_alpha should thus be 0.0 < _alpha < 1.0).
*..\param skeletonA interpolate between this skeleton...
*..\param skeletonB ... and this skeleton
*..\return a local copy of the interpolated skeleton
*/
//template <typename JOINT>
//SMRSkeletonT<JOINT> interpolateSkeletons( const SMRSkeletonT<JOINT> & skeletonA, const SMRSkeletonT<JOINT> & skeletonB, float alpha);


template <typename JOINT>
void InterpolateSkeletons( SMRSkeletonT<JOINT> *_skel1, SMRSkeletonT<JOINT> *_skel2, SMRSkeletonT<JOINT> *_skel3, SMRSkeletonT<JOINT> *_skel4,  SMRSkeletonT<JOINT> &_interpolatedSkeleton,\
                          float _alpha, float _tension=0.0,float _bias=0.0, float _continuity=0.0 );

/**
*  \brief Interpolate between _skeletonA and _skeletonB. modify _skelOut.
*  Interpolate between _skeletonA and _skeletonB. modify _skelOut.
*  \param skeletonA interpolate between this skeleton...
*  \param skeletonB ... and this skeleton
*  \param skelOut The resulting skeleton
*/
template <typename JOINT>
void interpolateSkeletons( const SMRSkeletonT<JOINT> & skeletonA, const SMRSkeletonT<JOINT> & skeletonB, SMRSkeletonT<JOINT> *  skelOut, float alpha);
/**
*  \brief new interpolation over skeleton vector, uses parameters bias, tension and continuity to modify path
*/
//template <typename JOINT>
//void
//interpolateSkeletons( const vector<SMRSkeletonT<JOINT>> & _keyVector, vector<SMRSkeletonT<JOINT>> * _resultVector,float _steps, float _tension,float _bias, float _continuity);

//template <typename JOINT>
//void
//interpolateSkeletons( map< const float, const SMRSkeletonT<JOINT>> & _keyPoses,vector<SMRSkeletonT<JOINT>> * _resultVector, unsigned int _steps, float _tension,float _bias, float _continuity);

/**
*  \brief Compute a points cloud distance between two skeletons as specified in the Gleicher et al. Motion graph paper.
*  Compute a points cloud distance between two skeletons as specified in the Gleicher et al. Motion graph paper
*  \param skeletonA compute distance between this skeleton
*  \param skeletonB ... and this skeleton
*  \return computed distance
*/
template <typename JOINT>
double PointsCloudDistance(const SMRSkeletonT<JOINT> & skeletonA, const SMRSkeletonT<JOINT> & skeletonB);
/**
* \brief Does squad interpolation over 4 skeletons influenced by tension, bias and continuity parameters
* \param _keyPoses pair of float,skeleton every skeleton brings in his timepoint, so interpolation could use time differences to improve results (not yet)
* \param _alpha current time
* \return skeleton at time point alpha
*/
template <typename JOINT>
void 
interpolateSkeletons( map<const float,const SMRSkeletonT<JOINT>* > & _keyPoses, SMRSkeletonT<JOINT> &_interpolatedSkeleton, float _alpha, float _tension,float _bias, float _continuity);
// implementation file
#include "SmrSkeletonTemplate.incl"

#endif
