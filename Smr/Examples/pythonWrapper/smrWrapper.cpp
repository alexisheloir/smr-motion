/**
 *
 * This file is the python wrapper for SMR
 *
 */


#include <boost/python.hpp>  // include the python binding declarations

#include "Smr.h" // include the standard Smr déclarations
#include "SmrGSMMSolver.h"

using namespace boost::python;

/**
 * Wrapper for boost.python, as type polymorphism is not supported, instances of generic classes
 * must be explicitely specified
 * class SmrVector3
 */
class SmrVector3Wrap : public SmrVector3
{
public:
  SmrVector3Wrap(): SmrVector3()
  {
  }
  SmrVector3Wrap(double x, double y, double z): SmrVector3(x,y,z)
  {
  }
  SmrVector3Wrap(const SmrVector3 &_vector): SmrVector3(_vector)
  {
  }
  SmrVector3Wrap default_normalize(void)
  {
    this->normalize();
    return *this;
  }
};

/**
 * Wrapper for boost.python, as type polymorphism is not supported, instances of generic classes
 * must be explicitely specified
 * class SmrQuaternion
 */
class SmrQuaternionWrap : public SmrQuaternion
{
public:
  SmrQuaternionWrap():SmrQuaternion()
  {
  }
  SmrQuaternionWrap( const SmrQuaternion _quat ):SmrQuaternion( _quat )
  {
  }
  SmrQuaternionWrap( const SmrVector3Wrap &_axis, double _angle ):SmrQuaternion( _axis, _angle )
  {
  }
  SmrQuaternionWrap( double _rotX, double _rotY, double _rotZ ):SmrQuaternion( _rotX, _rotY, _rotZ )
  {
  }
  SmrQuaternionWrap( double _w, double _x, double _y, double _z ):SmrQuaternion( _w, _x, _y, _z )
  {
  }
  SmrQuaternionWrap default_normalize(void)
  {
    this->normalize();
    return *this;
  }
  SmrVector3Wrap default_getRotationAxis(void)
  {
    SmrVector3Wrap axis(this->getRotationAxis());
    return axis;
  }
  float getX()
  {
    return m_x;
  }
  float getY()
  {
    return m_y;
  }
  float getZ()
  {
    return m_z;
  }
  float getW()
  {
    return m_w;
  }
};

class SmrDofWrap : public SmrDof
{
public:
  SmrDofWrap(PyObject* self): self(self), SmrDof()
  {
  }

  PyObject* self;

  void default_setRotationAxis(SmrVector3Wrap _vector3)
  {
    this->setRotationAxis(_vector3);
  }

};

class SmrKinematicJointWrap : public SmrKinematicJoint
{
public:
  SmrKinematicJointWrap(PyObject* self, bool _endJoint) : self(self), SmrKinematicJoint(_endJoint)
  {
  }

  PyObject* self;

  SmrKinematicJointWrap( const SmrKinematicJointWrap &_joint ) : SmrKinematicJoint(_joint)
  {
  }

  SmrKinematicJointWrap( const SmrKinematicJoint &_joint ) : SmrKinematicJoint(_joint)
  {
  }

  SmrQuaternionWrap default_getOrient()
  {
    SmrQuaternionWrap returnQuat(this->getOrient());
    return returnQuat;
  }

  handle<> default_getDof(const int _index)
  {
    SmrDofWrap* dofPtr = static_cast<SmrDofWrap*>(this->getDof(_index));
    assert(dofPtr != 0);
    return handle<>(borrowed(dofPtr->self));
  }

  void default_addDof(SmrDofWrap* dof)
  {
    m_dofVect.push_back(dof);;
  }


  void default_createDof(SmrVector3Wrap _axis, double _lowerBound, double _upperBound, double _value, double gain)
  {
    this->addDof(_axis, _lowerBound, _upperBound, _value, gain);
  }

  void setPos_3Doubles(double x, double y, double z)
  {
    setPos(x,y,z);
  }

  void setRot_3Doubles(double x, double y, double z)
  {
    setOrientation(x,y,z);
  }

  void setEndPos_3Doubles(double x, double y, double z)
  {
    setEndLength(x,y,z);
  }
};

class SmrJointWrap : public SmrJoint
{
public:
  SmrJointWrap(PyObject* self, bool _endJoint) : self(self), SmrJoint(_endJoint)
  {
  }

  PyObject* self;

  SmrJointWrap( const SmrJointWrap &_joint ) : SmrJoint(_joint)
  {
  }

  SmrJointWrap( const SmrJoint &_joint ) : SmrJoint(_joint)
  {
  }

  SmrQuaternionWrap default_getOrient()
  {
    SmrQuaternionWrap returnQuat(this->getOrient());
    return returnQuat;
  }

  void setPos_3Doubles(double x, double y, double z)
  {
    setPos(x,y,z);
  }

  void setRot_3Doubles(double x, double y, double z)
  {
    setOrientation(x,y,z);
  }

  void setRot_Quaternion(double w, double x, double y, double z)
  {
    setOrientation(SmrQuaternion(w,x,y,z));
  }
  void setEndPos_3Doubles(double x, double y, double z)
  {
    setEndLength(x,y,z);
  }
};

/**
 * Wrapper for boost.python, as type polymorphism is not supported, instances of generic classes
 * must be explicitely specified
 * class SmrSkeletonWrap
 */
class SmrSkeletonWrap : public SmrSkeleton
{
public:
  SmrSkeletonWrap(bool mode, bool rotOrder, string name=""):SmrSkeleton(mode,rotOrder,name)
  {
  }

  SmrJoint default_getJoint(int jointNumber)
  {
    return *(this->getJoint(jointNumber));
  }

  void default_insertJoint(SmrJoint* _joint)
  {
    return this->insertJoint(_joint);
  }

};


/**
 * Wrapper for boost.python, as type polymorphism is not supported, instances of generic classes
 * must be explicitely specified
 * class SmrKinematicChainWrap
 */
class SmrKinematicChainWrap : public SmrKinematicChain
{
public:
  SmrKinematicChainWrap(bool mode, bool rotOrder, string name="") : SmrKinematicChain(mode,rotOrder,name)
  {
  }

  handle<> default_getJoint(int jointNumber)
  {
    SmrKinematicJointWrap* jointPtr = static_cast<SmrKinematicJointWrap*>(this->getJoint(jointNumber));
    assert(jointPtr != 0);
    //return static_cast<SmrKinematicJointWrap *>(this->getJoint(jointNumber));
    SmrKinematicJointWrap returnJoint(false);
    return handle<>(borrowed(jointPtr->self));
  }

  handle<> default_getJointByName(string jointName)
  {
    SmrKinematicJointWrap* jointPtr = static_cast<SmrKinematicJointWrap*>(this->getJointByName(jointName));
    assert(jointPtr != 0);
    //return static_cast<SmrKinematicJointWrap *>(this->getJoint(jointNumber));
    SmrKinematicJointWrap returnJoint(false);
    return handle<>(borrowed(jointPtr->self));
  }

  void default_insertJoint(SmrKinematicJointWrap* _joint)
  {
    return this->insertJoint(_joint);
  }

};

class SmrIKConstraintWrap : public SmrIKConstraint 
{
public:
  SmrIKConstraintWrap()
  {
  }
  void default_setRelatedJointName(char* _jointName)
  {
    return this->setRelatedJointName(_jointName);
  }
  void default_setPosition(SmrVector3Wrap _position)
  {
    this->setPosition(_position);
  }
  void default_setOffset(SmrVector3Wrap _offset)
  {
    this->setOffset(_offset);
  }
};

class SmrGSMMSolverWrap : public SmrGSMMSolver
{
public:
  SmrGSMMSolverWrap(SmrKinematicChainWrap kinematicChain):SmrGSMMSolver(kinematicChain)
  {
  }
  void default_addConstraintPtr(SmrIKConstraintWrap * _constraintPtr)
  {
    cout << _constraintPtr->getOffset() << endl;
    this->addConstraintPtr(_constraintPtr);
  }
};

void exportPoseToBvhWrap(string _filename, SmrSkeletonWrap &_bindPose)
{
  exportPoseToBvh(_filename,_bindPose);
}

/**
 * Now, we declare the bindings like in an IDL
 */

BOOST_PYTHON_MODULE(SMRPy)
{

/**
 * Common mathematical classes
 */
  class_<SmrVector3Wrap>("SmrVector3")
    .def(init<double,double,double>())
    .def("init", &SmrVector3Wrap::init)
    .def("normalize", &SmrVector3Wrap::default_normalize)
    .def("norm", &SmrVector3Wrap::norm)
  ;

  class_<SmrQuaternionWrap>("SmrQuaternion")
    .def("getRotationAxis", &SmrQuaternionWrap::default_getRotationAxis)
    .def("getRotationAngle", &SmrQuaternionWrap::getRotationAngle)
    .def("normalize", &SmrQuaternionWrap::default_normalize)
    .def("getX", &SmrQuaternionWrap::getX)
    .def("getY", &SmrQuaternionWrap::getY)
    .def("getZ", &SmrQuaternionWrap::getZ)
    .def("getW", &SmrQuaternionWrap::getW)
  ;

/**
 * Common SmrSkeleton declarations
 */
  class_<SmrJoint>("SmrJoint", init<bool>())
    .def("getName", &SmrJoint::getName)
    .def("setName", &SmrJoint::setName)
  ;

  class_<SmrSkeletonWrap>("SmrSkeleton", init<bool, bool, std::string>())
    .def("getName",      &SmrSkeletonWrap::getName)
    .def("getNumjoints", &SmrSkeletonWrap::getNumJoints)
    .def("getJoint",     &SmrSkeletonWrap::default_getJoint)
    .def("insertJoint",  &SmrSkeletonWrap::default_insertJoint)
    //.def("getJointByName", &SmrSkeleton::getJointByName)
  ;

  class_<SmrDof, SmrDofWrap, boost::noncopyable>("SmrDof")
    .def("setRotationAxis",  &SmrDofWrap::default_setRotationAxis)
    .def("setRotationAngle", &SmrDofWrap::setRotationAngle)
    .def("getRotationAngle", &SmrDofWrap::getRotationAngle)
    .def("setUpperBound",   &SmrDofWrap::setUpperBound)
    .def("setLowerBound",   &SmrDofWrap::setLowerBound)
    .def("setGain",          &SmrDofWrap::setGain)
  ;

  class_<SmrJoint, SmrJointWrap, boost::noncopyable>("SmrJoint", init<bool>())
    .def("getName", &SmrJointWrap::getName)
    .def("setName", &SmrJointWrap::setName)
    .def("setPos", &SmrJointWrap::setPos_3Doubles)
    .def("setRot", &SmrJointWrap::setRot_3Doubles)
    .def("setRotQuat", &SmrJointWrap::setRot_Quaternion)
    .def("setEndVect", &SmrJointWrap::setEndPos_3Doubles)
    .def("getRot", &SmrJointWrap::default_getOrient)
    .def("setParentName", &SmrJointWrap::setParentName)
    .def("getParentName", &SmrJointWrap::getParentName)
    .def("setName", &SmrJointWrap::setName)
  ;
  //void (SmrKinematicJointWrap::*addDof_Vector3)(const SmrVector3 customAxis, const double lowerBound, const double _upperBound, const double _value) = &SmrKinematicJointWrap::addDof;

  class_<SmrKinematicJoint, SmrKinematicJointWrap, boost::noncopyable>("SmrKinematicJoint", init<bool>())
    .def("getName", &SmrKinematicJointWrap::getName)
    .def("setName", &SmrKinematicJointWrap::setName)
    .def("updateRot", &SmrKinematicJointWrap::updateRot)
    .def("addDof", &SmrKinematicJointWrap::default_addDof)
    .def("createDof", &SmrKinematicJointWrap::default_createDof)
    .def("getDof", &SmrKinematicJointWrap::default_getDof)
    .def("setPos", &SmrKinematicJointWrap::setPos_3Doubles)
    .def("setRot", &SmrKinematicJointWrap::setRot_3Doubles)
    .def("setEndVect", &SmrKinematicJointWrap::setEndPos_3Doubles)
    .def("getRot", &SmrKinematicJointWrap::default_getOrient)
    .def("setParentName", &SmrKinematicJointWrap::setParentName)
    .def("getParentName", &SmrKinematicJointWrap::getParentName)
    .def("setName", &SmrKinematicJointWrap::setName)
  ;

  class_<SmrKinematicChainWrap>("SmrKinematicChain", init<bool, bool, std::string>())
    .def("getName",       &SmrKinematicChainWrap::getName)
    .def("getNumjoints",  &SmrKinematicChainWrap::getNumJoints)
    .def("getJoint",      &SmrKinematicChainWrap::default_getJoint)
    .def("getJointByName",&SmrKinematicChainWrap::default_getJointByName)
    .def("insertJoint",   &SmrKinematicChainWrap::default_insertJoint)
    .def("setStartJointIndex",   &SmrKinematicChainWrap::setStartJointIndex)

    //.def("getJointByName", &SmrSkeleton::getJointByName)
  ;

/**
 *    Inverse Kinematics class binding
 */
  class_<SmrIKConstraintWrap>("SmrIKConstraint")
    .def("setRelatedJointName", &SmrIKConstraintWrap::default_setRelatedJointName)
    .def("setOffset", &SmrIKConstraintWrap::default_setOffset)
    .def("setPosition", &SmrIKConstraintWrap::default_setPosition)
  ;

  class_<SmrGSMMSolverWrap>("SmrGSMMSolver", init<SmrKinematicChainWrap>())
    .def("addConstraintPtr", &SmrGSMMSolverWrap::default_addConstraintPtr)
    .def("process",          &SmrGSMMSolverWrap::process)
    //.def("getSkeleton",      &SmrGSMMSolverWrap::getSkeleton)
  ;


/**
 *    export methods exposure
 */

  def("exportSkeletonToBvh", exportPoseToBvhWrap);
}