/**
*  \ingroup SmrCore
*  \file SMRJoint.h
*/
#ifndef SMRJOINT_H
#define SMRJOINT_H
//#pragma once

#include "SmrMath.h"
#include "SmrSTL.h"

#include "SmrVector3.h"
#include "SmrQuaternion.h"

#include <cstring>

/**
*  \class SMRJoint
*  \brief Represent a joint articulation in a BVH motion capture file data structure.
*/
class SMRJoint
{
public:
  /**
  *  \brief Constructor.
  *  \param endJoint [in] Specifies whether the joint is an end joint or not.
  */
  SMRJoint(bool endJoint = false);

  /**
  *  \brief Copy constructor.
  *  \param joint [in] Source joint to copy from.
  */
  SMRJoint(const SMRJoint & joint);

  /**
  *  \brief Destructor.
  */
  virtual ~SMRJoint();

  /**
  *  \brief Gets the joint's name.
  *  \return The name of the joint.
  */
  inline const string getName() const
  {
    return m_name;
  }

  /**
  *  \brief Gets the name of the joint's parent.
  *  \return The name of the joint's parent.
  */
  inline const string getParentName() const
  {
    return m_parent;
  }

  inline bool hasParent()
  {
    return strcmp(m_parent.c_str(),"")!=0;
  }

  /**
  *  \brief Sets the name of the parent joint.
  *  \param name [in] Name of the parent joint.
  */
  inline void setParentName(const string &name)
  {
    m_parent=name;
  }

  /**
  *  \brief Sets the position of the joint.
  *  \param x [in] The x-component of the position.
  *  \param y [in] The y-component of the position.
  *  \param z [in] The z-component of the position.
  */
  void setPosition(double x, double y, double z);

  /**
  *  \brief Sets the position of the joint.
  *  \param position [in] Position vector. 
  */
  void setPosition(const SMRVector3 position);

  /**
  *  \brief Sets the orientation quaternion.
  *  \param x [in] The x-component of the orientation quaternion.
  *  \param y [in] The y-component of the orientation quaternion.
  *  \param z [in] The z-component of the orientation quaternion.
  */
  void setOrientation(double x, double y, double z);

  /**
  *  \brief Sets the orientation quaternion.
  *  \param orientation [in] Orientation quaternion.
  */
  void setOrientation(SMRQuaternion orientation);

  /**
  *  \brief Gets the joint's position.
  *  \return The position vector of the joint.
  */
  inline const SMRVector3 & getPosition() const
  {
    return m_position;
  }

  /**
  *  \brief Gets the joint's orientation quaternion.
  *  \return The orientation quaternion of the joint.
  */
  inline const SMRQuaternion & getOrientation() const
  {
    return m_orientation;
  }

  /**
  *  \brief Gets the joint's end length.
  *  \return The end length vector of the joint.
  */
  inline const SMRVector3 & getEndLength() const
  {
    return m_endLength;
  }

  /**
  *  \brief Sets the joint's name.
  *  \param name [in] Name of the joint.
  */
  inline void setName(string name)
  {
    m_name = name;
  }

  /**
  *  \brief Sets the joint's end length vector.
  *  \param endLength [in] End length vector.
  */
  inline void setEndLength(SMRVector3 endLength)
  {
    m_endLength = endLength;
  }

  /**
  *  \brief Sets the joint's end length vector.
  *  \param x [in] The x-component of the end length.
  *  \param y [in] The y-component of the end length.
  *  \param z [in] The z-component of the end length.
  */
  inline void setEndLength(double x, double y, double z)
  {
    m_endLength.m_x = x;
    m_endLength.m_y = y;
    m_endLength.m_z = z;
  }

  /**
  *  \brief Determines if the joint is an end joint.
  *  \return Is end joint.
  */
  inline bool isEndJoint() const
  {
    return m_endJoint;
  }

  /**
  *  \brief Sets the end joint value.
  *  \param endJoint [in] The end joint value.
  */
  inline void setEndJoint(bool endJoint)
  {
    m_endJoint = endJoint;
  }

protected:
  /**
  *  \brief The name of the joint.
  */
  string m_name;

  /**
  *  \brief Determines if the joint is an end joint or not.
  */
  bool m_endJoint;

  /**
  *  \brief The name of the joint's parent.
  */
  string m_parent;

  /**
  *  \brief The position of the joint.
  */
  SMRVector3 m_position;

  /**
  *  \brief The orientation quaternion of the joint.
  */
  SMRQuaternion m_orientation;

  /**
  *  \brief The end length for an end joint.
  */
  SMRVector3 m_endLength;

};

/**
*  \brief Pointer to a joint.
*/
typedef SMRJoint * ptrJoint;

#endif
