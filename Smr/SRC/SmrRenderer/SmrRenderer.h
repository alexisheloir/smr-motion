/// \defgroup SMRRenderer Smr rendering routines
/// \file SMRRenderer.h


#ifndef __SMRRENDERER__H__
#define __SMRRENDERER__H__
#pragma once

// OpenGL includes

#include <GL/glew.h>

#include "SmrSkeleton.h"
#include "Path.h"
/**
 * \class SMRRenderer
 * \brief this class  implements the interface for
 * classes performing skeletal animation
 */


class SMRRenderer
{
public:
	/**
	 * Constructor
	 * \param _squel : a pointer to a valid SMRSkeleton
	 */
	SMRRenderer(SMRSkeleton * _squel){m_skeleton = _squel; m_scaleFactor = 1.0f;}
	virtual ~SMRRenderer(){}
	/**
	 * set skeleton pointer
	 * \param _squel : a pointer to a valid SmrSqueletton
	 */
	void setSkeletonPtr(SMRSkeleton * _squel){m_skeleton = _squel;}
	/**
	 * set skeleton scale factor
	 * \param _scale the scale factor
	 */
	void setScaleFactor(double _scale){m_scaleFactor = _scale;}
// protected data
protected:
	/**
	 * a pointer to a valid skeleton structure
	 */
	SMRSkeleton * m_skeleton;
	/**
	 * scaleFactor
	 */
	double m_scaleFactor;

public:
	/**
	* pure virtual
	* render the model within the current GL context
	*/
	virtual void draw() = 0 ;
};


#endif
