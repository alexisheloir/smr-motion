/**
 * \ingroup SMRRenderer
 * \file SMRSimpleRenderer.h
 * 
 */

#ifndef _SMR_SIMPLE_RENDERER__H_
#define _SMR_SIMPLE_RENDERER__H_
#pragma once

#include "SmrRenderer.h"

/**
 * \class SMRSimpleRenderer
 * \brief this class inherits from the generic
 * SMRRenderer class and implements a basic renderer 
 * that displays a skeleton in a wire rendering style fashion
 */


class SMRSimpleRenderer : public SMRRenderer
{
public:
	/**
	* Constructor
	*/
	SMRSimpleRenderer(SMRSkeleton * _squel=NULL);
private:
	float m_colorR;float m_colorG;float m_colorB;
	float m_colorArticulationR;float m_colorArticulationG;float m_colorArticulationB;
// implementation method
public:
	/**
	* render the model within the current GL context
	* allows to implement SMRRenderer interface
	*/
	void draw ();
	/**
	* set default colors for the renderer
	*/
	void setDefaultColors();
	/**
	* set colors for the renderer
	*/
	void setColors(float jointR,float jointG,float jointB,
				   float articulationR,float articulationG,float articulationB);

};

#endif
