/**
 * \ingroup SMRRenderer
 * \file SMRBasicRenderer.h
 * 
 */

#ifndef _SMR_BASIC_RENDERER__H_
#define _SMR_BASIC_RENDERER__H_
#pragma once

#include "SmrRenderer.h"
#include "Shaders.h"

/**
 * \class SMRBasicRenderer
 * \brief this class inherits from the generic
 * SMRRenderer and implements a implements a basic renderer 
 * that displays a skeleton as a mixture of different basic
 * shapes
 */


class SMRBasicRenderer : public SMRRenderer
{
public: 
	/*
	* type of shader
	*/
	typedef enum {toon, gooch} typeShader;

protected:
	typeShader m_renderStyle;

public:
	/**
	* Setter
	* @param _style is the type of shader used
	*/
	inline void setRenderStyle(typeShader _style) {m_renderStyle=_style;}
public:
	/**
	* Constructor
	*/
	SMRBasicRenderer(SMRSkeleton * _squel=NULL);
	~SMRBasicRenderer();

// implementation method
public:
	/**
	* render the model within the current GL context
	* allows to implement SMRRenderer interface
	*/
	void draw();
};

#endif
