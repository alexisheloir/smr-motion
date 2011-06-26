/**
 * \ingroup SmrCommon
 * \file common/Shaders.h
 * Mainly defines the shader manager class
 */

#ifndef __SHADERS_H__
#define __SHADERS_H__
#pragma once 

#include <GL/glew.h>

#include "SmrSTL.h"

// Print infoLog of a shader or a program
static void printInfoLog( GLuint obj );
// Print GLSL related constants
void printShaderConstants( void );

/**
* \class ShaderManager
* \brief This class is a singleton and allows to handle in a same and unique 
* object all the shaders use by the application. 
*/

class ShaderManager
{
protected:
	ShaderManager( void );
	~ShaderManager( void ) {releaseShaders();}

public:
	// Singleton related functions
	static ShaderManager *getInstance( void );
	static void kill( void );

protected:	
	// this map is the collection of shaders maintained
	// by the Manager
	typedef map<string, GLuint> ShaderMap;
	ShaderMap m_shaderMap;

	// implementation methods
private:
	// Is GLSL supported on this platform?
	bool glslSupported;

	// Check for GLSL support
	void InitGLSL( void );
	// Print infoLog of a shader or a program
	void printInfoLog( GLuint obj );
	// release Shaders <---> free memory
	void releaseShaders();

public:
	bool isGLSLSupported(){return glslSupported;};
	// Load GLSL vertex and frament shaders
	GLuint loadShaders( const string &vert, const string &frag , const string & progname);
	// Get Shaders by their name (if they have already been loaded)
	GLuint getShaderByName( const string &shader) {return m_shaderMap[shader];};

protected:	
	// The unique instance of this class
	static ShaderManager *_singleton;
};


// --------------------------------------------------------------------------
// Non member inline functions, for easier use.
// --------------------------------------------------------------------------

/**
* load a program consisting in a vertex and a fragment shader
* @param vert Name of the vertex program
* @param frag Name of the fragment program
* @param progname Name for your program
*/
inline GLuint loadShaders( const string &vert, const string &frag , const string & progname)
{
	return ShaderManager::getInstance()->loadShaders(vert,frag,progname);
}


inline bool isGLSLSupported()
{
	return ShaderManager::getInstance()->isGLSLSupported();
}

inline GLuint getShader( const string & shadername )
{
	return ShaderManager::getInstance()->getShaderByName(shadername);
}


#endif // __SHADERS_H__
