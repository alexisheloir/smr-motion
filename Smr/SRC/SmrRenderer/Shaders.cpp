#include "Shaders.h"

// Forward declarations
static void printInfoLog_2_0( GLuint obj );
static GLuint loadShaders_2_0( const string &vert, const string &frag );
static GLchar *loadShaderFile_2_0( const string &filename );

static void printInfoLog_1_5( GLhandleARB obj );
static GLhandleARB loadShaders_1_5( const string &vert, const string &frag );
static GLcharARB *loadShaderFile_1_5( const string &filename );

// -------------------------------------------------------------------------
// Class ShaderManager
//
// Defintion of member methods
// -------------------------------------------------------------------------

// Singleton initialization.  At first, there is no object created.
ShaderManager *ShaderManager::_singleton = NULL;

ShaderManager::ShaderManager( void ) : 
	glslSupported(false)
{
	InitGLSL();
}

ShaderManager *
ShaderManager::getInstance( void )
{
	if( _singleton == NULL ) {
		_singleton = new ShaderManager;
	}

	return _singleton;
}

void 
ShaderManager::kill( void )
{
	if( _singleton ) {
		delete _singleton;
		_singleton = NULL;
	}
}

void 
ShaderManager::releaseShaders()
{
	for( ShaderMap::iterator itor = m_shaderMap.begin(); itor != m_shaderMap.end(); ++itor ) 
	{
		GLuint prog = itor->second;
		// Remove program
		if( prog ) {
			if( GLEW_VERSION_2_0 ) {
				if( glIsProgram( prog ) ) {
					glDeleteProgram( prog );
				}
			}
			else if( GLEW_VERSION_1_5 ) {
				GLint type;
				glGetObjectParameterivARB( prog, GL_OBJECT_TYPE_ARB, &type );

				if( type == GL_PROGRAM_OBJECT_ARB ) {
					glDeleteObjectARB( prog );
				}
			}
		}
	}

	m_shaderMap.clear();
}
	
// Initialize GLSL, i.e. check if GLSL is supported and initialize
// extensions if needed.

void 
ShaderManager::InitGLSL( void )
{
  // Retrieve OpenGL extension list
  const string glExts = reinterpret_cast<const char*>
	(glGetString( GL_EXTENSIONS ));

  // Look for GLSL support
  int missing = 0;

  if( !GLEW_ARB_shader_objects ) {
    cerr << "* missing GL_ARB_shader_object extension" << endl;
    missing++;
  }

  if( !GLEW_ARB_shading_language_100 ) {
    cerr << "* missing GL_ARB_shadin_language_100 extension" << endl;
    missing++;
  }

  if( !GLEW_ARB_vertex_shader ) {
    cerr << "* missing GL_ARB_vertex_shader extension" << endl;
    missing++;
  }

  if( !GLEW_ARB_fragment_shader ) {
    cerr << "* missing GL_ARB_fragment_shader extension" << endl;
    missing++;
  }

  if( missing > 0 ) {
    cout << "* GLSL disabled" << endl;
    glslSupported = false;
  }
  else {
    glslSupported = true;
  }

  //printShaderConstants();
}


GLuint 
ShaderManager::loadShaders( const string &vert, const string &frag , const string & progname)
{
	// id of the prog
	GLuint progId;

	if( GLEW_VERSION_2_0 ) {
		progId = loadShaders_2_0( vert, frag );
		m_shaderMap[progname] =  progId;
		return progId;
	}
	else if( GLEW_VERSION_1_5 ) {
		progId = static_cast<GLuint>(loadShaders_1_5( vert, frag ));
		m_shaderMap[progname] =  progId;
		return progId;
	}
	else {
		return 0;
	}
}



// -------------------------------------------------------------------------
// printShaderConstants
//
// Print some constants related to GLSL.
// -------------------------------------------------------------------------

void printShaderConstants( void )
{
  GLint param;

  glGetIntegerv( GL_MAX_VERTEX_ATTRIBS, &param );
  cout << "GL_MAX_VERTEX_ATTRIBS: " << param << endl;

  glGetIntegerv( GL_MAX_VERTEX_UNIFORM_COMPONENTS, &param );
  cout << "GL_MAX_VERTEX_UNIFORM_COMPONENTS: " << param << endl;

  glGetIntegerv( GL_MAX_VARYING_FLOATS, &param );
  cout << "GL_MAX_VARYING_FLOATS: " << param << endl;

  glGetIntegerv( GL_MAX_VERTEX_TEXTURE_IMAGE_UNITS, &param );
  cout << "GL_MAX_VERTEX_TEXTURE_IMAGE_UNITS: " << param << endl;

  glGetIntegerv( GL_MAX_COMBINED_TEXTURE_IMAGE_UNITS, &param );
  cout << "GL_MAX_COMBINED_TEXTURE_IMAGE_UNITS: " << param << endl;

  glGetIntegerv( GL_MAX_TEXTURE_IMAGE_UNITS, &param );
  cout << "GL_MAX_TEXTURE_IMAGE_UNITS: " << param << endl;

  // This one is not really from GLSL but still usefull
  glGetIntegerv( GL_MAX_TEXTURE_UNITS, &param );
  cout << "GL_MAX_TEXTURE_UNITS: " << param << endl;

  glGetIntegerv( GL_MAX_TEXTURE_COORDS, &param );
  cout << "GL_MAX_TEXTURE_COORDS: " << param << endl;

  glGetIntegerv( GL_MAX_FRAGMENT_UNIFORM_COMPONENTS, &param );
  cout << "GL_MAX_FRAGMENT_UNIFORM_COMPONENTS: " << param << endl;
}


// -------------------------------------------------------------------------
// GLSL API Wrapper.
// -------------------------------------------------------------------------

void 
printInfoLog( GLuint obj )
{
  if( GLEW_VERSION_2_0 ) {
	printInfoLog_2_0( obj );
  }
  else if( GLEW_VERSION_1_5 ) {
	printInfoLog_1_5( static_cast<GLhandleARB>(obj) );
  }
}


static GLchar *loadShaderFile( const string &filename )
{
  if( GLEW_VERSION_2_0 ) {
	return loadShaderFile_2_0( filename );
  }
  else if( GLEW_VERSION_1_5 ) {
	return loadShaderFile_1_5( filename );
  }
  else {
	return NULL;
  }
}


/////////////////////////////////////////////////////////////////////////////
//
// OpenGL 2.0 GLSL Support
//
/////////////////////////////////////////////////////////////////////////////


// -------------------------------------------------------------------------
// loadShaderFile_2_0
//
// Load shader's GLSL code from file.  Return a pointer on a string
// which must be then deleted!
// -------------------------------------------------------------------------

GLchar *loadShaderFile_2_0( const string &filename )
{
  char *buffer = NULL;

  // Open file
  ifstream file( filename.c_str(), ios::in);

  if( file.fail() ) {
	cerr << "Error: couldn't open file \"" << filename << "\"!" << endl;
	return NULL;
  }

  // Get file length

  stringstream buf;
  buf << file.rdbuf();
  file.seekg( 0, ios::beg );
  long length = buf.str().size();


  try {
	buffer = new GLchar[ length ];

	// Read whole shader code
	file.read( buffer, length );
	file.close();	
	// Close the code string with the null char
	buffer[ length - 1 ] = '\0';
  }
  catch( std::bad_alloc &err ) {
	cerr << "Error: memory allocation failed for \""
		 << filename << "\"" << endl;
	cerr << " Reason: " << err.what() << endl;

	file.close();
  }

  return buffer;
}


// -------------------------------------------------------------------------
// printInfoLog_2_0
//
// Print log info about a shader or program.
// -------------------------------------------------------------------------

static void printInfoLog_2_0( GLuint obj )
{
  GLint infologLength = 0;

  // Get log's length
  if( glIsShader( obj ) ) {
	glGetShaderiv( obj, GL_INFO_LOG_LENGTH, &infologLength );
  }
  else if( glIsProgram( obj ) ) {
	glGetProgramiv( obj, GL_INFO_LOG_LENGTH, &infologLength );
  }
  else {
	cerr << "printInfoLog: invalid shader/program handle" << endl;
	return;
  }

  // Print log if not empty
  if( infologLength > 1 ) {
	try {
	  GLchar *infoLog = new GLchar[ infologLength ];

	  // Get the log...
	  if( glIsShader( obj ) ) {
		glGetShaderInfoLog( obj, infologLength, NULL, infoLog );

		// ...and print it
		cout << "Shader InfoLog (" << infologLength << "):" << endl;
		cout << infoLog << endl;
	  }
	  else if( glIsProgram( obj ) ) {
		glGetProgramInfoLog( obj, infologLength, NULL, infoLog );

		// ...and print it
		cout << "Program InfoLog (" << infologLength << "):" << endl;
		cout << infoLog << endl;
	  }

	  delete [] infoLog;
	}
	catch( std::bad_alloc &err ) {
	  cerr << "Error: memory allocation failed for "
		   << "shader/program info log" << endl;
	  cerr << " Reason: " << err.what() << endl;
	}
  }
}


// -------------------------------------------------------------------------
// loadShaders_2_0
//
// Load, compile and link vertex and fragment shaders.  Return a handle
// to the new shader program or 0 if an error occurs.
// -------------------------------------------------------------------------

static GLuint loadShaders_2_0( const string &vert, const string &frag )
{
  if( !isGLSLSupported() ) {
	return 0;
  }

  GLint vertCompiled, fragCompiled;
  GLint linked;

  // Load vertex and fragment shaders
  GLchar *vertexCode = loadShaderFile( vert );
  GLchar *fragmentCode = loadShaderFile( frag );

  if( !vertexCode ) {
	return 0;
  }

  if( !fragmentCode ) {
	delete [] vertexCode;
	return 0;
  }

  GLuint vertexShader = glCreateShader( GL_VERTEX_SHADER );
  GLuint fragmentShader = glCreateShader( GL_FRAGMENT_SHADER );

  // Upload vertexand fragment shader code to OpenGL
  glShaderSource( vertexShader, 1, const_cast<const GLchar **>(&vertexCode), NULL );
  glShaderSource( fragmentShader, 1, const_cast<const GLchar **>(&fragmentCode), NULL );

  // We don't need sahder code buffers anymore...
  delete [] vertexCode;
  delete [] fragmentCode;

  // Compile vertex shader
  glCompileShader( vertexShader );
  glGetShaderiv( vertexShader, GL_COMPILE_STATUS, &vertCompiled );
  printInfoLog( vertexShader );

  if( !vertCompiled ) {
	glDeleteShader( vertexShader );
	return 0;
  }
  else {
#ifdef TRACE
	cout << "* Vertex shader \"" << vert << "\" loaded" << endl;
#endif
  }

  // Compile fragment shader
  glCompileShader( fragmentShader );
  glGetShaderiv( fragmentShader, GL_COMPILE_STATUS, &fragCompiled );
  printInfoLog( fragmentShader );

  if( !fragCompiled ) {
	glDeleteShader( vertexShader );
	glDeleteShader( fragmentShader );
	return 0;
  }
  else {
#ifdef TRACE
	cout << "* Fragment shader \"" << frag << "\" loaded" << endl;
#endif
  }

  // Create program and attach vertex and fragment shaders
  GLuint program = glCreateProgram();
  glAttachShader( program, vertexShader );
  glAttachShader( program, fragmentShader );

  // Flag shaders for deletion when the program will
  // be deleted...
  glDeleteShader( vertexShader );
  glDeleteShader( fragmentShader );

  // Link vertex and fragment shaders between them
  glLinkProgram( program );
  glGetProgramiv( program, GL_LINK_STATUS, &linked );
  printInfoLog( program );

  // Check for success
  if( !linked ) {
	glDeleteProgram( program );
	return 0;
  }
  else {
#ifdef TRACE
	cout << "* Shader object successfully loaded" << endl;
#endif
  }

  // Validate program
  glValidateProgram( program );

  printInfoLog( program );

  return program;
}


/////////////////////////////////////////////////////////////////////////////
//
// OpenGL 1.5 GLSL Support
//
/////////////////////////////////////////////////////////////////////////////


// -------------------------------------------------------------------------
// loadShaderFile_1_5
//
// Load shader's GLSL code from file.  Return a pointer on a string
// which must be then deleted!
// -------------------------------------------------------------------------

static GLcharARB *loadShaderFile_1_5( const string &filename )
{
  ifstream file;
  GLcharARB *buffer = NULL;

  // Open file
  file.open( filename.c_str(), ios::in );

  if( file.fail() ) {
	cerr << "Error: couldn't open file \"" << filename << "\"!" << endl;
	return NULL;
  }

  // Get file length
  file.seekg( 0, ios::end );
  long length = file.tellg();
  file.seekg( 0, ios::beg );

  try {
	buffer = new GLcharARB[ length ];

	// Read whole shader code
	file.read( reinterpret_cast<GLcharARB *>(buffer), length );
	file.close();

	// Close the code string with the null char
	buffer[ length - 1 ] = '\0';
  }
  catch( std::bad_alloc &err ) {
	cerr << "Error: memory allocation failed for \""
		 << filename << "\"" << endl;
	cerr << " Reason: " << err.what() << endl;

	file.close();
  }

  return buffer;
}


// -------------------------------------------------------------------------
// printInfoLog_1_5
//
// Print log info about a shader or program.
// -------------------------------------------------------------------------

static void printInfoLog_1_5( GLhandleARB obj )
{
  GLint infologLength = 0;

  GLint type;
  glGetObjectParameterivARB( obj, GL_OBJECT_TYPE_ARB, &type );

  if( (type == GL_PROGRAM_OBJECT_ARB) ||
	  (type == GL_SHADER_OBJECT_ARB) ) {
	// Get log's length
	glGetObjectParameterivARB( obj, GL_OBJECT_INFO_LOG_LENGTH_ARB, &infologLength );
  }
  else {
	cerr << "printInfoLog: invalid shader/program handle" << endl;
	return;
  }

  // Print log if not empty
  if( infologLength > 1 ) {
	try {
	  GLcharARB *infoLog = new GLcharARB[ infologLength ];

	  // Get the log...
	  glGetInfoLogARB( obj, infologLength, NULL, infoLog );

	  if( type == GL_PROGRAM_OBJECT_ARB ) {
		cout << "Program InfoLog (" << infologLength << "):" << endl;
	  }
	  else {
		cout << "Shader InfoLog (" << infologLength << "):" << endl;
	  }

	  // ...and print it
	  cout << infoLog << endl;

	  delete [] infoLog;
	}
	catch( std::bad_alloc &err ) {
	  cerr << "Error: memory allocation failed for "
		   << "shader/program info log" << endl;
	  cerr << " Reason: " << err.what() << endl;
	}
  }
}


// -------------------------------------------------------------------------
// loadShaders_1_5
//
// Load, compile and link vertex and fragment shaders.  Return a handle
// to the new shader program or 0 if an error occurs.
// -------------------------------------------------------------------------

static GLhandleARB loadShaders_1_5( const string &vert, const string &frag )
{
  if( !isGLSLSupported() ) {
	return 0;
  }

  GLint vertCompiled, fragCompiled;
  GLint linked;

  // Load vertex and fragment shaders
  GLcharARB *vertexCode = loadShaderFile( vert );
  GLcharARB *fragmentCode = loadShaderFile( frag );

  if( !vertexCode ) {
	return 0;
  }

  if( !fragmentCode ) {
	delete [] vertexCode;
	return 0;
  }

  GLhandleARB vertexShader = glCreateShaderObjectARB( GL_VERTEX_SHADER_ARB );
  GLhandleARB fragmentShader = glCreateShaderObjectARB( GL_FRAGMENT_SHADER_ARB );

  // Upload vertexand fragment shader code to OpenGL
  glShaderSourceARB( vertexShader, 1, const_cast<const GLchar **>(&vertexCode), NULL );
  glShaderSourceARB( fragmentShader, 1, const_cast<const GLchar **>(&fragmentCode), NULL );

  // We don't need sahder code buffers anymore...
  delete [] vertexCode;
  delete [] fragmentCode;

  // Compile vertex shader
  glCompileShaderARB( vertexShader );
  glGetObjectParameterivARB( vertexShader, GL_OBJECT_COMPILE_STATUS_ARB, &vertCompiled );
  printInfoLog( vertexShader );

  if( !vertCompiled ) {
	glDeleteObjectARB( vertexShader );
	return 0;
  }
  else {
	cout << "* Vertex shader \"" << vert << "\" loaded" << endl;
  }

  // Compile fragment shader
  glCompileShaderARB( fragmentShader );
  glGetObjectParameterivARB( fragmentShader, GL_OBJECT_COMPILE_STATUS_ARB, &fragCompiled );
  printInfoLog( fragmentShader );

  if( !fragCompiled ) {
	glDeleteObjectARB( vertexShader );
	glDeleteObjectARB( fragmentShader );
	return 0;
  }
  else {
	cout << "* Fragment shader \"" << frag << "\" loaded" << endl;
  }

  // Create program and attach vertex and fragment shaders
  GLhandleARB program = glCreateProgramObjectARB();
  glAttachObjectARB( program, vertexShader );
  glAttachObjectARB( program, fragmentShader );

  // Flag shaders for deletion when the program will
  // be deleted...
  glDeleteObjectARB( vertexShader );
  glDeleteObjectARB( fragmentShader );

  // Link vertex and fragment shaders between them
  glLinkProgramARB( program );
  glGetObjectParameterivARB( program, GL_OBJECT_LINK_STATUS_ARB, &linked );
  printInfoLog( program );

  // Check for success
  if( !linked ) {
	glDeleteObjectARB( program );
	return 0;
  }
  else {
	cout << "* Shader object successfully loaded" << endl;
  }

  // Validate program
  glValidateProgramARB( program );

  printInfoLog( program );

  return program;
}
