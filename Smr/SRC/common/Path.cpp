#include "Path.h"

// -------------------------------------------------------------------------
// Class PathManager
//
// Defintion of member methods
// -------------------------------------------------------------------------

// Singleton initialization.  At first, there is no object created.
PathManager *PathManager::_singleton = NULL;

PathManager::PathManager( ) 
{
}

PathManager *
PathManager::getInstance( void )
{
	if( _singleton == NULL ) {
		_singleton = new PathManager;
	}

	return _singleton;
}

void 
PathManager::kill( void )
{
	if( _singleton ) {
		delete _singleton;
		_singleton = NULL;
	}
}

string 
PathManager::getFileName ( const string & _name)
{
	return getFilePath(_name) + _name ;
}

string 
PathManager::getFilePath ( const string & _name)
{
	string path;
	// first try fully qualified path
	{
		ifstream file(_name.c_str() , ios::in );
		if(!file.fail())
    {     
		  file.close();
      return "";
    }
	}
	// if not working try paths stored in the PathManager
	for(unsigned int i=0; i < m_path.size(); i++)
	{
		path = m_path[i];
		ifstream file( (m_path[i] + "/" + _name).c_str() , ios::in );
		if(!file.fail())
    {     
			file.close();
      path = path + "/";
      return path;
    }
	}
	// It seems the file does not exist in the path. Damn it !
	cerr << " *** Couldn't find " << _name << endl;
	return ".";
}
	
