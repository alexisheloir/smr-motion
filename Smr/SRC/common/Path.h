/**
 * \ingroup SmrCommon
 * \file Path.h
 *	mainly defines the Path Manager class
 */

#ifndef __PATH_H__
#define __PATH_H__

#include "SmrSTL.h"

/**
* \class PathManager
*
* \brief
* This class is a singleton and allows to handle multiple paths
* for file reading queries, i.e. the executable will try to find
* files in path stored in the PathManager
*/

class PathManager
{
protected:
	PathManager( void );
	~PathManager( void ) {}

public:
	// Singleton related functions
	static PathManager *getInstance( void );
	static void kill( void );

protected:	
	//  a vector of string representing path to search files from
	vector<string> m_path;

	// implementation methods
private:

public:
	/**
	* add a path to the file manager
	* @param _path string which is the path to add
	*/
	inline void addPath( const string & _path){
		m_path.push_back(_path);}
	/**
	* get the file full name along with the correct path
	* @param _name string which is the name of the file
	*/
	string getFileName ( const string & _name);
	/**
	* get the file full correct path
	* @param _name string which is the name of the file
	*/
	string getFilePath ( const string & _name);

protected:	
	// The unique instance of this class
	static PathManager *_singleton;
};


// --------------------------------------------------------------------------
// Non member inline functions, for easier use.
// --------------------------------------------------------------------------

/**
* add a path to the file manager
* @param _path string which is the path to add
*/
inline void addPath( const string & _path)
{
	return PathManager::getInstance()->addPath(_path);
}
	
/**
* get the file full name along with the correct path
* @param _name string which is the name of the file
*/
inline string getFileName ( const string & _name)
{
	return PathManager::getInstance()->getFileName(_name);
}	

/**
* get the file full correct path
* @param _name string which is the name of the file
*/
inline string getFilePath ( const string & _name)
{
	return PathManager::getInstance()->getFilePath(_name);
}	


#endif // __PATH_H__
