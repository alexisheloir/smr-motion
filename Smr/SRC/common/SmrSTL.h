/**
 * \ingroup SmrCommon
 * \file SmrSTL.h
 *	useful includes for using STL classes
 */

#ifndef __SMRSTL_H__
#define __SMRSTL_H__

#define WIN32_LEAN_AND_MEAN //see http://www.gamedev.net/community/forums/topic.asp?topic_id=127476
                            //otherwise, triggers a redundant "winsock.h" include (in this particular
                            //project)

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <deque>
#include <list>
#include <map>
#include <algorithm>
#include <cassert>

using namespace std;

// Function Object deletion for containers (Functor)... We should use a smart
// pointer instead to be exception-safe.
struct DeleteObjectPtr {
  template<typename T>
  void operator()( const T *ptr ) const {
    if (ptr != NULL)
    {
      delete ptr;
      ptr=NULL;
    }
  }
};

// Function Object deletion for containers (Functor)... We should use a smart
// pointer instead to be exception-safe.
struct DeleteObjectSmartPtr {
  template<typename T>
  void operator()(T object ){
	object.flush();
  }
};

// Function Object deletion for Map containers (Functor)... We should use a smart
// pointer instead to be exception-safe.
struct DeleteObjectPtrInMap {
  template<typename T>
  void operator()( const T *ptr ) const {
	delete ptr->second;
  }
};

// Function Object deletion for containers (Functor)... We should use a smart
// pointer instead to be exception-safe.
struct DeleteObjectSmartPtrInMap {
  template<typename T>
  void operator()( const T object ) const {
	delete object.second;
  }
};


// template<typename T>
// static void DeleteObjectPtr(T *ptr) {
//  delete ptr;
// }
#endif

