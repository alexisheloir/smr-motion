/*
 *  SmrExporter.h
 *  SmrCore
 *
 *  Created by Alexis Heloir/ Nicolas Courty on 18/10/05.
 *  Copyright 2005 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef _SMREXPORTER__H_
#define _SMREXPORTER__H_

#include "SmrSTL.h"

#include "SmrSkeleton.h"
#include "SmrMotion.h"

/**
* Those C style functions are provided to export different 
* mocap file formats
*/
 
/**
 * Write a motion into a bvh file, change bind pose according to _bindPose 
 */
void exportMotionToBVH(string _filename, SMRMotion &_motion, SMRSkeleton &_bindPose);

/**
 * Write a motion into a bvh file
 */
void exportMotionToBVH(string _filename, SMRMotion &_motion);

/**
 * Write a pose into a bvh file (as a single framed motion)
 */
void exportPoseToBVH(string _filename, SMRSkeleton &_pose);

#endif // _SMREXPORTER__H_
