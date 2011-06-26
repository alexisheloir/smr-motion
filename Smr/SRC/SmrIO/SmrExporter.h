/*
 *  SmrExporter.h
 *  SmrCore
 */

#ifndef SMREXPORTER_H
#define SMREXPORTER_H

#include "SmrSTL.h"

#include "SmrSkeleton.h"
#include "SmrMotion.h"

typedef enum{ROOTROTATIONFIRST,ROOTTRANSLATIONFIRST} RootRotationOrderType;

/**
 *  The functions are provided to export different mocap file formats.
 */
 
/**
 * Write a motion into a bvh file, change bind pose according to _bindPose 
 */
void exportMotionToBVH(string _filename, SMRMotion &_motion, SMRSkeleton &_bindPose, RootRotationOrderType _rootRotOrder = ROOTTRANSLATIONFIRST);

/**
 * Write a motion into a bvh file
 */
void exportMotionToBVH(string _filename, SMRMotion &_motion, RootRotationOrderType _rootRotOrder = ROOTTRANSLATIONFIRST);

/**
 * Write a pose into a bvh file (as a single framed motion)
 */
void exportPoseToBVH(string _filename, SMRSkeleton &_pose);

///\brief Writes a SMRSkeleton as an Acclaim Skeleton file (ASF).
void exportSkeletonToAcclaim(string filename,const SMRSkeleton& skeleton);

///\brief Writes a SMRMotion as an Acclaim Motion Capture file (AMC).
void exportMotionToAcclaim(string filename,SMRMotion& motion);

void dumpSkeletonOBJ(string fileName,SMRSkeleton& skeleton);

#endif
