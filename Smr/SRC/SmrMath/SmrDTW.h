/**
 *  \ingroup SmrMath
 *  \file SmrDTW.h
 *  A set of useful functions for computing Dynamic Time Warping 
 * (a.k.a. DTW)
 */
#ifndef SMRDTW_H
#define SMRDTW_H
#pragma once

#include "SmrTimeSerie.h"
#include "SmrQuaternion.h"
#include "SmrSkeleton.h"
#include "SmrMotion.h"

#include "newmat.h"

/**
 *  \brief \brief .
 *
 *  returns the min value and its position
 */
template <class T> T minChoice(const T & a, const T & b, const T & c, int & pos);


/**
 *  \brief Computes the distance matrix between two series.
 */
template <class T> 
void buildDistanceMatrix(const SMRTimeSerie<T> & serie1,
             const SMRTimeSerie<T> & serie2,
             Matrix & matrix);

/**
 *  \brief \brief .
 *
 *  specialization for quaternions
 */
void buildDistanceMatrix(const SMRTimeSerie<SMRQuaternion> & serie1,
             const SMRTimeSerie<SMRQuaternion> & serie2,
             Matrix & matrix);

/**
 *  \brief \brief .
 *
 *  specialization for skeletons
 */
void buildDistanceMatrix(const SMRTimeSerie<SMRQuaternion> & serie1,
             const SMRTimeSerie<SMRQuaternion> & serie2,
             Matrix & matrix);

/**
 *  \brief \brief .
 *
 *  specialization for motions
 */
void buildDistanceMatrix(SMRMotion & motion1,
             SMRMotion & motion2,
             Matrix & matrix);


/**
 *  \brief \brief .
 *
 *  computes DTW from a distance matrix and outputs the two align series
 */
double performDTW(Matrix & _finalmatrix,
            vector<unsigned int> & alignX,
          vector<unsigned int> & alignY
          );

/**
 *  \brief \brief .
 *
 *  computes DTW from two series and outputs the two align series
 */
template <class T> 
double computeDTW(const SMRTimeSerie<T> & serie1,
            const SMRTimeSerie<T> & serie2,
          vector<unsigned int> & alignX,
          vector<unsigned int> & alignY
          );

/**
 *  \brief \brief .
 *
 *  computes the elastic distance between two series
 *  \return the double-precision value of the distance
 */
template <class T> 
double getDTWDistance(const SMRTimeSerie<T> & serie1,
                const SMRTimeSerie<T> & serie2
            );

/**
 *  \brief \brief .
 *  
 *  compute the alignment function between two series
 *  \return a SMRTimeSerie<double> corresponding to the matching function
 */
SMRTimeSerie<double> computeAlignFunction(vector<unsigned int> & alignX,
                          vector<unsigned int> & alignY);

#include "SmrDTW.inl"

#endif
