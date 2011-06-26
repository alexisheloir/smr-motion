#include "SmrDTW.h"

void buildDistanceMatrix(const SMRTimeSerie<SMRQuaternion> & _serie1,
             const SMRTimeSerie<SMRQuaternion> & _serie2,
             Matrix & _matrix)
{
  _matrix.resize(_serie1.getLength(),_serie2.getLength());
  for (int i = 1; i <= _matrix.nrows(); i++)
    for (int j = 1; j <= _matrix.ncols(); j++) 
      // geodesic distance
      _matrix(i,j) = GeodesicDistance(_serie1[i-1],_serie2[j-1]);

}

void buildDistanceMatrix(const SMRTimeSerie<SMRSkeleton> & _serie1,
             const SMRTimeSerie<SMRSkeleton> & _serie2,
             Matrix & _matrix)
{
  _matrix.resize(_serie1.getLength(),_serie2.getLength());
  for (int i = 1; i <= _matrix.nrows(); i++)
    for (int j = 1; j <= _matrix.ncols(); j++) {
      // Points cloud distance
      SMRSkeleton skel1 = _serie1[i-1];
      SMRSkeleton skel2 = _serie2[i-1];
      _matrix(i,j) = PointsCloudDistance(skel1,skel2);
      }
      

}


void buildDistanceMatrix(SMRMotion & _motion1,
             SMRMotion & _motion2,
             Matrix & _matrix)
{
  _matrix.resize(_motion1.getNumFrames(),_motion2.getNumFrames());
  for (int i = 1; i <= _matrix.nrows(); i++)
    for (int j = 1; j <= _matrix.ncols(); j++) 
      // Points cloud distance
      _matrix(i,j) = PointsCloudDistance(_motion1.getSkeleton(i-1),_motion2.getSkeleton(j-1));
}


SMRTimeSerie<double> 
computeAlignFunction(vector<unsigned int> & _alignX,
           vector<unsigned int> & _alignY)
{
  // check regularity for alignement vectors
  assert(_alignX.size()==_alignY.size());
  // create result
  SMRTimeSerie<double> result;
  // create a TimeSerie with distance to diagonal
  // i.e. removing linear trend
  for (unsigned int i = 0 ; i < _alignX.size() ; i++)
    result.getSerie().push_back( double(_alignX.at(i)) - double(_alignY.at(i)) );
  return result;
}

double performDTW(Matrix & _finalmatrix,
          vector<unsigned int> & _alignX,
          vector<unsigned int> & _alignY
          )
{
  Matrix distanceMatrix = _finalmatrix;
  Matrix _pathmatrix(_finalmatrix.nrows(),_finalmatrix.ncols());
  { // compute cumulative distance matrix
    int pos;
    for (int i = 1; i <= _finalmatrix.nrows(); i++){
      for (int j = 1; j <= _finalmatrix.ncols(); j++) {
        double path;
        if (i==1&&j==1) {
          path  = _finalmatrix(i,j);
          pos = 0;
        }
        else if (i==1) {
          path  = _finalmatrix(i,j-1);
          pos = 2 ;
        }
        else if (j==1) {
          path  = _finalmatrix(i-1,j);
          pos = 1;
        }
        else path = minChoice(_finalmatrix(i-1,j), _finalmatrix(i,j-1) , _finalmatrix(i-1,j-1) , pos) ;

        _pathmatrix(i,j) = pos; 
        _finalmatrix(i,j) = _finalmatrix(i,j) + path;

      }
    }
  }

  unsigned int i = _pathmatrix.nrows();
  unsigned int j = _pathmatrix.ncols();

  _alignX.push_back(i);
  _alignY.push_back(j);

  double distance = 0;

  while (i!=1 && j!=1)
  {
    if (_pathmatrix(i,j) == 1) {i = i-1;}
    else if (_pathmatrix(i,j) == 2) {j = j-1;}
    else if (_pathmatrix(i,j) == 3) {i = i-1; j = j-1;}
    
    _alignX.push_back(i);
    _alignY.push_back(j);

    distance += distanceMatrix(i,j);
  }

  return distance;
}
