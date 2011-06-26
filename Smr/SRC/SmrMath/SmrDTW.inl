template <class T> 
void buildDistanceMatrix(const SMRTimeSerie<T> & _serie1,
						 const SMRTimeSerie<T> & _serie2,
						 Matrix & _matrix)
{
	for (int i = 1; i <= _matrix.nrows(); i++){
		for (int j = 1; j <= _matrix.ncols(); j++) {
			_matrix(i,j) = fabs(_serie1[i-1] - _serie2[j-1]);

		}
	}
}

// return the min value and its position
template <class T> 
T minChoice(const T & a, const T & b, const T & c, int & pos)
{
	T minValue = min ( min ( a, b), c );
	if (minValue==a) {pos=1; return a;}
	if (minValue==b) {pos=2; return b;}
	if (minValue==c) {pos=3; return c;}
	// Critical case, shouldn't happen
	pos=0;
	return 0;
}

template <class T> 
double computeDTW(const SMRTimeSerie<T> & _serie1,
			      const SMRTimeSerie<T> & _serie2,
				  vector<unsigned int> & _alignX,
				  vector<unsigned int> & _alignY
				  )
{
	// Create a matrix wich is of size dim(serie1) X dim(serie2)
	Matrix dMatrix(_serie1.getLength(), _serie2.getLength());
	buildDistanceMatrix(_serie1, _serie2, dMatrix);
	return performDTW(dMatrix, _alignX,_alignY);
}

template <class T> 
double getDTWDistance(const SMRTimeSerie<T> & _serie1,
			          const SMRTimeSerie<T> & _serie2
					  )
{
	// Create a matrix wich is of size dim(serie1) X dim(serie2)
	Matrix dMatrix(_serie1.getLength(), _serie2.getLength());
	buildDistanceMatrix(_serie1, _serie2, dMatrix);
	vector<unsigned int> dummyX, dummyY;
	return performDTW(dMatrix,dummyX,dummyY);
}
