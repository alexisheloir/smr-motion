template <class T> 
SMRTimeSerie<T>::SMRTimeSerie(){
}

template <class T> 
SMRTimeSerie<T>::SMRTimeSerie(const SMRTimeSerie<T> & _serie){
	for (unsigned int i = 0; i < _serie.getLength() ; i++) m_serie.push_back( _serie[i]);
}

//// overload for quaternions
//SMRTimeSerie<SmrQuaternion>::SMRTimeSerie(const string & _filename, unsigned int n)
//{
//	// Open file
//	ifstream file( _filename.c_str(), ios::in );
//
//	if( file.fail() ) {
//		cerr << "Couldn't load " << _filename << endl;
//	}
//	else{
//		double w, x, y, z;
//		char token ='o';
//		unsigned int cpt = 0 ;
//		try {
//			while(!file.eof() ) {//!file.eof() 
//				file >> w;
//				file >> x;
//				file >> y;
//				file >> z;
//				if(!(cpt%n)) {
//					m_serie.push_back(SmrQuaternion(w,x,y,z));
//					//cout << "Adding quaternion : " << SmrQuaternion(w,x,y,z) << endl;
//				}
//				file >> token; // eat the '\n' 
//				cpt ++;
//			}
//		}
//		catch( ios_base::failure err ) {
//		}
//		cout << "Number of Quaternions read: "<< cpt  << endl;
//	}
//}
//
//

template <class T> 
SMRTimeSerie<T>::SMRTimeSerie(const string & _filename, unsigned int n)
{
	// Open file
	ifstream file( _filename.c_str(), ios::in );

	if( file.fail() ) {
		cerr << "Couldn't load " << _filename << endl;
	}
	else{
		//string token;
		char token ='o';
		unsigned int cpt = 0 ;
		T value;
		try {
			while( token!='\n'&&!file.eof() ) {//!file.eof() 
				file >> value; // this is the value
				if(!(cpt%n)) m_serie.push_back(value);
				file >> token; // eat the colon 
				cpt ++;
			}
		}
		catch( ios_base::failure & err ) {
			cerr << "Couldn't load " << _filename << endl;
			cerr << " Reason: " << err.what() << endl;
			m_serie.clear();
		}

	}
}

template <class T> 
void 
SMRTimeSerie<T>::dumpSerieToFile(const string _filename)
{
	// Open file for writing
	ofstream file( _filename.c_str(), ios::out );

	if( file.fail() ) {
		cerr << "Couldn't open " << _filename << endl;
	}
	else
	{
		try {
			for (typename vector<T>::const_iterator it = m_serie.begin() ; it < m_serie.end() ; it ++)
				file << *it << " ";
			file << endl;		
			file.close();
		}
		catch( ios_base::failure &err ) {
			cerr << "Couldn't write to " << _filename << endl;
			cerr << " Reason: " << err.what() << endl;
		}
	}
}



