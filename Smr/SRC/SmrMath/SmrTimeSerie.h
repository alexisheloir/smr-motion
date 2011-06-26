/**
 * \ingroup SmrMath
 * \file SMRTimeSerie.h
 * 
 */


/** 
 * This class provides a support for generic time series
 *
 * author: Nicolas Courty
 * Date : 23/03/2006
 */

#ifndef __SMRTIMESERIE__H__
#define __SMRTIMESERIE__H__

#include "SmrSTL.h"
#include "SmrQuaternion.h"

 /*! \class SMRTimeSerie
  *  \brief This class provides a support for generic time series
  */

template <class T = double>

class SMRTimeSerie
{
private:
  vector<T> m_serie;
public:
  /// generic constructor
  SMRTimeSerie(void);
  /// copy constructor
  SMRTimeSerie(const SMRTimeSerie<T> & _serie);
  /// constructor from file, only read every nth value of the file
  SMRTimeSerie(const string & _filename, unsigned int n = 1);
  /// destructor
  ~SMRTimeSerie(void){};

public:
  /// element accessor
  inline T operator[] (unsigned int i) const {return m_serie.at(i);};
  /// returns serie Length
  inline unsigned int getLength () const {return static_cast<unsigned int>(m_serie.size());};
  /// get access to data
  vector<T> & getSerie(){return m_serie;}
  /// add element
  inline void add(T _value){m_serie.push_back(_value);}
  /// empty serie
  inline void empty(){m_serie.erase(m_serie.begin(),m_serie.end());}

  /// saves the time serie to a file _filename
  void dumpSerieToFile(const string _filename);
  /// add _decal to every value of the serie
  SMRTimeSerie<T> operator+ (T _decal){
    SMRTimeSerie<T> _tSerie;
    for (unsigned int i = 0; i < getLength() ; i++) _tSerie.m_serie.push_back( m_serie.at(i) + _decal);
    return _tSerie;
  }
  /// get min value of the serie
  inline T getMin() const {return *(min_element(m_serie.begin(),m_serie.end()) );}
  /// get max value of the serie
  inline T getMax() const {return *(max_element(m_serie.begin(),m_serie.end()) );}

  /// output stream operator
  friend ostream & operator << (ostream & os, const SMRTimeSerie<T> serie){
    os << "Number of elements : " << serie.getLength() << endl ;
    os << "Values : ";
    for (typename vector<T>::const_iterator it = serie.m_serie.begin() ; it < serie.m_serie.end() ; it ++)
       os << *it << " ";
     os << endl;
    return os;
  }
};

#include "SmrTimeSerie.inl"

#endif

