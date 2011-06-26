/**
 *  \ingroup SmrCore
 *  \file SMRSkeletonTemplate.h
 */

#include "logger.h"

#ifndef SMRSMARTPOINTER_H
#define SMRSMARTPOINTER_H

template <class T>
class SMRSmartPtr
{
public:

  SMRSmartPtr() : m_pointee(NULL),m_referenceCounter(NULL)
  {
  }

  explicit SMRSmartPtr(T* _pointee) : m_pointee(_pointee)
  {
    m_referenceCounter = new(int);
    *m_referenceCounter = 1;
  }

  SMRSmartPtr(T* _pointee, int *_referenceCounter) : m_pointee(_pointee),m_referenceCounter(_referenceCounter)
  {
    if (m_referenceCounter) (*m_referenceCounter)++;
  }

  SMRSmartPtr(const SMRSmartPtr &_smartPtr)
  {
    m_pointee = _smartPtr.getPointee();
    m_referenceCounter = _smartPtr.getReferenceCounter();
    if(m_referenceCounter) (*m_referenceCounter)++;
  }

  SMRSmartPtr<T>& operator=(const SMRSmartPtr &_smartPtr)
  {
    if (_smartPtr != m_pointee )
    {
      this->~SMRSmartPtr();
      m_pointee = _smartPtr.getPointee();
      m_referenceCounter = _smartPtr.getReferenceCounter();
      if(m_referenceCounter) (*m_referenceCounter)++;
    }
    return (*this);
  }

  int* getReferenceCounter() const
  {
    return m_referenceCounter;
  }

  int* getRifirenceCounter()
  {
    return m_referenceCounter;
  }

  ~SMRSmartPtr();

  void flush();
  
  T& operator*() const
  {
    return *m_pointee;
  }

  bool operator!() const
  {
    if (m_pointee == NULL) return true;
    else return false;
  }

  T* operator->() const
  {
    return m_pointee;
  }

  T* getPointee() const
  {
    return m_pointee;
  }

private: 
  T* m_pointee;
  int* m_referenceCounter; 
};

template <typename T>
SMRSmartPtr<T>::~SMRSmartPtr()
{
  if (m_pointee && m_referenceCounter)
  {
    (*m_referenceCounter)--;
    if (*m_referenceCounter == 0)
    {
      LOG_TRACE(logger,"deleting this joint " << m_pointee->getName());
      delete(m_pointee);
      m_pointee = NULL;
      delete(m_referenceCounter);
      m_referenceCounter = NULL;
    }
  }
}

template <typename T>
void SMRSmartPtr<T>::flush()
{
  if (m_pointee && m_referenceCounter)
  {
      LOG_TRACE(logger,"deleting this joint " << m_pointee->getName() << "referenced " << *m_referenceCounter << " times");
      delete(m_pointee);
      m_pointee = NULL;
      delete(m_referenceCounter);
      m_referenceCounter = NULL;
  }
}

#endif

