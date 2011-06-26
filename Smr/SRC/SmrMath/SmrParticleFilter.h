/**
 *  \ingroup SmrMath
 *  \file SMRParticleFilter.h
 *  \brief A set of classes and functions that implements particle filters
 *  for computer Animation. See [Courty/Arnaud 2008] for detailed references
 */
#ifndef SMRPARTICLEFILTER_H
#define SMRPARTICLEFILTER_H
#pragma once

#include "SmrRandom.h"


/** 
 *  \class SMRParticleT
 *  \brief Particle
 *
 *  defines a templated particle which is a combination of an 
 *  information and a weight
 */
template<class T>
class SMRParticleT
{
public:
  /**
   *  \brief Constructor.
   */
  SMRParticleT(const T & _val, double _w)
  {
    m_state=_val;
    m_weight=_w;
  }

private:
  /**
   *  \brief State.
   */
  T m_state;
  
  /**
   *  \brief Weight.
   */
  double m_weight;

public:
  /**
   *  \brief Overloaded operator.
   */
  bool operator<(const SMRParticleT & p)
  {
    return m_weight<p.m_weight;
  }

  /**
   *  \brief Sets the weight.
   */
  inline void setWeight(double v)
  {
    m_weight = v;
  }

  /**
   *  \brief Returns the weight.
   */
  inline double getWeight() const
  {
    return m_weight;
  }

  /**
   *  \brief Returns the state.
   */
  inline T & getState()
  {
    return m_state;
  }

  /**
   *  \brief Sets the state.
   */
  inline void setState(const T & skeleton)
  {
    m_state=skeleton;
  }
};

/** 
 * \class SMRParticleFilter
 * \brief describes an abstract simple particle filter.
 * This class is pure virtual
 */
template <class T>
class SMRParticleFilter
{
public:
  /**
   *  \brief Constructor.
   */
  SMRParticleFilter()
  {

  }
  
  /**
   *  \brief Destructor.
   */
  virtual ~SMRParticleFilter()
  {
  
  }

  // ----------- computation part ----------------------
protected:
  /**
   *  \brief A vector of particles.
   */
  vector< SMRParticleT<T> > m_particles;

  /**
   *  \brief The current state.
   */
  T m_state;

protected: // implementation methode

  /**
   *  \brief Samples.
   *
   *  sample around the current state using Normal Law
   */
  virtual void sample() = 0;

  /**
   *  \brief Evaluates the particles.
   *
   *  evaluate particles : update weights wrt. distance to target
   */
  virtual void evaluateParticles() = 0;

  /**
   *  \brief Normalizes the weights.
   *
   *  normalize weights
   */
  void normalizeWeights();

  /**
   *  \brief Determines if resampling is needed.
   *
   *  Resampling criterium
   */
  bool needReSampling();

  /**
   *  \brief Resamples.
   *
   *  resample
   */
  void reSample();

  /**
   *  \brief Selects the best particle.
   *
   *  select particle with greatest weight (MAP)
   */
  SMRParticleT<T> & selectBest();
  
public:
  /**
   *  \brief Returns the number of particles.
   *
   *  get number of particles
   */
  inline unsigned int getNumParticles() const
  {
    return m_particles.size();
  }

  /**
   *  \brief Sets the state.
   *
   *  set state
   */
  void setState(const T & state)
  {
    m_state=state;
  }

  /**
   *  \brief Returns a particle.
   *
   *  get a reference on a special particle 
   */
  SMRParticleT<T> & getParticle(unsigned int num)
  {
    return m_particles.at(num);
  }

  /**
   *  \brief Returns the current state.
   *
   *  get a reference on the current state
   */
  T & getCurrentState()
  {
    return m_state;
  }

protected:
  /**
   *  \brief Processes the filter.
   *
   *  the whole filtering operation
   */
  void processFilter();

  /**
   *  \brief Computes the filter.
   *
   *  batch filtering
   */
  virtual void computeFilter() = 0;
};

/**
 *  \brief Normalizes the weights.
 *
 *  normalize weights
 */
template <class T>
void SMRParticleFilter<T>::normalizeWeights()
{
  double sum = 0;
  for (unsigned int i = 0; i < getNumParticles() ; i++)
    sum+=m_particles.at(i).getWeight();
  for (unsigned int j = 0; j < getNumParticles() ; j++)
    m_particles.at(j).setWeight(m_particles.at(j).getWeight()/sum);
}

/**
 *  \brief Resampling criterium.
 *
 *  Resampling criterium
 */
template <class T>
bool SMRParticleFilter<T>::needReSampling()
{  
  double sumSquare = 0;
  for (unsigned int i = 0; i < getNumParticles() ; i++)
    sumSquare+=m_particles.at(i).getWeight()*m_particles.at(i).getWeight();
  return ((1.0/sumSquare)<=getNumParticles()/2);
}

/**
 *  \brief Resamples.
 *
 *  resample
 */
template <class T> void
SMRParticleFilter<T>::reSample()
{
  cout << "Resampling **((--))**" << endl;
  // resample particles along their own distribution
  // compute base weight, each particle has the same weight
  double baseWeight=1.0/getNumParticles();
  vector< SMRParticleT<T> > tmpV;
  for (unsigned int i = 0; i < getNumParticles() ; i++)
  {
    // here is the tricky part : sampling of a binomial law
    double r = SMRRandom::getNormalizedRandom();
    if (r==1) r = SMRRandom::getNormalizedRandom();
    double base = 0;
    int count = -1;
    do 
    {  count++;
      base+=m_particles.at(count).getWeight();
    }
    while (r>base);
    tmpV.push_back(SMRParticleT<T>(m_particles.at(count).getState(),baseWeight));
  }
  m_particles=tmpV;
}

/**
 *  \brief Selects the best particle.
 *
 *  select particle with greatest weight
 */
template <class T>
SMRParticleT<T> & 
SMRParticleFilter<T>::selectBest()
{
  unsigned int index=0;
  for (unsigned int i =0; i<getNumParticles() ; i++)
    if (m_particles.at(i).getWeight()>m_particles.at(index).getWeight()) 
      index = i;
  return m_particles.at(index);
}

/**
 *  \brief Processes the filter.
 *
 *  the whole filtering operation
 */
template <class T> void 
SMRParticleFilter<T>::processFilter()
{
  // sample around the current state using Normal Law
  sample();
  // evaluate particles : update weights wrt. distance to target
  evaluateParticles();
  // normalize weights
  normalizeWeights();
  // resample if necessary
  if(needReSampling()) 
    reSample();
}

#endif
