//
//Copyright or © or Copr. University of South Brittany, Valoria Laboratory 
//Samsara Group Contributor(s) : Nicolas Courty, Alexis Heloir, Sylvie Gibet,
//Copyright or © or Copr. German Research for Artificial Intelligence, DFKI 
//EMBOTS Group Contributor(s) : Alexis Heloir, Michael Kipp, Stefan John, 
//Pascal Pohl
//
// Corresponding author e-mail: alexis.heloir@dfki.de
//
// This software is a computer program whose purpose is to provide a set 
// of C++ classes and functions that allows to perform skeletal animation 
// on virtual characters.
//
//This software is governed by the CeCILL-C license under French law and
//abiding by the rules of distribution of free software. You can use, 
//modify and/ or redistribute the software under the terms of the CeCILL-C
//license as circulated by CEA, CNRS and INRIA at the following URL
//"http://www.cecill.info". 
//
//As a counterpart to the access to the source code and  rights to copy,
//modify and redistribute granted by the license, users are provided only
//with a limited warranty  and the software's author,  the holder of the
//economic rights,  and the successive licensors  have only  limited
//liability. 
//
//In this respect, the user's attention is drawn to the risks associated
//with loading, using, modifying and/or developing or reproducing the
//software by the user in light of its specific status of free software,
//that may mean  that it is complicated to manipulate,  and  that  also
//therefore means  that it is reserved for developers  and  experienced
//professionals having in-depth computer knowledge. Users are therefore
//encouraged to load and test the software's suitability as regards their
//requirements in conditions enabling the security of their systems and/or 
//data to be ensured and, more generally, to use and operate it in the 
//same conditions as regards security. 
//
//The fact that you are presently reading this means that you have had
//knowledge of the CeCILL-C license and that you accept its terms.


/// \defgroup SmrCommon Smr common routines
/// \file Smr.h
/// \brief All necesary includes are done here

#ifndef __SMR_H__
#define __SMR_H__


// --- CORE
//#include "SmrCore.h"
// --- Renderer
//#include "SmrSimpleRenderer.h"
// --- Utils
//
#include "Path.h"
// --- Math
#include "SmrMath.h"
// --- Random
#include "SmrRandom.h"

// --- IO
#include "SmrIO.h"

// --- Logger
#include "logger.h"


// --- Debug
//#define NDEBUG // The assert macro is disabled if at the moment of including assert.h a macro with the name NDEBUG has already been defined.
#include "assert.h"


using namespace std;

/**
 * \namespace Smr
 * \brief The Smr namespace gathers all the objects of the \ref index Smr Library
 */
namespace Smr {
  static bool Initialized = false;
  inline void initSmr(bool _glewSupport = true)
  {
#ifdef TRACE
    cerr << "*** Initializing Smr Library" << endl;
#endif
#if 0
    if (_glewSupport){
      // init glew
      GLenum err = glewInit();
      if( GLEW_OK != err ) {
        // Problem: glewInit failed, something is seriously wrong.
#ifdef TRACE
        cerr << "Error: " << glewGetErrorString( err ) << endl;
#endif
      }
      
#ifdef TRACE
      cout << "GLEW Version String: " << glewGetString( GLEW_VERSION ) << endl;
      cout << "OpenGL Version String: " << glGetString( GL_VERSION ) << endl;   
#endif
    }
#endif
    // generate new random
    SMRRandom::randomize();

    // add files path
    addPath("./Data");
    addPath("../Data");
    addPath("../../Data");
    addPath("../../../Data");
    addPath("../../../../Data");


    config.configure();
    //logger.setLogLevel(TRACE_LOG_LEVEL);
    //logger.setLogLevel(DEBUG_LOG_LEVEL);
    logger.setLogLevel(INFO_LOG_LEVEL);
    //logger.setLogLevel(WARN_LOG_LEVEL);
    //logger.setLogLevel(ERROR_LOG_LEVEL);
    //logger.setLogLevel(FATAL_LOG_LEVEL);


    Smr::Initialized = true;
  }

  inline void shutdown()  {
    //ShaderManager::kill();
    PathManager::kill();
    Smr::Initialized = false;
  }
}
#endif

/*
 #------------------------------------------------------------------------------------
 #
 #
 # Additional documentation for the generation of the reference page (using doxygen)
 #
 #
 #------------------------------------------------------------------------------------
 */

/**
   \mainpage Index
  
   This is the reference documentation of the SMR skeletal animation library.
   These pages have been generated using <a href="http://www.doxygen.org">doxygen</a>.
   The documentation contains detailed descriptions of all classes and functions provided by the SMR library.

   The SMR animation and motion analysis library is a portable C++ library which has been designed to
   offer a consistent framework dedicated to the analysis and synthesis of human motion. SMR provides 
   common methods for motion analysis ( PCA analysis, Time warping, Interpolation ) and straightforward 
   data structures commonly used in animation frameworks (skeletal structures, kinematic chains), 
   SMR provides easy access to forward and inverse kinematics algorithms as well as a collection 
   of functions dedicated to load common animation assets (bvh and asf/amc motion files). 
   SMR is generic (as generic as possible), although a minimal openGL visualization framework is provided, 
   SMR may be integrated into existing rendering frameworks like Horde3D  or Panda 3D. 

   Content:
   -# The \ref introduction section gives an overview of SMR.
   -# An overview about character animation basics can be found in section \ref charanim.
   -# See the \ref installation section on how to install and compile the SMR library.
   -# The \ref faq "FAQ" section answers frequently asked questions.
   -# The \ref features section lists major features of the library.
   -# For information on the supported motion file formats see the \ref fileformats section.
   -# A series of tutorials can be found in the \ref tutorials section. \image html tutorials_thumbs.png
   -# For useful links concerning character animation go to the \ref links section.
   -# The \ref glossary section describes the most important concepts.

   Reference:
   - <A HREF="annotated.html">Class List</A>
   - <A HREF="hierarchy.html">Class Hierarchy</A>

   Go to the \ref introduction "first section (Introduction)".
**/

/*   If you have downloaded the \ref index "Smr" package, you actually have a local copy of these pages in the <a href="Smr/documentation/reference/">Smr/documentation/reference/</a>\endlink directory.
  
   Use the menu above to navigate through the documentation pages.
   As a first step, you may look at the list of <a href="modules.html">available modules</a> or directly go to the \ref smr_introduction "introduction page" or to the <a href="examples.html">Examples</a> section.
*/


/*
<AH_rem> I would simply remove the following, and introduce it somewhere else
    - Flexible enough for multiple formats
      - Should work in absolute frame/relative coordinate 
      - Should work in Joint-based/bone-based representation
*/


/**
  \page introduction Introduction
  
  \section introduction Introduction
  The SMR library is a <em>skeletal C++ animation library</em>.
  It provides useful classes and functions to load/save, display and process various types of motions, as 
  well as motion control routines such as <em>inverse kinematics</em>. The SMR library provides as well 
  <em>statistic analysis tools</em> specially designed for recorded motions such as <em>PCA decomposition</em>.
  
  \section motivation Motivation
  - Favoring ease of use and consistency before tricky optimizations
  - Multi-purpose:
    - Motion capture
    - Forward kinematics
    - Motion analysis (statistical and analytical)
    - Procedural animation
  
  \section other Related Work
  - Ikan [Badler et al.]
    - Improved analytical inverse kinematics
    - Classical HAL with 7 DOF
    - Cal3d [Gnu]
    - Forward kinematics only
  - MKM [Multon et al.]
    - Forward kinematics and inverse kinematics
    - Analytical + Efficient ccd like IK
    - commercial library
  - Early versions of dance [Shapiro et al.]
    - Dynamics
    - commercial library

  \section history History
  - SMR: <em>Synthèse et Analyse du Mouvement pour la Simulation et l’Animation Réaliste d’Avatars</em>
  (Synthesis and Analysis of the Movement for the Simulation and Animation of Realistic Avatars)
  - First version of SMR Was born in fall 2005
  - Second version of SMR in spring 2006
  - APP registration in spring 2008 (Agence de protection des programmes) 


  <AH_rem> OK, the components, core, scripting capabilities and future work need more refinement. It is crutial to stay the most general possible here, we should just talk about concepts, not about classes and technical features. I will see what I can do about it later.</AH_rem>
  \section components Components
  \image html modules.png "Overview of the components defining the SMR library."
  - Core
    - Skeleton, Joint, DOF
    - Actuators: MotionPlayer, IKSolver
    - Blenders
  - Math
    - Vectors, Quaternions
    - Matrix operations and decomposition
    - Time series algorithms (DTW)
  - IO
    - Loaders: ASF/AMC, BVH, Vicon VSK/.V
  - Renderer
    - Basic integrated renderers (shader and non shader)
  - Common
    - Declarations, path managers

  \section core Core
  - Main class: SMRSkeleton
    - A Skeleton contains a joint collection: SMRJoint
    - A Joint may contains several degrees of freedom
  - Skeleton modifiers: SMRActuator
    - SMRActuator is abstract class
    - Specializations: SMRMotionPlayer, SMRIKSolver
  - SMRBlender: stitch skeleton together

  \section scripting Scripting Capabilities
  The SMR library supports scripting by providing an Python interface to the most important classes. The pyBoost library is used to expose the classes.
  See the <A target="_top" href="http://www.language-binding.net/pyplusplus/examples/boost/boost.html">Language Binding Project</A> page for more information about pyBoost.
  - Classes exposed using pyBoost library:
    - SMRDOF
    - SMRGSMMSolver
    - SMRKinematicChain
    - SMRSkeleton
    - SMRQuaternion
    - SMRVector3

  \section future Future Work
  - Implement Swing/Twist joints
  - Switch to MotionSegment interpolation
  - Combine with panda morph targets
  - Choose and implement client server capabilities
  - Put each actuator in a separated thread
  - Support for COLLADA
  - Test, test and test again !

  \section akn Acknowledgements

  Go to the \ref charanim "next section (Character Animation)" or return to the \ref main "index". 
**/


/**
<AH_rem> This looks too much like a catalogue. We can assume that people who want to use the library know a little bit about character animation. A nicer way to use computer animation techniques would be to use it in order to introduce some aspects of SAMSARA, with links to the corresponding tutorial.</AR_rem>
  \page charanim Character Animation
  
  \section ingredients Ingredients
  - character animation = modeling + animation
  - model consists of
    - mesh (skin)
    - skeleton
    - skinning/rigging
    - morph targets

  \section mesh Mesh
  - created by artist or 3D scan
  - consists of polygons
  - vertex/vertices + face(s)
  - texture (e.g. bitmap file) determines faces‘looks
  - usually triangulated for graphics engine (automatic)
  - number of polygons determine whether manageable in real time (low-poly models vs. high-poly models)
  - structure of mesh highly important for skinning/deformation

  \section skeleton Skeleton
  - created after mesh
  - consists of joints
  - bone between two joints(usually synonymous with root joint)
  - joint hierarchy
  - more joints =>more difficult to animate

  \section skinning Skinning/Rigging
  - skeleton moves =>mesh should follow (deformation)
  - bind each vertex to a joint
  - resulting mesh deformation can look awkward
  - bind each vertex to multiple joints, using weights („influence“)
  - structure of mesh important!

  \section morphtargets Morph Targets
  - easy technique to animate faces
  - no skeleton!
  - given two meshes A and B with equal structures: compute intermediate mesh

  \section animation Animation
  - Animation = change (of a figure‘s pose) over time
  - Pose = specific configuration of skeleton (rotation of each joint + translation of root joint)
  - Animation: time →pose
  - For continuous motion: > 25 fps (usually around 50 fps)

  \section manualanim Manual Animation
  - artist defines only „important“(=key) poses
  - computer does „in-betweening“
  - usually interpolation between joint rotations
  - animator adjusts temporal dynamics
  - alternatives
    - motion capture
    - procedural animation
    - physical simulation

  \section animreal Animation in Real-Time
  - Model + animation produced in 3D tool (3DS, Blender, Maya)
  - Can be played in realtime using game/graphics engine(Unreal, Ogre3D, Crystal Space)
  - Exchange format: COLLADA <AH_rem> COLLADA is currently NOT supported in SMR </AH_rem>

  \section realtime Real Time
  - Inside the graphics engine
    - usually in the main loop called N times per second
    - set morph targets (linear combination)
    - set animation time
    - manipulate joints individually (e.g. gaze)
  - Higher-level controls
    - play/pause animation
    - blend animations
      - mask joints

  \section articulated Articulated Structures
  - Most used method to control human-like structures for animation
  - Kinematics
    - study of motion independent of underlying forces (no physics!): position, velocity, accel.
  - Articulated figure
    - consists of series of rigid links connected at (rotary) joints
  - Degrees of freedom (DOF)
    - number of indep. variables to specify the state of the structure
  - State vector:
    - set of parameters defining the DOF

  \section ik Inverse Kinematics
  \image html chain.png "A kinematic chain is defined over joints and there orientations. The end effector is the last joint in the kinematic chain."
  - convenient for arm animation (picking, pointing, gesture) and walking
  - drawbacks
    - becomes harder with increasing DOF
    - multiple solutions exist
      - solution: introduce constraints
      - however: no precise control
  - animation usually relies on both FK + IK

  \image html multiple_solutions.png "Using Inverse Kinematics can lead to multiple solutions for a given configuration."
   

  \section joints Joints
  - Revolute joint (1 DOF)
  - Prismatic joint (1 DOF)
  - Ball-and-socket joint (3 DOF)

  \section representation Representation
  - use tree of nodes and arcs
  - highest node = root node
  - every node positioned relative to frame-of-reference of parent node
  - every arc represents an affine transformation

  \image html articulatedfigure.png "An example of an articulated figure representing an upper human body."

  \section represrots Representation of Rotations
  - Rotations are the most important operation in skeletal animation
  - Typical problem: interpolate between two joint positions
  - So far: matrix representation
  - Alternatives:
    - Euler angles
    - Quaternion

  \section euler Euler Angles
  - any rotation can be expressed using x/y/z rotations: R = (rx, ry, rz)
  - advantage:
    - easy to understand / visualize
    - minimal: 3 parameters for 3 DOF
  - disadvantages:
    - ambiguous: R(45°,90°,0)=R(0,0,45°)→depending on your choice you get 2 different interpolation paths! (VIDEO)
    - linear interpolation results in nonlinear motion
    - gimbal lock

  \section quaternions Quaternions
  - Origin: extending complex/imaginary numbers (Hamilton 1866)
  - Intuition: represent a rotation by an axis of rotation and a rotation angle
  - Notation:
    - Q = w + x i+ y j+ z k
    - q = (w, v) v= (x, y, z)
  - Multiplication:
    - q1q2= (w1w2–v1v2, v1 x v2+ w1v2+ w2v1)
  - Important property: composition by multiplication
    - q = q1q2
  - Fast transformation, e.g. Euler -> Quat
    - qx= (cos(rx/2), (sin(rx/2), 0, 0))
    - q = qzqyqx
  - Most importantly: algorithm for smooth interpolation without gimbal lock
    - SLERP
  - Advantages
    - interpolation without singularities
    - computationally efficient
  - Disadvantage
    - non-intuitive maths

  Go to the \ref installation "next section (Installation)" or return to the \ref main "index". 
**/


/**
  \page installation Installation

  \section dependencies Dependencies
  The SMR library relies on several external libraries. For ease of distribution/commodity some of the libraries are embedded in the source.
  External libraries lie in the <a href="../thirdParty">/Smr/thirdParty</a> directory. 
  Libraries' source code (either include files, either portable code to be compiled) are found in the \b Universal directory.
  Libraries binaries (already compiled libraries) are found in the directory matching your actual architecture.
  
  The SMR library relies on the following dependencies:
  - <A target="_top" href="http://www.robertnz.net/nm_intro.htm">Newmat</A> for matrix computation
  
  For rendering purpose, the SMR library can use the Horde3D next-gen rendering engine. The following dependencies have to be provided:
  - <A target="_top" href="http://www.horde3d.org/">Horde3D library</A> (Horde3D core library)
  - <A target="_top" href="http://www.horde3d.org/">Horde3D utils library</A> (Horde3D utils library)
  - <A target="_top" href="http://glfw.sourceforge.net/">glfw library</A> (portable openGL framework)

  \section organisation Folder organisation
  \b /smr contains the following folders : 
  - \b data folder containing example data in order to test and run SMR library,
  - \b doc folder containing the documentation you are actually browsing if you downloaded Smr,
  - \b SRC folder containing the actual source code of SMR library,
  - \b thirdParty folder containing depencency libraries, source code and include files,
  - \b Examples folder containing some relevant examples teaching you how to do useful things with SMR library

  \section compilation Compilation
  SMR library make use of the cross platform <a href="www.cmake.org"> cmake </a> build environment. Depending on your platform, you will have to
  - get cmake : download binary <a href="www.cmake.org">here</a> or use your favorite package management tool.
  - run cmake in the Smr base directory, choose the relevant <a href="http://www.cmake.org/HTML/cmake-2.6.html#section_Generators"> project generator </a>
  - built binaries are created at the top level of the Smr directory structure in a folder matching matches your acrhitecture. At the moment, Smr has been tested on win32 architectures, apple PPC and Intel architectures and unix x86 32 and 64 bit architectures.
  - cmake configuration and platform issues can be tweaked in the <a href="../CMakeLists.txt">/Smr/CMakeLists.txt</a> configuration file of the CMake build system.
  - Happy compiling !

  \image html cmake.png "The CMake GUI tool."

  Go to the \ref faq "next section (FAQ)" or return to the \ref main "index". 
**/


/**
  \page faq Frequently Asked Questions
  -# <b>What does SMR stand for?</b>\n\n
    \n\n
  -# <b>Are there other animation libraries?</b>\n\n
    \n\n
  -# <b>Is there a .NET C# version of the SMR library available?</b>\n\n
    Currently not.
    \n\n

  Go to the \ref features "next section (Features)" or return to the \ref main "index". 
**/


/**
  \page features Features
  - skeletal C++ animation library
  - portable
  - consistent framework dedicated to the analysis and synthesis of human motion
  - methods for motion analysis (statistical and analytical):
    - PCA analysis
    - Time warping
    - Interpolation
  - straightforward data structures:
    - skeletal structures
    - kinematic chains
  - easy access to forward and inverse kinematics algorithms
  - dedicated functions to load common animation assets:
    - BVH
    - ASF/AMC motion files
    - Vicon VSK/.V
  - generic paradigm
  - statistic analysis tools
  - procedural animation
  - Time series algorithms (DTW)

  Go to the \ref fileformats "next section (File Formats)" or return to the \ref main "index". 
**/


/**
  \page fileformats File Formats
  - ASF/AMC (Rotation First). A file format specification can be found <A target="_top" href="http://www.cs.wisc.edu/graphics/Courses/cs-838-1999/Jeff/ASF-AMC.html">here</A>.
    - See an example \subpage asf file
    - See an example \subpage amc file
  - BVH (Translation First) Biovision Hierarchical Data. A file format specification can be found <A target="_top" href="http://www.cs.wisc.edu/graphics/Courses/cs-838-1999/Jeff/BVH.html">here</A>.
    - See an example \subpage bvh file
  - Vicon VSK
    - See an example \subpage vsk file

  Go to the \ref tutorials "next section (Tutorials)" or return to the \ref main "index". 

  \page asf ASF
  Here is an example of an ASF skeleton file:
  \include example.asf

  \page amc AMC
  Here is an example of an AMC motion file:
  \include example.amc

  \page bvh BVH
  Here is an example of a BVH motion capture file:
  \include example.bvh

  \page vsk VSK
  Here is an example of a VSK motion capture file:
  \include example.vsk
**/


/**
  \page tutorials Tutorials
  - \subpage tutorial00 \image html tutorial00_01_thumb.png
  - \subpage tutorial01 \image html tutorial01_01_thumb.png
  - \subpage tutorial02 \image html tutorial02_01_thumb.png
  - \subpage tutorial03 \image html tutorial03_01_thumb.png
  - \subpage tutorial04 \image html tutorial04_01_thumb.png
  - \subpage tutorial05 \image html tutorial05_01_thumb.png

  Go to the \ref links "next section (Links)" or return to the \ref main "index". 
**/


/**
\page tutorial00 Tutorial 00: Basic Application
  \section tut00 Description
  \image html tutorial00_01.png
  Overview:
  This tutorial shows how to setup a basic application with the tutorial classes provided in the SMR library.
  The classes encapsulate Horde3D functionality. The camera can be moved with the keys listed in the following section.
  - \ref tut00_01
  - \ref tut00_02
  - \ref tut00_03

  Key mapping:
  - ESC - Quit application
  - F1 - Toggle windowed/fullscreen
  - F2 - Debug view mode on/off
  - F3 - Wireframe mode on/off
  - F4 - Show FPS on/off
  - W - Move forward
  - S - Move backward
  - A - Strafe left
  - D - Strafe right
  - Left Shift - Speed up camera movement

  \section tut00_01 Step 1: Basic Window
  
  Include necessary headers:
  \include tutorial00_01.cpp
  \n
  The main entry function:
  \include tutorial00_02.cpp
  \n

  \section tut00_02 Step 2: Preparing the Main Class
  The main class:
  \include tutorial00_03.cpp
  \n

  \section tut00_03 Step 3: Implementing the Main Class
  The constructor:
  \include tutorial00_04.cpp
  \n
  The \c onLoadResources function:
  \include tutorial00_05.cpp
  \n
  The \c onAddNodes function:
  \include tutorial00_06.cpp
  \n

  Go to the \ref tutorial01 "next tutorial (Kinematic Chain)" or return to the \ref main "index". 
**/


/**
\page tutorial01 Tutorial 01: Kinematic Chain
  \section tut01 Description
  \image html tutorial01_01.png
  Overview:
  In this tutorial a basic kinematic chain is created with four joints defining three bones.
  Procedural animation techniques are used to set the rotations of the joint procedurally.
  - \ref tut01_01
  - \ref tut01_02
  - \ref tut01_03

  \section tut01_01 Step 1: Creating a Kinematic Chain
  
  Include necessary headers:
  \include tutorial01_01.cpp
  \n
  Prepare variable:
  \include tutorial01_02.cpp
  \n
  Instantiate an empty kinematic chain:
  \include tutorial01_03.cpp
  \n

  \section tut01_02 Step 2: Adding Joints

  Add four joints:
  \include tutorial01_04.cpp
  \n
  Instantiate a new kinematic joint:
  \include tutorial01_05.cpp
  \n
  Give a name to the joint:
  \include tutorial01_06.cpp
  \n
  Set up joint translation parameters in local frame (bone length):
  \include tutorial01_07.cpp
  \n
  Take care of parenting issues:
  \include tutorial01_08.cpp
  \n
  Add the new joint into the kinematics chain and keep a reference of the newly created joint for the next turn (parenting issue):
  \include tutorial01_09.cpp
  \n
  After creating four joints set root joint:
  \include tutorial01_10.cpp
  \n

  \section tut01_03 Step 3: Animating the Kinematic Chain

  Update joint rotations procedurally:
  \include tutorial01_11.cpp
  \n

  Go to the \ref tutorial02 "next tutorial (Inverse Kinematics)" or return to the \ref main "index". 
**/


/**
\page tutorial02 Tutorial 02: Inverse Kinematics
  \section tutorial02 Description
  \image html tutorial02_01.png
  Overview:
  In this tutorial we extend the concept of the kinematic chain and create one with a higher number of joints.
  This time an inverse kinematics solver is used to update the orientations of the joints in order for the kinematic chain to reach with
  its tip a randomly placed goal object.
  - \ref tutorial02_01
  - \ref tutorial02_02
  - \ref tutorial02_03
  - \ref tutorial02_04

  \section tutorial02_01 Step 1: Preparation

  Include necessary headers:
  \include tutorial02_01.cpp
  \n
  Add member variables to class:
  \include tutorial02_02.cpp
  \n

  \section tutorial02_02 Step 2: Creating an Inverse Kinematics Chain
  Setting degrees of freedom of joints:
  \include tutorial02_03.cpp
  \n
  Set up joint gain (the more distal the more gain):
  \include tutorial02_04.cpp
  \n
  Force the root joint not to move (basis should be fix):
  \include tutorial02_05.cpp
  \n
  Add the fly constraint (relative to tip):
  \include tutorial02_06.cpp
  \n

  \section tutorial02_03 Step 3: Creating the IK Solver
  Create solver:
  \include tutorial02_07.cpp
  \n
  Set constraint:
  \include tutorial02_08.cpp
  \n

  \section tutorial02_04 Step 4: Updating the Kinematic Chain
  Update constraint position according to fly position:
  \include tutorial02_09.cpp
  \n
  Compute inverse kinematics:
  \include tutorial02_10.cpp
  \n

  Go to the \ref tutorial03 "next tutorial (Skinning)" or return to the \ref main "index". 
**/

/**
\page tutorial03 Tutorial 03: Skinning
  \section tut00 Description
  \image html tutorial03_01.png

  Overview:
  In this tutorial the kinematic chain is hidden but deforms a mesh in the shape of a tentacle. The procedure is called skinning.
  - \ref tutorial03_01
  - \ref tutorial03_02
  - \ref tutorial03_03

  \section tutorial03_01 Step 1: Preparation

  Add variables
  \include tutorial03_01.cpp
  \n
  Add to \c onLoadResources function:
  \include tutorial03_02.cpp
  \n
  Add to \c onAddNodes function:
  \include tutorial03_03.cpp
  \n

  \section tutorial03_02 Step 2: Inverse Kinematic Chain

  Create a kinematic chain according to the Horde tentacle object:
  \include tutorial03_04.cpp
  \n

  \section tutorial03_03 Step 3: Animating the Tentacle
  Update the tentacle:
  \include tutorial03_05.cpp
  \n

  Go to the \ref tutorial04 "next tutorial (Loading Motion Files)" or return to the \ref main "index". 
**/

/**
\page tutorial04 Tutorial 04: Loading Motion Files
  \section tut00 Description
  \image html tutorial04_01.png

  Overview:
  In this tutorial a skeleton and a corresponding walking motion are loaded from two files. The animation is played in a loop.
  - \ref tutorial04_01
  - \ref tutorial04_02

  \section tutorial04_01 Step 1: Loading File

  Add variable:
  \include tutorial04_01.cpp
  \n
  Load skeleton and motion file (Acclaim file format ASF/AMC):
  \include tutorial04_02.cpp
  \n

  \section tutorial04_02 Step 2: Accessing Motion Data

  Get number of frames in motion file:
  \include tutorial04_03.cpp
  \n
  Get skeleton of frame:
  \include tutorial04_04.cpp
  \n

  Go to the \ref tutorial05 "next tutorial (Analytical IK)" or return to the \ref main "index". 
**/

/**
\page tutorial05 Tutorial 05: Analytical IK
  \section tutorial05 Description
  \image html tutorial05_01.png
  Overview:
  In this tutorial an analytical inverse kinematics solver was used in order to compute the rotation of the 
  human-arm like kinematic chain according to a goal object.
  - \ref tutorial05_01
  - \ref tutorial05_02
  - \ref tutorial05_03
  - \ref tutorial05_04

  \section tutorial05_01 Step 1: Preparation

  Include necessary headers:
  \include tutorial05_01.cpp
  \n
  Add member variables to class:
  \include tutorial05_02.cpp
  \n

  \section tutorial05_02 Step 2: Creating an Inverse Kinematics Chain
  Add the fly constraint (relative to tip):
  \include tutorial05_03.cpp
  \n

  \section tutorial05_03 Step 3: Creating the IK Solver
  Create solver:
  \include tutorial05_04.cpp
  \n
  Set constraint:
  \include tutorial05_05.cpp
  \n

  \section tutorial05_04 Step 4: Updating the Kinematic Chain
  Update constraint position according to fly position:
  \include tutorial05_06.cpp
  \n
  Compute inverse kinematics:
  \include tutorial05_07.cpp
  \n

  Return to the \ref main "index".
**/

/**
  \page links Links
  Development
  - Boost C++ Libraries
    [ <A target="_top" href="http://www.boost.org/">Homepage</A>
    | <A target="_top" href="http://www.boostpro.com/products/free">Installers for Windows</A> ]
  - CMake - Cross Platform Make
    [ <A target="_top" href="http://www.cmake.org/">Homepage</A>
    | <A target="_top" href="http://www.cmake.org/cmake/resources/software.html">Downloads</A> ]
  - Horde3D
    [ <A target="_top" href="http://www.horde3d.org/">Homepage</A>
    | <A target="_top" href="http://www.horde3d.org/download.html">Download</A>
    | <A target="_top" href="http://www.horde3d.org/docs/manual.html">Documentation</A>
    | <A target="_top" href="http://www.horde3d.org/forums/">Forums</A> ]
  - Panda3D
    [ <A target="_top" href="http://panda3d.org/">Homepage</A>
    | <A target="_top" href="http://panda3d.org/download.php">Download</A>
    | <A target="_top" href="http://panda3d.org/wiki/index.php/Main_Page">Manual</A>
    | <A target="_top" href="http://panda3d.org/apiref.php?page=classes">Reference</A>
    | <A target="_top" href="http://panda3d.org/phpbb2/index.php">Forums</A> ]
  - Python
    [ <A target="_top" href="http://www.python.org/">Homepage</A>
    | <A target="_top" href="http://www.python.org/download/">Download</A>
    | <A target="_top" href="http://www.python.org/doc/">Documentation</A> ]
  - GLFW
    [ <A target="_top" href="http://glfw.sourceforge.net/">Homepage</A>
    | <A target="_top" href="http://glfw.sourceforge.net/download.html">Download</A> ]
  - Doxygen
    [ <A target="_top" href="http://www.stack.nl/~dimitri/doxygen/">Homepage</A>
    | <A target="_top" href="http://www.stack.nl/~dimitri/doxygen/download.html">Download</A>
    | <A target="_top" href="http://www.stack.nl/~dimitri/doxygen/manual.html">Manual</A> ]
  - Microsoft Visual C++ 2008 Express Edition
    [ <A target="_top" href="http://www.microsoft.com/Express/">Homepage</A>
    | <A target="_top" href="http://www.microsoft.com/express/download/">Download</A> ]
  - TortoiseSVN
    [ <A target="_top" href="http://tortoisesvn.tigris.org/">Homepage</A>
    | <A target="_top" href="http://tortoisesvn.net/downloads">Download</A> ]
    

  Miscellaneous
  - <A target="_top" href="http://embots.dfki.de/">EMBOTS Embodied Agents Research Group</A>
  - <A target="_top" href="http://www.blender.org/">Blender</A>
  - <A target="_top" href="http://www.feelingsoftware.com/">Feeling Software</A>
  - Motion Capture Data:
    - <A target="_top" href="http://mocap.cs.cmu.edu/">CMU Motion Capture Database</A>
    - <A target="_top" href="http://www.animazoo.com/Free_downloads.aspx">Animazoo</A>
    - <A target="_top" href="http://accad.osu.edu/research/mocap/mocap_data.htm">ACCAD Lab Mocap DB</A>
    - <A target="_top" href="http://www.mocapdata.com/">mocapdata.com</A>  
  - Cal3D Character Animation Library
    [ <A target="_top" href="http://home.gna.org/cal3d/">Homepage</A>
    | <A target="_top" href="http://download.gna.org/cal3d/documentation/guide/">Cal3D User's Guide</A>
    | <A target="_top" href="http://download.gna.org/cal3d/documentation/api/html/">Cal3D API Reference</A> ]
  - <A target="_top" href="http://www.wotsit.org/">Wotsit.org</A>
  - <A target="_top" href="http://developer.nvidia.com/object/fx_composer_home.html">NVIDIA FX Composer</A>
  - <A target="_top" href="http://www.collada.org/">COLLADA</A>
  - <A target="_top" href="http://www.gamedev.net/reference/articles/article1691.asp">The Matrix and Quaternions FAQ</A>
  
    

  Go to the \ref glossary "next section (Glossary)" or return to the \ref main "index".
**/


/**
  \page glossary Glossary

  \ref A
  \ref B
  \ref C
  \ref D
  \ref E
  \ref F
  \ref G
  \ref H
  \ref I
  \ref J
  \ref K
  \ref M
  \ref P
  \ref Q
  \ref R
  \ref S
  \ref T
  \ref V

  \section A
  \par Absolute Mode
  \par Actuator
  \par Agent
  \par AMC
  Acclaim motion capture data file.
  \par ASF
  Acclaim skeleton file.
  \par Avatar

  \section B
  \par Ball and Socket Joint
  3 DOFs
  \par Bone
  \par BVH
  Biovision Hierarchical Data

  \section C
  \par CCD
  Cyclic Coordinate Descent
  \par Character Animation

  \section D
  \par DOF
  Degree of freedom
  \par DTW
  Dynamic time warping
  \par Dynamics

  \section E
  \par End Effector
  Position and orientation of the last joint in a chain of joints.
  \par Euler Angles
  Describe the orientation of a rigid body in 3-dimensional Euclidean space.

  \section F
  \par Forward Kinematics

  \section G
  \par Generic Programming

  \section H
  \par HAL
  Human-arm like

  \section I
  \par Inbetweening
  Process of generating intermediate frames between two keyframes to give the appearance that the first keyframe evolves smoothly into the second keyframe.
  \par Interpolation
  \par Inverse Kinematics

  \section J
  \par Jacobian
  \par Joint

  \section K
  \par Keyframe
  \par Kinematic Chain
  \par Kinematics

  \section M
  \par Morph Target
  \par Motion Analysis
  \par Motion Blending
  \par Motion Capture
  Process of recording movement and translating that movement onto a digital model.
  \par Motion File
  \par Motion Synthesis

  \section P
  \par PCA
  Principal component analysis
  \par Pose
  \par Procedural Animation
  Type of computer animation, used to automatically generate animation in real-time to 
  allow for a more diverse series of actions than could otherwise be created using predefined animations.

  \section Q
  \par Quaternion

  \section R
  \par Relative Mode
  \par Rotation First
  \par Rotation Order

  \section S
  \par Skeletal Animation
  Technique in computer animation in which a character is represented in two parts: a surface representation used to draw the character (called the skin)
  and a hierarchical set of bones used for animation only (called the skeleton).
  \par Skeleton
  \par Skinning

  \section T
  \par Template
  \par Time Series
  \par Time Warping
  \par Translation First
  See Transformation Order.
  \par Tweening

  \section V
  \par VSK
  Vicon skeleton file format.

  Return to the \ref main "index".
**/


/** \addtogroup smr_introduction Introduction to the Smr Library */
/*@{*/
/**
  \page intro introduction

  The SMR library is a skeletal animation library, designed for C++ programmers.
  It provides useful classes and functions to load/save, display and process various types of motions, as well  as motion control routines such as Inverse Kinematics. SMR library provides aswell statistic analysis tools specially designed for recorded motions like PCA decomposition.

  \section s2 How to compile ?
  SMR library make use of the cross platform <a href="www.cmake.org"> cmake </a> build environment. Depending on your platform, you will have to
  - get cmake : download binary <a href="www.cmake.org">here</a> or use your favorite package management tool.
  - run cmake in the Smr base directory, choose the relevant <a href="http://www.cmake.org/HTML/cmake-2.6.html#section_Generators"> project generator </a>
  - built binaries are created at the top level of the Smr directory structure in a folder matching matches your acrhitecture. At the moment, Smr has been tested on win32 architectures, apple PPC and Intel architectures and unix x86 32 and 64 bit architectures.
  - cmake configuration and platform issues can be tweaked in the <a href="../CMakeLists.txt">/Smr/CMakeLists.txt</a> configuration file of the CMake build system.
  - Happy compiling !

  \section dependencies Dependencies
  SMR library rely on several external libraries. for ease of distribution/commodity, come of these libraries are embedded in the source. external libraries lay in the <a href="../thirdParty">/Smr/thirdParty</a> directory. libraries source code (either include files, either portable code to be compiled) are found in the \b Universal directory. libraries binaries (already compiled libraries) are found in the directory matching your actual architecture.
  SMR library relies on the following dependencies :
  - Newmat for matrix computation
  For rendering purpose, SMR library can use Horde3D next gen rendering engine, the following dependencies have to be provided :
  - Horde3D library (Horde3D core library)
  - Horde3D utils library (Horde3D utils library)
  - glfw library (portable openGL framework)

  \section organisation Folder organisation
  \b /smr contains the following folders : 
  - \b data folder containing example data in order to test and run SMR library,
  - \b doc folder containing the documentation you are actually browsing if you downloaded Smr,
  - \b SRC folder containing the actual source code of SMR library,
  - \b thirdParty folder containing depencency libraries, source code and include files,
  - \b Tutorials folder containing some relevant examples teatching you how to do useful things with SMR library

  \section s3 What's next ?

  If you are ready to get more, and to start writing your own programs using Smr, you are invited to go to the tutorial section.

**/
/*@}*/
