
#include "Smr.h"
#include "argh.h"

SMRSkeleton refPose;
SMRSkeleton mySkeleton;
SMRMotion myMotion;

/**
* \brief Exit properly 
*/
static void ShutDown(){
  cout << "Shutting down the light..." ;
  // shutdown Smr library
  Smr::shutdown();
  cout << "Done !" << endl;
}

int main(int argc, char *argv[])
{

  /*********************** Parameters handling issues ***********************************/

  string buildfn;
  bool worst = false;

  string inputSkeletonFile;
  string inputMotionFile;
  string outputFile;
  string outputFormat;
  string rot;

  double newFrameRate = 0.0;
  double length = 0.0;
  double start = 0.0;

  RootRotationOrderType bvhRootRotationOrder = ROOTTRANSLATIONFIRST;

  ParamHandler argh;
  argh.AddLong("Tpose", 'T').SetString().SetDesc("input TPose file" , "<file>");
  argh.AddLong("input", 'M').SetString().SetDesc("input motion file (for asf and vsk formats)" , "<file>");
  argh.AddLong("rotate along X (-90)", 'R').SetBool().SetDesc("rotate the motion by -90 degrees along x axis");
  argh.AddLong("output", 'o').SetString().SetDesc("output file" , "<file>");
  argh.AddLong("format", 'F').SetString().SetDesc("output Format (bvh or acclaim), mandatory", "<format>");
  argh.AddLong("version", 'V').SetBool().SetDesc("Displays version information.");
  argh.AddLong("help", 'h').SetBool().SetDesc("Displays quick manual.");
  argh.AddLong("fps", 'f').SetString().SetDesc("set framerate", "<frame/s>");
  argh.AddLong("duration", 'd').SetString().SetDesc("set length of output clip", "<length in s>");
  argh.AddLong("seek", 's').SetString().SetDesc("seek motion clip at second", "time in s");
  argh.AddLong("BVHRootRotationOrder", 'r').SetBool().SetDesc("set BVH root rotationtion order like this: Rx Ry Rz Tx Ty Tz \
                                                               (default is Tx Ty Tz Rx Ry Rz)");

  argh.StartParse(argc, argv);
  for(;;)
  {
    int c = argh.GetParam();
    if(c == -1)break;
    switch(c)
    {
    case 'T': inputSkeletonFile = argh.GetString(); break;//worst = argh.GetBool(); break;
    case 'R': rot = "ROTX"; break;
    case 'M': inputMotionFile = argh.GetString(); break;
    case 'o': outputFile = argh.GetString(); break;
    case 'V': printf("%s\n", "version"); return 0;
    case 'f': newFrameRate = atof(argh.GetString().c_str()); break; // -k
    case 'F': outputFormat = argh.GetString().c_str(); break; // -k
    case 'd': length = atof(argh.GetString().c_str()); break; // --idle
    case 's': start = atof(argh.GetString().c_str()); break; 
    case 'r': bvhRootRotationOrder = ROOTROTATIONFIRST; break;
    case 'h':
      printf(
        "*** SMR motion conversion application ***\n"
        "\n Usage: motionConverter [<option> [<...>]] \n"
        " This program illustrates simple operations\n"
        " you can perform over motion clips\n"
        " thanks to SMR library\n\n");
      argh.ListOptions();
      printf("\nNo warranty whatsoever.\n");
      return 0;
    default:
      printf(
        "*** SMR motion conversion application ***\n"
        "\n Usage: motionConverter [<option> [<...>]] \n"
        " This program illustrates simple operations\n"
        " you can perform over motion clips\n"
        " thanks to SMR library\n\n");
      argh.ListOptions();
      printf("\nNo warranty whatsoever.\n");
      return 0;
    }
  }
  if(!argh.ok())return -1;
  if(inputMotionFile.length() == 0)
  {
    printf(
      "*** SMR motion conversion application ***\n"
      "\n Usage: motionConverter [<option> [<...>]] \n"
      " This program illustrates simple operations\n"
      " you can perform over motion clips\n"
      " thanks to SMR library\n\n");
    argh.ListOptions();
    printf("\nNo warranty whatsoever.\n");
    return 0;
  }

  /*****************************The actual programm***********************************/

  //initialize Smr environment
  Smr::initSmr();

  //instanciate a motion
  SMRMotion inputMotion;
  SMRSkeleton refSkeleton;

  ifstream infile(inputMotionFile.c_str());
  if (infile.fail())
  {
    cout << "motion file " + inputMotionFile + " not found" << endl;
    exit(1);
  }
  infile.close();
  if (inputSkeletonFile != "")
  {
    infile.open(inputSkeletonFile.c_str());
    if (infile.fail())
    {
      cout << "skeleton file " + inputSkeletonFile + " not found" << endl;
      exit(1);
    }
  }
  infile.close();
  if (inputMotionFile.find(".amc",0) !=  string::npos)
  {
    if (inputSkeletonFile.find(".asf",0) == string::npos )
    {
      cout << "You shall specify a asf skeleton file with an amc file" << endl;
      return 0;
    }else
    {
      inputMotion = loadMotionFromAcclaim(inputSkeletonFile, inputMotionFile);
      refSkeleton = inputMotion.getSkeleton(0);
      SMRQuaternion identity;
      identity.identity();
      for (unsigned int i = 0; i < refSkeleton.getNumJoints(); i++)
      {
        refSkeleton.getJoint(i)->setOrientation(identity);
      }
    }
  }else if (inputMotionFile.find(".V",0) !=  string::npos || inputMotionFile.find(".v",0) !=  string::npos )
  {
    if (inputSkeletonFile.find(".vsk",0) == string::npos )
    {
      cout << "You shall specify a vsk skeleton file with" << endl;
      return 0;
    }else
    {
      inputMotion = loadMotionFromVSK(inputSkeletonFile, inputMotionFile);
      refSkeleton = inputMotion.getSkeleton(0);
      SMRQuaternion identity;
      identity.identity();
      for (unsigned int i = 0; i < refSkeleton.getNumJoints(); i++)
      {
        refSkeleton.getJoint(i)->setOrientation(identity);
      }
    }
  }else if (inputMotionFile.find(".bvh",0) !=  string::npos )
  {
    inputMotion = loadMotionFromBVH(inputMotionFile);
    refSkeleton = inputMotion.getSkeleton(0);
    SMRQuaternion identity;
    identity.identity();
    for (unsigned int i = 0; i< refSkeleton.getNumJoints(); i++)
    {
      refSkeleton.getJoint(i)->setOrientation(identity);
    }
  }else
  {
    printf(
      "\n Motion format not recognised, this converter\n"
      "recognises the following formats :\n"
      "\n bvh motion file        : -M filename.bvh \n"
      "acclaim asf/amc motion : -T filename.asf, -M filenaem.amc\n"
      "vicon vsk/V motion     : -T filename.vsk, -M filenaem.V\n"
      "maybe you should chek your input files extentensions \n"
      );
    return 0;
  }

  //get motion actual frameRate
  unsigned int frameRate = static_cast<unsigned int>
    (1 / static_cast<double>(inputMotion.getTimeStep()));

  //get start frame index and end frame index
  unsigned int begin = static_cast<unsigned int>(start * frameRate);
  unsigned int duration = 0;
  if (length == 0)
  {
    duration = inputMotion.getNumFrames();
  } else
  {
    duration = static_cast<unsigned int>(length * frameRate);
  }

  cout << "start frame: " << begin << " duration: " << duration <<endl;

  // feed output motion according to input parameters.
  //outputMotion = inputMotion;
  inputMotion.cutMotion(begin,begin + duration);

  if (newFrameRate > 0.0)
    inputMotion.resample(1.0/newFrameRate);

  //outputMotion.insertSkeleton(refPose, 0);

  cout << inputMotion.getNumFrames() << endl;

  if (rot != "")
  {
    SMRQuaternion rotation(SMRVector3(1.0,0.0,0.0),0.5*M_PI);
    inputMotion.rotate(rotation);
    //rotation = SMRQuaternion(SMRVector3(0.0,1.0,0.0),0.5*M_PI);
    //inputMotion.rotate(rotation);
    refSkeleton.setRotationOrder(TRANSLATIONFIRST);
    refSkeleton.getJoint(0)->setOrientation(rotation*refSkeleton.getJoint(0)->getOrientation());
    inputMotion.changeBindPose(refSkeleton);
  }

  int point = inputMotionFile.rfind(".");
  inputMotionFile = inputMotionFile.erase(point,inputMotionFile.length()-point);
  if(outputFormat == "bvh")
  {
    if (outputFile.length() == 0) outputFile = \
      inputMotionFile + ".bvh";
    exportMotionToBVH(outputFile,inputMotion,bvhRootRotationOrder);
  }else if (outputFormat == "acclaim")
  {
    if (outputFile.length() == 0) 
      outputFile = inputMotionFile + ".amc";
    string outputSkeletonFile = inputMotionFile + ".asf";

    exportMotionToAcclaim(outputFile,inputMotion);
    exportSkeletonToAcclaim(outputSkeletonFile,refSkeleton);
  }else 
  {
    cout << " you shall specify an output format -F ( either \"bvh\" or \"acclaim\") !" << endl;
    exit(1);
  }
  ShutDown();
  return 0;
}
