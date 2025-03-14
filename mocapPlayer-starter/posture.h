/*
Revision 1 - Steve Lin, Jan. 14, 2002
Revision 2 - Alla and Kiran, Jan 18, 2002
Revision 3 - Jernej Barbic and Yili Zhao, Feb, 2012
*/
#ifndef _POSTURE_H
#define _POSTURE_H

#include "vector.h"
#include "types.h"

class dvector : public vector {

public:
	dvector() : dvector(0, 0, 0) {}
	dvector(double x, double y, double z) : vector(x, y, z) { defaultVal = false; }
	dvector(double a[3]) : dvector(a[0], a[1], a[2]) {}
	dvector(const vector& v) : dvector(v.p[0], v.p[1], v.p[2]) {}

	bool defaultVal;
};

//Root position and all bone rotation angles (including root) 
struct Posture
{
	
public:

  //Root position (x, y, z)		
  vector root_pos;

  //Euler angles (thetax, thetay, thetaz) of all bones in their local coordinate system.
  //If a particular bone does not have a certain degree of freedom, 
  //the corresponding rotation is set to 0.
  //The order of the bones in the array corresponds to their ids in .ASf file: root, lhipjoint, lfemur, ...
  vector bone_rotation[MAX_BONES_IN_ASF_FILE];
  
  // bones that are translated relative to parents (resulting in gaps) (rarely used)
  vector bone_translation[MAX_BONES_IN_ASF_FILE];

  // bones that change length during the motion (rarely used)
  vector bone_length[MAX_BONES_IN_ASF_FILE];

  bool default_root_pos = false;
  bool default_bone_placements[MAX_BONES_IN_ASF_FILE];

  bool defaultPos = false;
};

#endif

