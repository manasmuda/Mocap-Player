/*
  interpolator.h
  
  Create interpolated motion.

  Revision 1 - Alla and Kiran, Jan 18, 2002
  Revision 2 - Jernej Barbic, Yili Zhao, Feb 2012
*/


#ifndef _INTERPOLATOR_H
#define _INTERPOLATOR_H

#include "motion.h"
#include "quaternion.h"

enum InterpolationType
{
  LINEAR = 0, BEZIER = 1
};

enum AngleRepresentation
{
  EULER = 0, QUATERNION = 1
};

class Interpolator
{
public: 
  //constructor, destructor
  Interpolator();
  ~Interpolator();

  //Set interpolation type
  void SetInterpolationType(InterpolationType interpolationType) {m_InterpolationType = interpolationType;};
  //Set angle representation for interpolation
  void SetAngleRepresentation(AngleRepresentation angleRepresentation) {m_AngleRepresentation = angleRepresentation;};

  //Create interpolated motion and store it into pOutputMotion (which will also be allocated)
  void Interpolate(Motion * pInputMotion, Motion ** pOutputMotion, int N);

protected:
  InterpolationType m_InterpolationType; //Interpolation type (Linear, Bezier)
  AngleRepresentation m_AngleRepresentation; //Angle representation (Euler, Quaternion)

  // conversion routines
  // angles are given in degrees; assume XYZ Euler angle order
  vector Rotation2Euler(double R[9]);
  void Euler2Rotation(vector angles, double R[9]);
  Quaternion<double> Euler2Quaternion(vector angles);
  vector Quaternion2Euler(Quaternion<double> q);

  //euler interpolation
  vector Lerp(double t, vector start, vector end);

  // quaternion interpolation
  Quaternion<double> Slerp(double t, Quaternion<double> & qStart, Quaternion<double> & qEnd);
  Quaternion<double> Lerp(double t, Quaternion<double> & qStart, Quaternion<double> & qEnd);
  Quaternion<double> Double(Quaternion<double> p, Quaternion<double> q);

  // interpolation routines
  void LinearInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N);
  void BezierInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N);
  void LinearInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N);
  void BezierInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N);

  // Key Frame Interpolate Routines
  void KeyFrameLinearInterpolationEuler(Motion* pInputMotion, Motion* pOutputMotion);
  void KeyFrameBezierInterpolationEuler(Motion* pInputMotion, Motion* pOutputMotion);
  void KeyFrameLinearInterpolationQuaternion(Motion* pInputMotion, Motion* pOutputMotion);
  void KeyFrameBezierInterpolationQuaternion(Motion* pInputMotion, Motion* pOutputMotion);

  // Bezier spline evaluation
  vector DeCasteljauEuler(double t, vector p0, vector p1, vector p2, vector p3); // evaluate Bezier spline at t, using DeCasteljau construction, vector version
  Quaternion<double> DeCasteljauQuaternion(double t, Quaternion<double> p0, Quaternion<double> p1, Quaternion<double> p2, Quaternion<double> p3); // evaluate Bezier spline at t, using DeCasteljau construction, Quaternion version

};

#endif

