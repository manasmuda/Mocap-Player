#include "quaternion.h"

/*
  Transforms the given matrix (assumed orthogonal, specified 
  in row-major order) into one of the two corresponding quaternions.

  This implementation follows David Baraff's SIGGRAPH course notes 
  (with David's permission):
  "An Introduction to Physically Based Modeling:
   Rigid Body Simulation I: Unconstrained Rigid Body Dynamics"
  http://www.cs.cmu.edu/~baraff/pbm/pbm.html
*/

template <typename real>
Quaternion<real> Quaternion<real>::Matrix2Quaternion(real * R)
{
/* 
   Order of matrix elements is row-major:

   (0,0) 0  (0,1) 1  (0,2) 2
   (1,0) 3  (1,1) 4  (1,2) 5
   (2,0) 6  (2,1) 7  (2,2) 8
*/

  Quaternion<real> q;
  real trace, u;
  trace = R[0] + R[4] + R[8];

  if(trace >= 0)
  {
    u = (real)sqrt(trace + 1);
    q.s = (real)0.5 * u;
    u = (real)0.5 / u;
    q.x = (R[7] - R[5]) * u;
    q.y = (R[2] - R[6]) * u;
    q.z = (R[3] - R[1]) * u;
  }
  else
  {
    int i = 0;
    if(R[4] > R[0])
      i = 1;

    if(R[8] > R[3*i+i])
      i = 2;

    switch (i)
    {
      case 0:
        u = (real)sqrt((R[0] - (R[4] + R[8])) + 1);
        q.x = 0.5f * u;
        u = 0.5f / u;
        q.y = (R[3] + R[1]) * u;
        q.z = (R[2] + R[6]) * u;
        q.s = (R[7] - R[5]) * u;
      break;

      case 1:
        u = (real)sqrt((R[4] - (R[8] + R[0])) + 1);
        q.y = 0.5f * u;
        u = 0.5f / u;
        q.z = (R[7] + R[5]) * u;
        q.x = (R[3] + R[1]) * u;
        q.s = (R[2] - R[6]) * u;
      break;

      case 2:
        u = (real)sqrt((R[8] - (R[0] + R[4])) + 1);
        q.z = 0.5f * u;

        u = 0.5f / u;
        q.x = (R[2] + R[6]) * u;
        q.y = (R[7] + R[5]) * u;
        q.s = (R[3] - R[1]) * u;
      break;
    }
  }

  return q;
}

template <typename real>
real Quaternion<real>::DotProduct(Quaternion<real> q1, Quaternion<real> q2) {
    return q1.s * q2.s + q1.x * q2.x + q1.y * q2.y + q1.z * q2.z;
}

template Quaternion<double> Quaternion<double>::Matrix2Quaternion(double * R);
template Quaternion<float> Quaternion<float>::Matrix2Quaternion(float * R);

template double Quaternion<double>::DotProduct(Quaternion<double> q1, Quaternion<double> q2);
template float Quaternion<float>::DotProduct(Quaternion<float> q1, Quaternion<float> q2);

