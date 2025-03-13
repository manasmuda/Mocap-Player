#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include "motion.h"
#include "interpolator.h"
#include "types.h"

#define DEG2RAD(angle) ((angle) * M_PI / 180.0)

#define MATRIX_MULTIPLY(m1,m2, rm)\
    for(int mmrow = 0; mmrow < 3; mmrow++){\
        for (int mmcolumn = 0; mmcolumn < 3; mmcolumn++) {\
            rm[3*mmrow+mmcolumn] = 0;\
            for (int mmk = 0; mmk < 3; mmk++) {\
                float mmsum = m1[3*mmrow+mmk] * m2[3*mmk+mmcolumn];\
                rm[3*mmrow+mmcolumn] += mmsum;\
            }\
        }\
    }\

Interpolator::Interpolator()
{

  //Set default interpolation type
  m_InterpolationType = LINEAR;

  //set default angle representation to use for interpolation
  m_AngleRepresentation = EULER;
}

Interpolator::~Interpolator()
{
}

//Create interpolated motion
void Interpolator::Interpolate(Motion * pInputMotion, Motion ** pOutputMotion, int N) 
{
  //Allocate new motion
  *pOutputMotion = new Motion(pInputMotion->GetNumFrames(), pInputMotion->GetSkeleton()); 

  //Perform the interpolation
  if (N >= 0) {
      if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == EULER))
          LinearInterpolationEuler(pInputMotion, *pOutputMotion, N);
      else if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == QUATERNION))
          LinearInterpolationQuaternion(pInputMotion, *pOutputMotion, N);
      else if ((m_InterpolationType == BEZIER) && (m_AngleRepresentation == EULER))
          BezierInterpolationEuler(pInputMotion, *pOutputMotion, N);
      else if ((m_InterpolationType == BEZIER) && (m_AngleRepresentation == QUATERNION))
          BezierInterpolationQuaternion(pInputMotion, *pOutputMotion, N);
      else
      {
          printf("Error: unknown interpolation / angle representation type.\n");
          exit(1);
      }
  }
  else {
      if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == EULER))
          KeyFrameLinearInterpolationEuler(pInputMotion, *pOutputMotion);
      else if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == QUATERNION))
          KeyFrameLinearInterpolationQuaternion(pInputMotion, *pOutputMotion);
      else if ((m_InterpolationType == BEZIER) && (m_AngleRepresentation == EULER))
          KeyFrameBezierInterpolationEuler(pInputMotion, *pOutputMotion);
      else if ((m_InterpolationType == BEZIER) && (m_AngleRepresentation == QUATERNION))
          KeyFrameBezierInterpolationQuaternion(pInputMotion, *pOutputMotion);
      else
      {
          printf("Error: unknown interpolation / angle representation type.\n");
          exit(1);
      }
  }
}

void Interpolator::LinearInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

  int startKeyframe = 0;
  while (startKeyframe + N + 1 < inputLength)
  {
    int endKeyframe = startKeyframe + N + 1;

    Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
    Posture * endPosture = pInputMotion->GetPosture(endKeyframe);

    // copy start and end keyframe
    pOutputMotion->SetPosture(startKeyframe, *startPosture);
    pOutputMotion->SetPosture(endKeyframe, *endPosture);

    // interpolate in between
    for(int frame=1; frame<=N; frame++)
    {
      Posture interpolatedPosture;
      double t = 1.0 * frame / (N+1);

      // interpolate root position
      interpolatedPosture.root_pos = Lerp(t, startPosture->root_pos, endPosture->root_pos);

      // interpolate bone rotations
      for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
        interpolatedPosture.bone_rotation[bone] = Lerp(t, startPosture->bone_rotation[bone], endPosture->bone_rotation[bone]);

      pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
    }

    startKeyframe = endKeyframe;
  }

  for(int frame=startKeyframe+1; frame<inputLength; frame++)
    pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

vector Interpolator::Rotation2Euler(double R[9])
{
    vector ea;
  double cy = sqrt(R[0]*R[0] + R[3]*R[3]);

  if (cy > 16*DBL_EPSILON) 
  {
    ea.p[0] = atan2(R[7], R[8]);
    ea.p[1] = atan2(-R[6], cy);
    ea.p[2] = atan2(R[3], R[0]);
  } 
  else 
  {
      ea.p[0] = atan2(-R[5], R[4]);
      ea.p[1] = atan2(-R[6], cy);
      ea.p[2] = 0;
  }

  for(int i=0; i<3; i++)
      ea.p[i] *= 180 / M_PI;
  return ea;
}

void Interpolator::Euler2Rotation(vector angles, double R[9])
{
    double x = DEG2RAD(angles.p[0]); // Roll (X)
    double y = DEG2RAD(angles.p[1]); // Pitch (Y)
    double z = DEG2RAD(angles.p[2]); // Yaw (Z)

    double cx = cos(x), sx = sin(x);
    double cy = cos(y), sy = sin(y);
    double cz = cos(z), sz = sin(z);

    R[0] = cy * cz;
    R[1] = sx * sy * cz - cx * sz;
    R[2] = cx * sy * cz + sx * sz;

    R[3] = cy * sz;
    R[4] = sx * sy * sz + cx * cz;
    R[5] = cx * sy * sz - sx * cz;

    R[6] = -sy;
    R[7] = sx * cy;
    R[8] = cx * cy;
}

void Interpolator::BezierInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
    int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

    int startKeyframe = 0, endKeyframe = 0;
    Posture *startPosture, *endPosture;
    Posture* prevPosture, * nextPosture;

    vector c1, c2, an, bn;
    vector pp, sp, ep, np;

#define LerpAvgFuturePoint(stp, enp, fip)\
    Lerp(0.5, Lerp(2.0, stp, enp), fip)\

#define DeCasteljauInterpolate(t, interpolatedPosture, propertyName_)\
    sp = startPosture->propertyName_;\
    ep = endPosture->propertyName_;\
    if (prevPosture != nullptr) {\
        pp = prevPosture->propertyName_;\
        an = LerpAvgFuturePoint(pp, sp, ep);\
        c1 = Lerp(1 / 3, sp, an);\
    } else {\
        c1 = Lerp(1 / 3, sp, Lerp(2.0, nextPosture->propertyName_, ep));\
    }\
    if (nextPosture != nullptr) {\
        np = nextPosture->propertyName_; \
        bn = LerpAvgFuturePoint(sp, ep, np); \
        c2 = Lerp(1 / 3, ep, bn); \
    } else { \
        c2 = Lerp(1 / 3, ep, Lerp(2.0, prevPosture->propertyName_, sp)); \
    }\
    interpolatedPosture.propertyName_ = DeCasteljauEuler(t, sp, c1, c2, ep);\

    while (startKeyframe + N + 1 < inputLength)
    {
        endKeyframe = startKeyframe + N + 1;

        startPosture = pInputMotion->GetPosture(startKeyframe);
        endPosture = pInputMotion->GetPosture(endKeyframe);

        if (startKeyframe > 0) {
            prevPosture = pInputMotion->GetPosture(startKeyframe - 1);
        }
        else
        {
            prevPosture = nullptr;
        }

        if (endKeyframe < inputLength - 1) {
            nextPosture = pInputMotion->GetPosture(endKeyframe + 1);
        }
        else {
            nextPosture = nullptr;
        }

        pOutputMotion->SetPosture(startKeyframe, *startPosture);
        pOutputMotion->SetPosture(endKeyframe, *endPosture);

        for (int frame = 1; frame <= N; frame++)
        {
            Posture interpolatedPosture;
            double t = 1.0 * frame / (N + 1);

            DeCasteljauInterpolate(t, interpolatedPosture, root_pos)

            for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++) {
                DeCasteljauInterpolate(t, interpolatedPosture, bone_rotation[bone])
            }

            pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
        }

        startKeyframe = endKeyframe;
    }

    for (int frame = startKeyframe + 1; frame < inputLength; frame++)
        pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::LinearInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
    int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

    int startKeyframe = 0;
    while (startKeyframe + N + 1 < inputLength)
    {
        int endKeyframe = startKeyframe + N + 1;

        Posture* startPosture = pInputMotion->GetPosture(startKeyframe);
        Posture* endPosture = pInputMotion->GetPosture(endKeyframe);

        pOutputMotion->SetPosture(startKeyframe, *startPosture);
        pOutputMotion->SetPosture(endKeyframe, *endPosture);

        for (int frame = 1; frame <= N; frame++)
        {
            Posture interpolatedPosture;
            double t = 1.0 * frame / (N + 1);

            interpolatedPosture.root_pos = Quaternion2Euler(Slerp(t, Euler2Quaternion(startPosture->root_pos), Euler2Quaternion(endPosture->root_pos)));

            // interpolate bone rotations
            for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
                interpolatedPosture.bone_rotation[bone] = Quaternion2Euler(Slerp(t, Euler2Quaternion(startPosture->bone_rotation[bone]), Euler2Quaternion(endPosture->bone_rotation[bone])));

            pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
        }

        startKeyframe = endKeyframe;
    }

    for (int frame = startKeyframe + 1; frame < inputLength; frame++)
        pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::BezierInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
    int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

    int startKeyframe = 0, endKeyframe = 0;
    Posture* startPosture, * endPosture;
    Posture* prevPosture, * nextPosture;

    Quaternion<double> c1, c2, an, bn;
    Quaternion<double> pp, sp, ep, np;

#define SlerpAvgFuturePoint(stp, enp, fip)\
    Slerp(0.5, Slerp(2.0, stp, enp), fip)\


#define DeCasteljauInterpolate(t, interpolatedPosture, propertyName_)\
    sp = Euler2Quaternion(startPosture->propertyName_);\
    ep = Euler2Quaternion(endPosture->propertyName_);\
    if (prevPosture != nullptr) {\
        pp = Euler2Quaternion(prevPosture->propertyName_);\
        an = SlerpAvgFuturePoint(pp, sp, ep);\
        c1 = Slerp(1 / 3, sp, an);\
    } else {\
        c1 = Slerp(1 / 3, sp, Slerp(2.0, Euler2Quaternion(nextPosture->propertyName_), ep));\
    }\
    if (nextPosture != nullptr) {\
        np = Euler2Quaternion(nextPosture->propertyName_); \
        bn = SlerpAvgFuturePoint(sp, ep, np); \
        c2 = Slerp(1 / 3, ep, bn); \
    } else { \
        c2 = Slerp(1 / 3, ep, Slerp(2.0, Euler2Quaternion(prevPosture->propertyName_), sp)); \
    }\
    interpolatedPosture.propertyName_ = Quaternion2Euler(DeCasteljauQuaternion(t, sp, c1, c2, ep));\

    while (startKeyframe + N + 1 < inputLength)
    {
        endKeyframe = startKeyframe + N + 1;

        startPosture = pInputMotion->GetPosture(startKeyframe);
        endPosture = pInputMotion->GetPosture(endKeyframe);

        if (startKeyframe > 0) {
            prevPosture = pInputMotion->GetPosture(startKeyframe - 1);
        }
        else
        {
            prevPosture = nullptr;
        }

        if (endKeyframe < inputLength - 1) {
            nextPosture = pInputMotion->GetPosture(endKeyframe + 1);
        }
        else {
            nextPosture = nullptr;
        }

        pOutputMotion->SetPosture(startKeyframe, *startPosture);
        pOutputMotion->SetPosture(endKeyframe, *endPosture);

        for (int frame = 1; frame <= N; frame++)
        {
            Posture interpolatedPosture;
            double t = 1.0 * frame / (N + 1);

            DeCasteljauInterpolate(t, interpolatedPosture, root_pos)

                for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++) {
                    DeCasteljauInterpolate(t, interpolatedPosture, bone_rotation[bone])
                }

            pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
        }

        startKeyframe = endKeyframe;
    }

    for (int frame = startKeyframe + 1; frame < inputLength; frame++)
        pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::KeyFrameLinearInterpolationEuler(Motion* pInputMotion, Motion* pOutputMotion)
{
    int inputLength = pInputMotion->GetNumFrames();

#define KeyFrameInterpolation(propertyCheckName_, propertyName_)\
    int startFrame = -1;\
    for (int i = 0; i < inputLength; i++) {\
        Posture* curPosture = pInputMotion->GetPosture(i);\
        if (!curPosture->propertyCheckName_) {\
            startFrame = i;\
            break;\
        }\
    }\
    if(startFrame!=-1){\
    while (startFrame < inputLength - 1) {\
        int endFrame = -1;\
        for (int i = startFrame + 1; i < inputLength; i++) {\
            Posture* curPosture = pInputMotion->GetPosture(i);\
            if (!curPosture->propertyCheckName_) {\
                endFrame = i;\
                break;\
            }\
        }\
        if(endFrame == -1){\
            break;\
        }\
        int N = endFrame - startFrame - 1;\
        Posture* startPosture = pInputMotion->GetPosture(startFrame);\
        Posture* endPosture = pInputMotion->GetPosture(endFrame);\
        Posture* startOutputPosture = pOutputMotion->GetPosture(startFrame);\
        Posture* endOutputPosture = pOutputMotion->GetPosture(endFrame);\
        startOutputPosture->propertyName_ = startPosture->propertyName_;\
        endOutputPosture->propertyName_ = endPosture->propertyName_;\
        for (int frame = 1; frame <= N; frame++)\
        {\
            Posture* interpolatedOutputPosture = pOutputMotion->GetPosture(startFrame + frame);\
            double t = 1.0 * frame / (N + 1);\
            interpolatedOutputPosture->propertyName_ = Lerp(t, startPosture->propertyName_, endPosture->propertyName_);\
        }\
        startFrame = endFrame;\
    }\
    }\

    KeyFrameInterpolation(default_root_pos, root_pos)
    //KeyFrameInterpolation(default_bone_placements[bone], bone_rotation[bone])
    
    for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++) {
        KeyFrameInterpolation(default_bone_placements[bone], bone_rotation[bone])
    }
}

void Interpolator::KeyFrameBezierInterpolationEuler(Motion* pInputMotion, Motion* pOutputMotion)
{
    int inputLength = pInputMotion->GetNumFrames();

    Posture* startPosture, * endPosture;
    Posture* prevPosture, * nextPosture;

    vector c1, c2, an, bn;
    vector pp, sp, ep, np;

#define LerpAvgFuturePoint(stp, enp, fip)\
    Lerp(0.5, Lerp(2.0, stp, enp), fip)\

#define DeCasteljauInterpolate(t, interpolatedPosture, propertyName_)\
    sp = startPosture->propertyName_;\
    ep = endPosture->propertyName_;\
    if (prevPosture != nullptr) {\
        pp = prevPosture->propertyName_;\
        an = LerpAvgFuturePoint(pp, sp, ep);\
        c1 = Lerp(1 / 3, sp, an);\
    } else {\
        c1 = Lerp(1 / 3, sp, Lerp(2.0, nextPosture->propertyName_, ep));\
    }\
    if (nextPosture != nullptr) {\
        np = nextPosture->propertyName_; \
        bn = LerpAvgFuturePoint(sp, ep, np); \
        c2 = Lerp(1 / 3, ep, bn); \
    } else { \
        c2 = Lerp(1 / 3, ep, Lerp(2.0, prevPosture->propertyName_, sp)); \
    }\
    interpolatedPosture->propertyName_ = DeCasteljauEuler(t, sp, c1, c2, ep);\

#define KeyFrameInterpolation(propertyCheckName_, propertyName_)\
    int startFrame = -1;\
    for (int i = 0; i < inputLength; i++) {\
        Posture* curPosture = pInputMotion->GetPosture(i);\
        if (!curPosture->propertyCheckName_) {\
            startFrame = i;\
            break;\
        }\
    }\
    if(startFrame!=-1){\
    while (startFrame < inputLength - 1) {\
        int endFrame = -1;\
        for (int i = startFrame + 1; i < inputLength; i++) {\
            Posture* curPosture = pInputMotion->GetPosture(i);\
            if (!curPosture->propertyCheckName_) {\
                endFrame = i;\
                break;\
            }\
        }\
        if(endFrame == -1){\
            break;\
        }\
        int N = endFrame - startFrame - 1;\
        Posture* startPosture = pInputMotion->GetPosture(startFrame);\
        Posture* endPosture = pInputMotion->GetPosture(endFrame);\
        Posture* startOutputPosture = pOutputMotion->GetPosture(startFrame);\
        Posture* endOutputPosture = pOutputMotion->GetPosture(endFrame);\
        startOutputPosture->propertyName_ = startPosture->propertyName_;\
        endOutputPosture->propertyName_ = endPosture->propertyName_;\
        for (int frame = 1; frame <= N; frame++)\
        {\
            Posture* interpolatedOutputPosture = pOutputMotion->GetPosture(startFrame + frame);\
            double t = 1.0 * frame / (N + 1);\
            DeCasteljauInterpolate(t, interpolatedOutputPosture, propertyName_)\
        }\
        startFrame = endFrame;\
    }\
    }\


    KeyFrameInterpolation(default_root_pos, root_pos)

    for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++) {
        KeyFrameInterpolation(default_bone_placements[bone], bone_rotation[bone])
    }
}

void Interpolator::KeyFrameLinearInterpolationQuaternion(Motion* pInputMotion, Motion* pOutputMotion)
{
    int inputLength = pInputMotion->GetNumFrames();

#define KeyFrameInterpolation(propertyCheckName_, propertyName_)\
    int startFrame = -1;\
    for (int i = 0; i < inputLength; i++) {\
        Posture* curPosture = pInputMotion->GetPosture(i);\
        if (!curPosture->propertyCheckName_) {\
            startFrame = i;\
            break;\
        }\
    }\
    if(startFrame!=-1){\
    while (startFrame < inputLength - 1) {\
        int endFrame = -1;\
        for (int i = startFrame + 1; i < inputLength; i++) {\
            Posture* curPosture = pInputMotion->GetPosture(i);\
            if (!curPosture->propertyCheckName_) {\
                endFrame = i;\
                break;\
            }\
        }\
        if(endFrame == -1){\
            break;\
        }\
        int N = endFrame - startFrame - 1;\
        Posture* startPosture = pInputMotion->GetPosture(startFrame);\
        Posture* endPosture = pInputMotion->GetPosture(endFrame);\
        Posture* startOutputPosture = pOutputMotion->GetPosture(startFrame);\
        Posture* endOutputPosture = pOutputMotion->GetPosture(endFrame);\
        startOutputPosture->propertyName_ = startPosture->propertyName_;\
        endOutputPosture->propertyName_ = endPosture->propertyName_;\
        for (int frame = 1; frame <= N; frame++)\
        {\
            Posture* interpolatedOutputPosture = pOutputMotion->GetPosture(startFrame + frame);\
            double t = 1.0 * frame / (N + 1);\
            interpolatedOutputPosture->propertyName_ = Quaternion2Euler(Slerp(t, Euler2Quaternion(startPosture->propertyName_), Euler2Quaternion(endPosture->propertyName_)));\
        }\
        startFrame = endFrame;\
    }\
    }\

    KeyFrameInterpolation(default_root_pos, root_pos)
    
    for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++) {
        KeyFrameInterpolation(default_bone_placements[bone], bone_rotation[bone])
    }
}

void Interpolator::KeyFrameBezierInterpolationQuaternion(Motion* pInputMotion, Motion* pOutputMotion)
{

    int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

    int startKeyframe = 0, endKeyframe = 0;
    Posture* startPosture, * endPosture;
    Posture* prevPosture, * nextPosture;

    Quaternion<double> c1, c2, an, bn;
    Quaternion<double> pp, sp, ep, np;

#define SlerpAvgFuturePoint(stp, enp, fip)\
    Slerp(0.5, Slerp(2.0, stp, enp), fip)\


#define DeCasteljauInterpolate(t, interpolatedPosture, propertyName_)\
    sp = Euler2Quaternion(startPosture->propertyName_);\
    ep = Euler2Quaternion(endPosture->propertyName_);\
    if (prevPosture != nullptr) {\
        pp = Euler2Quaternion(prevPosture->propertyName_);\
        an = SlerpAvgFuturePoint(pp, sp, ep);\
        c1 = Slerp(1 / 3, sp, an);\
    } else {\
        c1 = Slerp(1 / 3, sp, Slerp(2.0, Euler2Quaternion(nextPosture->propertyName_), ep));\
    }\
    if (nextPosture != nullptr) {\
        np = Euler2Quaternion(nextPosture->propertyName_); \
        bn = SlerpAvgFuturePoint(sp, ep, np); \
        c2 = Slerp(1 / 3, ep, bn); \
    } else { \
        c2 = Slerp(1 / 3, ep, Slerp(2.0, Euler2Quaternion(prevPosture->propertyName_), sp)); \
    }\
    interpolatedPosture->propertyName_ = Quaternion2Euler(DeCasteljauQuaternion(t, sp, c1, c2, ep));\

#define KeyFrameInterpolation(propertyCheckName_, propertyName_)\
    int startFrame = -1;\
    for (int i = 0; i < inputLength; i++) {\
        Posture* curPosture = pInputMotion->GetPosture(i);\
        if (!curPosture->propertyCheckName_) {\
            startFrame = i;\
            break;\
        }\
    }\
    if(startFrame!=-1){\
    while (startFrame < inputLength - 1) {\
        int endFrame = -1;\
        for (int i = startFrame + 1; i < inputLength; i++) {\
            Posture* curPosture = pInputMotion->GetPosture(i);\
            if (!curPosture->propertyCheckName_) {\
                endFrame = i;\
                break;\
            }\
        }\
        if(endFrame == -1){\
            break;\
        }\
        int N = endFrame - startFrame - 1;\
        Posture* startPosture = pInputMotion->GetPosture(startFrame);\
        Posture* endPosture = pInputMotion->GetPosture(endFrame);\
        Posture* startOutputPosture = pOutputMotion->GetPosture(startFrame);\
        Posture* endOutputPosture = pOutputMotion->GetPosture(endFrame);\
        startOutputPosture->propertyName_ = startPosture->propertyName_;\
        endOutputPosture->propertyName_ = endPosture->propertyName_;\
        for (int frame = 1; frame <= N; frame++)\
        {\
            Posture* interpolatedOutputPosture = pOutputMotion->GetPosture(startFrame + frame);\
            double t = 1.0 * frame / (N + 1);\
            DeCasteljauInterpolate(t, interpolatedOutputPosture, propertyName_)\
        }\
        startFrame = endFrame;\
    }\
    }\


    KeyFrameInterpolation(default_root_pos, root_pos)

    for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++) {
        KeyFrameInterpolation(default_bone_placements[bone], bone_rotation[bone])
    }

}


Quaternion<double> Interpolator::Euler2Quaternion(vector angles)
{
    double R[9];
    Euler2Rotation(angles, R);
    Quaternion<double> q = Quaternion<double>::Matrix2Quaternion(R);
    return q;
}

vector Interpolator::Quaternion2Euler(Quaternion<double> q)
{
    double R[9];
    q.Quaternion2Matrix(R);
    return Rotation2Euler(R);
}

vector Interpolator::Lerp(double t, vector start, vector end) {
    return start * (1 - t) + end * t;
}

Quaternion<double> Interpolator::Lerp(double t, Quaternion<double>& qStart, Quaternion<double>& qEnd_) {
    Quaternion<double> result;

    double cosA = Quaternion<double>::DotProduct(qStart, qEnd_);
    if (cosA < 0) {
        qEnd_ = qEnd_ * -1;
        cosA *= -1;
    }

    result = (1 - t) * qStart + t * qEnd_;
    result.Normalize();

    return result;
}

Quaternion<double> Interpolator::Slerp(double t, Quaternion<double> & qStart, Quaternion<double> & qEnd_)
{
  Quaternion<double> result;

  double cosA = Quaternion<double>::DotProduct(qStart, qEnd_);
  if (cosA < 0) {
      qEnd_ = qEnd_ * -1;
      cosA *= -1;
  }

  if (cosA > 0.999f) {
      return Lerp(t, qStart, qEnd_);
  }

  double sinA = sqrt(1 - cosA * cosA);
  double A = atan2(sinA, cosA);
  double oneBysinA = 1 / sinA;

  result = sinf((1.0f - t) * A) * oneBysinA * qStart + sinf(t * A) * oneBysinA * qEnd_;
  result.Normalize();//Just added in case of floating point errors
  return result;
}

Quaternion<double> Interpolator::Double(Quaternion<double> p, Quaternion<double> q)
{
  Quaternion<double> result;
  return result;
}

vector Interpolator::DeCasteljauEuler(double t, vector p0, vector p1, vector p2, vector p3)
{
   vector result =  p0 * pow(1 - t, 3) +  p1 * (3* pow(1 - t, 2) * t) +  p2 * (3 * (1 - t) * pow(t, 2)) +  p3 * pow(t, 3);
   return result;
}

Quaternion<double> Interpolator::DeCasteljauQuaternion(double t, Quaternion<double> p0, Quaternion<double> p1, Quaternion<double> p2, Quaternion<double> p3)
{
  Quaternion<double> result;

  Quaternion<double> q0 = Slerp(t, p0, p1);
  Quaternion<double> q1 = Slerp(t, p1, p2);
  Quaternion<double> q2 = Slerp(t, p2, p3);

  Quaternion<double> r0 = Slerp(t, q0, q1);
  Quaternion<double> r1 = Slerp(t, q1, q2);

  result = Slerp(t, r0, r1);

  return result;
}

