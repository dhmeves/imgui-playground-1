/**
 * @file fsciks.h
 * @author Daniel Meves
 * @date June 23 2025
 * @brief File containing inverse kinematics functions related to Series RC40
 *          controllers from Rexroth
 */

#pragma once

//#include "fsc_lib.h"
#include <iostream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include "math.h"
#include "fsc_madgwick.h"


struct Point
{
    float x;
    float y;
    float z;
};

struct Vector_Rotation
{
    // Find better names for each point
    Point one;
    Point two;
    Point three;
};

struct Jacobian
{
    float delWnaut;
    float delXnaut;
    float delYnaut;
    float delZnaut;
    float delWone;
    float delXone;
    float delYone;
    float delZone;
};

struct Polar_Coordinates
{
    uint16_t hypotenuse;  // r or rho
    float betaRoll;     // theta angle
    float betaPitch;    // alpha angle
    float betaYaw;      // phi angle
};

struct FSCIKS_OUTPUT
{
    Point xOut;
    Point yOut;
    Point zOut;
    Polar_Coordinates polarOut;
};

class Fsciks_dan
{
public:
    enum LINK_DIRECTION_e
    {
        forward,
        backward,
        NUM_LINK_DIRECTIONS
    };
      struct THREE_DOF_ROT
      {
          float roll;
          float pitch;
          float yaw;
      };

      struct Linkage
      {
          struct Linkage* next_link;
          struct Linkage* prev_link;
          uint16_t length;
          Quaternion origin;
          Quaternion q;
          THREE_DOF_ROT angles;
      };




      struct FELLER_BUNCHER_s
      {
          Linkage boom;
          Linkage arm;
          Linkage grabber;
      };

      FELLER_BUNCHER_s fellBunch;
      Quaternion og;
      Quaternion unitQ;

      void initLinkage(Linkage* link, Linkage* prev_link, Linkage* next_link);
      void updateLinkageLength(Linkage* link, uint16_t newLength);
      void updateLinkageOrigin(Linkage* link, Quaternion* origin);
      void updateLinkageQuaternion(Linkage* link, Quaternion* newQ);
      void computeLinkageAngles(Linkage* link);
      Quaternion rotationAroundX(float alpha);
      Quaternion rotationAroundY(float phi);
      Quaternion rotationAroundZ(float theta);
      Quaternion quaternionMultiplication(Quaternion* q1, Quaternion* q2);
      Quaternion conjugate(Quaternion* q);
      Quaternion conjugateWithUnitAndTheta(float theta);
      Vector_Rotation conjugateMultiplication(Quaternion* q);
      Point newPointAfterRotation(Point* currentPoint, Vector_Rotation* vRot);
      float pythagorasTheorem(float x, float y);
      float lawOfCosines(float a, float b, float angC);
      float q2SolOne(float length1, float length2, float x, float y);
      float q2SolTwo(float length1, float length2, float x, float y);
      float q1Sol(float length1, float length2, float x, float y, float q2);
      Point getEffectorPosition(Polar_Coordinates polar3d);
      Jacobian constructJacobian(Quaternion qNaut, Quaternion qOne);
      void jacobianDeterminant(Jacobian* jac, float* det, float* invDet);
      Quaternion q2SolViaJacobianAndConjugate(Quaternion* q1, Jacobian* jac);
      Polar_Coordinates accumulate(Linkage* link, LINK_DIRECTION_e dir, float bRoll, float bPitch, float bYaw, uint16_t prev_hyp);
      void initializeIK(void);
      FSCIKS_OUTPUT computeIK(void);
      Polar_Coordinates quaternionToPolar(Quaternion* q);
      float normalizedQuaternion(Quaternion* q);
};
