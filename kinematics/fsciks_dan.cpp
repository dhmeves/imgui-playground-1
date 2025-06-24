#include "fsciks_dan.h"


void Fsciks_dan::initLinkage(Linkage * link, Linkage * prev_link, Linkage * next_link)
{
  // Doublely linked list of linkage nodes
  link->prev_link = prev_link;
  link->next_link = next_link;

  og.w = 0.0f;
  og.x = 0.0f;
  og.y = 0.0f;
  og.z = 0.0f;

  unitQ.w = 1.0f;
  unitQ.x = 0.0f;
  unitQ.y = 0.0f;
  unitQ.z = 0.0f;

  Quaternion * newQ;
  newQ->w = 0.0f;
  newQ->x = 0.0f;
  newQ->y = 0.0f;
  newQ->z = 0.0f;

  THREE_DOF_ROT * initAng;
  initAng->roll = 0.0f;
  initAng->pitch = 0.0f;
  initAng->yaw = 0.0f;

  link->origin = og;
  link->q = *newQ;
  link->angles = *initAng;
  link->length = 0;
  return;
}

void Fsciks_dan::updateLinkageLength(Linkage * link, uint16_t newLength)
{ 
  link->length = newLength;
  return;
}

void Fsciks_dan::updateLinkageOrigin(Linkage * link, Quaternion * origin) // Origin may not be 0.0,0.0,0.0, etc...
{
  link->origin = *origin;
  return;
}

void Fsciks_dan::updateLinkageQuaternion(Linkage * link, Quaternion * newQ)
{
  link->q = *newQ;
  return;
}

void Fsciks_dan::computeLinkageAngles(Linkage * link) // compute and store in linkage struct
{
  computeIMUAngles(link->q, &link->angles.roll, &link->angles.pitch, &link->angles.yaw);
  return;
}

// https://www.youtube.com/watch?v=0FbDyWXemLw

float Fsciks_dan::normalizedQuaternion(Quaternion * q)
{
  float normQ = fsc_sqrt(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
  return normQ;
}

Quaternion Fsciks_dan::rotationAroundX(float alpha) // Pitch in polar angles to quaternions
{
  Quaternion rot;

  rot.w = cos(alpha / 2);
  rot.x = sin(alpha / 2);
  rot.y = 0;
  rot.z = 0;

  return rot;
}

Quaternion Fsciks_dan::rotationAroundY(float phi) // Yaw in polar angles to quaternions
{
  Quaternion rot;

  rot.w = cos(phi / 2);
  rot.x = 0;
  rot.y = sin(phi / 2);
  rot.z = 0;

  return rot;
}

Quaternion Fsciks_dan::rotationAroundZ(float theta) // Roll in polar angles to quaternions
{
  Quaternion rot;

  rot.w = cos(theta / 2);
  rot.x = 0;
  rot.y = 0;
  rot.z = sin(theta / 2);

  return rot;
}

Quaternion Fsciks_dan::quaternionMultiplication(Quaternion * q1, Quaternion * q2)
{
  Quaternion composition;

  composition.w = q1->w * q2->w - q1->x * q2->x - q1->y * q2->y - q1->z * q2->z;
  composition.x = q1->w * q2->x + q1->x * q2->w + q1->y * q2->z - q1->z * q2->y;
  composition.y = q1->w * q2->y - q1->x * q2->z + q1->y * q2->w + q1->z * q2->x;
  composition.z = q1->w * q2->z + q1->x * q2->y - q1->y * q2->x + q1->z * q2->w;

  return composition;
}

Quaternion Fsciks_dan::conjugate(Quaternion * q)
{
  Quaternion conj;

  conj.w = q->w;
  conj.x = -q->x;
  conj.y = -q->y;
  conj.z = -q->z;

  return conj;
}

Quaternion Fsciks_dan::conjugateWithUnitAndTheta(float theta) // !TODO (DHM) - REMOVE IF NOT NEEDED
{
  Quaternion conj; 
  
  //conj = cos(theta / 2) - sin(theta / 2) * unitQ;

  return conj;
}

Vector_Rotation Fsciks_dan::conjugateMultiplication(Quaternion * q) // Convert Quaternion Rotation to Cartesian Rotation
{
  Vector_Rotation vRot;
  Point one;
  Point two;
  Point three;

  one.x = 1 - 2 * q->y * q->y - 2 * q->z * q->z;
  one.y = 2 * q->x * q->y - 2 * q->z * q->w;
  one.z = 2 * q->x * q->z + 2 * q->y * q->w;

  two.x = 2 * q->x * q->y + 2 * q->z * q->w;
  two.y = 1 - 2 * q->x * q->x - 2 * q->z * q->z;
  two.z = 2 * q->y * q->z - 2 * q->x * q->w;

  three.x = 2 * q->x * q->z - 2 * q->y * q->w;
  three.y = 2 * q->y * q->z + 2 * q->x * q->w;
  three.z = 1 - 2 * q->x * q->x - 2 * q->y * q->y;

  vRot.one = one;
  vRot.two = two;
  vRot.three = three;

  return vRot;
}

Point Fsciks_dan::newPointAfterRotation(Point * currentPoint, Vector_Rotation * vRot) // Find new point location in cartesian after rotation
{
  Point pPrime;

  pPrime.x = vRot->one.x * currentPoint->x + vRot->two.x * currentPoint->x + vRot->three.x * currentPoint->x;
  pPrime.y = vRot->one.y * currentPoint->y + vRot->two.y * currentPoint->y + vRot->three.y * currentPoint->y;
  pPrime.z = vRot->one.z * currentPoint->z + vRot->two.z * currentPoint->z + vRot->three.z * currentPoint->z;

  return pPrime;
}

// Inverse Kinematics of a 2-link Planar Robot
// https://www.youtube.com/watch?v=RH3iAmMsolo

#define M_PI 3.14159

float Fsciks_dan::pythagorasTheorem(float x, float y)
{
  float d = fsc_sqrt(x * x + y * y);
  return d;
}

float Fsciks_dan::lawOfCosines(float a, float b, float angC) // Use to find hypotenuse from linkage origin
{
  float c = fsc_sqrt(a * a + b* b - 2 * a * b * cos(angC));
  return c;
}

float Fsciks_dan::q2SolOne(float length1, float length2, float x, float y) // Rotation Angle possible solution one - only for testing and debugging to compare to quaternion solution - MAY NOT CONVERGE
{
  float q2 = M_PI - acos((length1 * length1 + length2 * length2 - x * x - y * y) / (2 * length1 * length2));
  return q2;
}

float Fsciks_dan::q2SolTwo(float length1, float length2, float x, float y) // Rotation Angle possible solution two - only for testing and debugging to compare to quaternion solution - MAY NOT CONVERGE
{
  float q2 = acos((x * x + y * y - length1 * length1 - length2 * length2) / (2 * length1 * length2));
  return q2;
}

float Fsciks_dan::q1Sol(float length1, float length2, float x, float y, float q2) // Rotation Angle q1 solution
{
  float q1 = atan(y / x) - atan((length2 * sin(q2)) / (length1 + length2 * cos(q2)));
  return q1;
}

// Inverse Kinematics: Jacobian Solver
// https://www.youtube.com/watch?v=2_cdDGwnl80
// https://github.com/SpehleonLP/IK-Guide
// https://www.youtube.com/watch?v=-e7muqC3670
// https://docs.quanser.com/quarc/documentation/quaternion_product_jacobian_block.html

Point Fsciks_dan::getEffectorPosition(Polar_Coordinates polar3d) // Works best with accumulated polar coordinates
{
  // Polar to Cartesian
  // r     is the Radius
  // alpha is the horizontal angle from the X axis
  // polar is the vertical angle from the Z axis
  Point cartesian;
  uint16_t r = polar3d.hypotenuse;
  float alpha = polar3d.betaYaw;
  float polar = polar3d.betaRoll;

  cartesian.x = r * sin(polar) * cos(alpha);
  cartesian.y = r * sin(polar) * sin(alpha);
  cartesian.z = r * cos(polar);

  return cartesian;
}

Jacobian Fsciks_dan::constructJacobian(Quaternion qNaut, Quaternion qOne) // partial differentals
{
  Jacobian * jac;
  jac->delWnaut = qNaut.w - qOne.w;
  jac->delWone = qOne.w - qNaut.w;
  jac->delXnaut = qNaut.x - qOne.x;
  jac->delXone = qOne.x - qNaut.x;
  jac->delYnaut = qNaut.y - qOne.y;
  jac->delYone = qOne.y - qNaut.y;
  jac->delZnaut = qNaut.z - qOne.z;
  jac->delZone = qOne.z - qNaut.z;
  return *jac;
}

void Fsciks_dan::jacobianDeterminant(Jacobian * jac, float * det, float * invDet) // use for convergence - https://www.khanacademy.org/math/multivariable-calculus/multivariable-derivatives/jacobian/v/the-jacobian-determinant
{
  float nonInvJacobian[4][4] = 
  {
    { jac->delWnaut / jac->delWone, jac->delWnaut / jac->delXone, jac->delWnaut / jac->delYone, jac->delWnaut / jac->delZone},
    { jac->delXnaut / jac->delWone, jac->delXnaut / jac->delXone, jac->delXnaut / jac->delYone, jac->delXnaut / jac->delZone},
    { jac->delYnaut / jac->delWone, jac->delYnaut / jac->delXone, jac->delYnaut / jac->delYone, jac->delYnaut / jac->delZone},
    { jac->delZnaut / jac->delWone, jac->delZnaut / jac->delXone, jac->delZnaut / jac->delYone, jac->delZnaut / jac->delZone}
  };

  float invJacobian[4][4] = 
  {
    { jac->delWnaut / jac->delWone, jac->delXnaut / jac->delWone, jac->delYnaut / jac->delWone, jac->delZnaut / jac->delWone},
    { jac->delWnaut / jac->delXone, jac->delXnaut / jac->delXone, jac->delYnaut / jac->delXone, jac->delZnaut / jac->delXone},
    { jac->delWnaut / jac->delYone, jac->delXnaut / jac->delYone, jac->delYnaut / jac->delYone, jac->delZnaut / jac->delYone},
    { jac->delWnaut / jac->delZone, jac->delXnaut / jac->delZone, jac->delYnaut / jac->delZone, jac->delZnaut / jac->delZone}
  };

  *det = (nonInvJacobian[0][0] * (((nonInvJacobian[1][1] * nonInvJacobian[2][2] * nonInvJacobian[3][3]) + (nonInvJacobian[1][2] * nonInvJacobian[2][3] * nonInvJacobian[3][0]) + (nonInvJacobian[1][3] * nonInvJacobian[2][0] * nonInvJacobian[3][1]))
                                - ((nonInvJacobian[3][1] * nonInvJacobian[2][2] * nonInvJacobian[1][3]) + (nonInvJacobian[3][2] * nonInvJacobian[2][3] * nonInvJacobian[1][0]) + (nonInvJacobian[3][3] * nonInvJacobian[2][0] * nonInvJacobian[1][1])))
          - nonInvJacobian[1][0] * (((nonInvJacobian[0][1] * nonInvJacobian[2][2] * nonInvJacobian[3][3]) + (nonInvJacobian[0][2] * nonInvJacobian[2][3] * nonInvJacobian[3][0]) + (nonInvJacobian[0][3] * nonInvJacobian[2][0] * nonInvJacobian[3][1]))
                                - ((nonInvJacobian[3][1] * nonInvJacobian[2][2] * nonInvJacobian[0][3]) + (nonInvJacobian[3][2] * nonInvJacobian[2][3] * nonInvJacobian[0][0]) + (nonInvJacobian[3][3] * nonInvJacobian[2][0] * nonInvJacobian[0][1])))
          + nonInvJacobian[2][0] * (((nonInvJacobian[0][1] * nonInvJacobian[1][2] * nonInvJacobian[3][3]) + (nonInvJacobian[0][2] * nonInvJacobian[1][3] * nonInvJacobian[3][0]) + (nonInvJacobian[0][3] * nonInvJacobian[1][0] * nonInvJacobian[3][1]))
                                - ((nonInvJacobian[3][1] * nonInvJacobian[1][2] * nonInvJacobian[0][3]) + (nonInvJacobian[3][2] * nonInvJacobian[1][3] * nonInvJacobian[0][0]) + (nonInvJacobian[3][3] * nonInvJacobian[1][0] * nonInvJacobian[0][1])))
          - nonInvJacobian[3][0] * (((nonInvJacobian[0][1] * nonInvJacobian[1][2] * nonInvJacobian[2][3]) + (nonInvJacobian[0][2] * nonInvJacobian[1][3] * nonInvJacobian[2][0]) + (nonInvJacobian[0][3] * nonInvJacobian[1][0] * nonInvJacobian[2][1]))
                                - ((nonInvJacobian[2][1] * nonInvJacobian[1][2] * nonInvJacobian[0][3]) + (nonInvJacobian[2][2] * nonInvJacobian[1][3] * nonInvJacobian[0][0]) + (nonInvJacobian[2][3] * nonInvJacobian[1][0] * nonInvJacobian[0][1]))));

  *invDet = (invJacobian[0][0] * (((invJacobian[1][1] * invJacobian[2][2] * invJacobian[3][3]) + (invJacobian[1][2] * invJacobian[2][3] * invJacobian[3][0]) + (invJacobian[1][3] * invJacobian[2][0] * invJacobian[3][1]))
                                - ((invJacobian[3][1] * invJacobian[2][2] * invJacobian[1][3]) + (invJacobian[3][2] * invJacobian[2][3] * invJacobian[1][0]) + (invJacobian[3][3] * invJacobian[2][0] * invJacobian[1][1])))
          - invJacobian[1][0] * (((invJacobian[0][1] * invJacobian[2][2] * invJacobian[3][3]) + (invJacobian[0][2] * invJacobian[2][3] * invJacobian[3][0]) + (invJacobian[0][3] * invJacobian[2][0] * invJacobian[3][1]))
                                - ((invJacobian[3][1] * invJacobian[2][2] * invJacobian[0][3]) + (invJacobian[3][2] * invJacobian[2][3] * invJacobian[0][0]) + (invJacobian[3][3] * invJacobian[2][0] * invJacobian[0][1])))
          + invJacobian[2][0] * (((invJacobian[0][1] * invJacobian[1][2] * invJacobian[3][3]) + (invJacobian[0][2] * invJacobian[1][3] * invJacobian[3][0]) + (invJacobian[0][3] * invJacobian[1][0] * invJacobian[3][1]))
                                - ((invJacobian[3][1] * invJacobian[1][2] * invJacobian[0][3]) + (invJacobian[3][2] * invJacobian[1][3] * invJacobian[0][0]) + (invJacobian[3][3] * invJacobian[1][0] * invJacobian[0][1])))
          - invJacobian[3][0] * (((invJacobian[0][1] * invJacobian[1][2] * invJacobian[2][3]) + (invJacobian[0][2] * invJacobian[1][3] * invJacobian[2][0]) + (invJacobian[0][3] * invJacobian[1][0] * invJacobian[2][1]))
                                - ((invJacobian[2][1] * invJacobian[1][2] * invJacobian[0][3]) + (invJacobian[2][2] * invJacobian[1][3] * invJacobian[0][0]) + (invJacobian[2][3] * invJacobian[1][0] * invJacobian[0][1]))));

  return;
}

Quaternion Fsciks_dan::q2SolViaJacobianAndConjugate(Quaternion * q1, Jacobian * jac) // Recommended q2 solution - [q2 = q1 * p * conjugate(q1)] - returns normalized quaternion
{
  Quaternion qConj = conjugate(q1);
  float invJacobian[4][4] = 
  {
    { jac->delWnaut / jac->delWone, jac->delXnaut / jac->delWone, jac->delYnaut / jac->delWone, jac->delZnaut / jac->delWone},
    { jac->delWnaut / jac->delXone, jac->delXnaut / jac->delXone, jac->delYnaut / jac->delXone, jac->delZnaut / jac->delXone},
    { jac->delWnaut / jac->delYone, jac->delXnaut / jac->delYone, jac->delYnaut / jac->delYone, jac->delZnaut / jac->delYone},
    { jac->delWnaut / jac->delZone, jac->delXnaut / jac->delZone, jac->delYnaut / jac->delZone, jac->delZnaut / jac->delZone}
  };

  // Vector-Matrix Multiplication
  Quaternion temp;
  temp.w = q1->w * invJacobian[0][0] + q1->w * invJacobian[0][1] + q1->w * invJacobian[0][2] + q1->w * invJacobian[0][3];
  temp.x = q1->x * invJacobian[1][0] + q1->x * invJacobian[1][1] + q1->x * invJacobian[1][2] + q1->x * invJacobian[1][3];
  temp.y = q1->y * invJacobian[2][0] + q1->y * invJacobian[2][1] + q1->y * invJacobian[2][2] + q1->y * invJacobian[2][3];
  temp.z = q1->z * invJacobian[3][0] + q1->z * invJacobian[3][1] + q1->z * invJacobian[3][2] + q1->z * invJacobian[3][3];

  // Quaternion Multiplication with conjugate to normalize
  Quaternion q2 = quaternionMultiplication(&temp, &qConj);
  return q2;
}

// Recursively sum linkage lengths, positions, angles
Polar_Coordinates Fsciks_dan::accumulate(Linkage * link, LINK_DIRECTION_e dir, float bRoll, float bPitch, float bYaw, uint16_t prev_hyp) // !TODO (DHM): could remove bRoll, bPitch, bYaw if not needed
{
 static float betaRoll, betaPitch, betaYaw = 0.0f;
 uint16_t hypotenuse = 0;
 Polar_Coordinates polar;

 if (dir == backward && link->prev_link != NULL) // start at end-effector - work inverse kinematics
 {  // double check if total linkage is computed recursively
  computeLinkageAngles(link);
  betaRoll += link->angles.roll;
  betaPitch += link->angles.pitch;
  betaYaw += link->angles.yaw;

  hypotenuse = lawOfCosines(link->length, prev_hyp, link->angles.yaw);
  accumulate(link->prev_link, backward, betaRoll, betaPitch, betaYaw, hypotenuse);
 }
 else if (dir == forward && link->next_link != NULL) // start at origin - work forwards kinematics
 { // double check if total linkage is computed recursively
  computeLinkageAngles(link);
  betaRoll += link->angles.roll;
  betaPitch += link->angles.pitch;
  betaYaw += link->angles.yaw;

  hypotenuse = lawOfCosines(prev_hyp, link->length, link->angles.yaw);
  accumulate(link->next_link, forward, betaRoll, betaPitch, betaYaw, hypotenuse);
 }
 else
 {
  polar.betaRoll = betaRoll;
  polar.betaPitch = betaPitch;
  polar.betaYaw = betaYaw;
  polar.hypotenuse = hypotenuse;
  betaRoll = 0.0f;
  betaPitch = 0.0f;
  betaYaw = 0.0f;
  return polar;
 }
}

void Fsciks_dan::initializeIK(void) // init function - set up for Feller Buncher arm
{
  // 2-D Planar Robot - Feller Buncher
  initLinkage(&fellBunch.boom, NULL, &fellBunch.arm);
  initLinkage(&fellBunch.arm, &fellBunch.boom, &fellBunch.grabber);
  initLinkage(&fellBunch.grabber, &fellBunch.arm, NULL);

  updateLinkageLength(&fellBunch.boom, 48); // test values
  updateLinkageLength(&fellBunch.arm, 36);
  updateLinkageLength(&fellBunch.grabber, 12);

  updateIMU(&fellBunch.boom.q, 0, 0, 0, 0, 0, 0, 0); // MM7 values
  updateIMU(&fellBunch.arm.q, 0, 0, 0, 0, 0, 0, 0);
  updateIMU(&fellBunch.grabber.q, 0, 0, 0, 0, 0, 0, 0);

  updateLinkageOrigin(&fellBunch.boom, &og);
  updateLinkageOrigin(&fellBunch.arm, &fellBunch.boom.q);
  updateLinkageOrigin(&fellBunch.grabber, &fellBunch.arm.q);

  computeLinkageAngles(&fellBunch.boom);
  computeLinkageAngles(&fellBunch.arm);
  computeLinkageAngles(&fellBunch.grabber);

  return;
}

void Fsciks_dan::computeIK(void) // loop function - do business logic here
{
  updateIMU(&fellBunch.boom.q, 0, 0, 0, 0, 0, 0, 0); // MM7 values
  updateIMU(&fellBunch.arm.q, 0, 0, 0, 0, 0, 0, 0);
  updateIMU(&fellBunch.grabber.q, 0, 0, 0, 0, 0, 0, 0);

  updateLinkageOrigin(&fellBunch.boom, &og);
  updateLinkageOrigin(&fellBunch.arm, &fellBunch.boom.q);
  updateLinkageOrigin(&fellBunch.grabber, &fellBunch.arm.q);

  Quaternion futureBoomQ; // desired rotation
  Quaternion futureArmQ;
  Quaternion futureGrabberQ;

  Jacobian jacBoom = constructJacobian(futureBoomQ, fellBunch.boom.q);
  Jacobian jacArm = constructJacobian(futureArmQ, fellBunch.arm.q);
  Jacobian jacGrabber = constructJacobian(futureGrabberQ, fellBunch.grabber.q);

  float jacBoomDet, jacBoomInvDet, jacArmDet, jacArmInvDet, jacGrabberDet, jacGrabberInvDet; // check convergence
  jacobianDeterminant(&jacBoom, &jacBoomDet, &jacBoomInvDet);
  jacobianDeterminant(&jacArm, &jacArmDet, &jacArmInvDet);
  jacobianDeterminant(&jacGrabber, &jacGrabberDet, &jacGrabberInvDet);
  
  Quaternion boomSP = q2SolViaJacobianAndConjugate(&fellBunch.boom.q, &jacBoom); // PID setpoint - may need converging theorem
  Quaternion armSP = q2SolViaJacobianAndConjugate(&fellBunch.arm.q, &jacArm);
  Quaternion grabberSP = q2SolViaJacobianAndConjugate(&fellBunch.grabber.q, &jacGrabber);

  Polar_Coordinates endEffectorLocation = accumulate(&fellBunch.grabber, backward, 0.0f, 0.0f, 0.0f, 0);
  Point cart = getEffectorPosition(endEffectorLocation);

  Quaternion rotX = rotationAroundX(M_PI); // in radians
  Quaternion rotY = rotationAroundY(M_PI / 2); // in radians
  Quaternion rotZ = rotationAroundZ(M_PI / 4); // in radians

  // Example Manual Mode
  // Move the Boom by the radian angles above
  Jacobian jacRotX = constructJacobian(rotX, fellBunch.boom.q);
  Jacobian jacRotY = constructJacobian(rotY, fellBunch.boom.q);
  Jacobian jacRotZ = constructJacobian(rotZ, fellBunch.boom.q);

  float jacRotXDet, jacRotXInvDet, jacRotYDet, jacRotYInvDet, jacRotZDet, jacRotZInvDet; // check convergence
  jacobianDeterminant(&jacRotX, &jacRotXDet, &jacRotXInvDet);
  jacobianDeterminant(&jacRotY, &jacRotYDet, &jacRotYInvDet);
  jacobianDeterminant(&jacRotZ, &jacRotZDet, &jacRotZInvDet);

  Quaternion boomX = q2SolViaJacobianAndConjugate(&fellBunch.boom.q, &jacRotX); // PID setpoint - may need converging theorem
  Quaternion boomY = q2SolViaJacobianAndConjugate(&fellBunch.boom.q, &jacRotY);
  Quaternion boomZ = q2SolViaJacobianAndConjugate(&fellBunch.boom.q, &jacRotZ);

  Polar_Coordinates boomXpolar = quaternionToPolar(&boomX); // Confirm polar coordinates in testing and debugging
  Polar_Coordinates boomYpolar = quaternionToPolar(&boomY);
  Polar_Coordinates boomZpolar = quaternionToPolar(&boomZ);

  Point cartX = getEffectorPosition(boomXpolar); // Confirm cartesian coordinates in testing and debugging
  Point cartY = getEffectorPosition(boomYpolar);
  Point cartZ = getEffectorPosition(boomZpolar);

  endEffectorLocation = accumulate(&fellBunch.grabber, backward, 0.0f, 0.0f, 0.0f, 0);
  cart = getEffectorPosition(endEffectorLocation);

  return;
}

Polar_Coordinates Fsciks_dan::quaternionToPolar(Quaternion * q) // quick polar check
{
  Polar_Coordinates polar3d;
  polar3d.hypotenuse = normalizedQuaternion(q);
  polar3d.betaRoll = acos(q->z / polar3d.hypotenuse);
  polar3d.betaYaw = acos(q->x / fsc_sqrt(q->x * q->x + q->y * q->y));
  polar3d.betaPitch = (M_PI / 2) - polar3d.betaYaw;
  return polar3d;
}
