#ifndef __QUATERNIONS_H__
#define __QUATERNIONS_H__

#include "vector3.h"

/** Quaternion */
typedef struct
{
    float scalar;
	Vector3 v;
} Quaternion;

Quaternion quat_create(Vector3 axis, float scalar);
Quaternion quat_create_from_axis_angle(Vector3 axis, float angle);
Quaternion quat_create_identity();

Quaternion quat_multiply_quaternion(const Quaternion* q1, const Quaternion* q2);
Quaternion quat_multiply_scalar(const Quaternion* q1, float scalar);
Quaternion quat_add_quaternion(const Quaternion* q1, const Quaternion* q2);
Quaternion quat_subtract_quaternion(const Quaternion* q1, const Quaternion* q2);
Quaternion quat_conjugate(const Quaternion* q1);
Quaternion quat_inverse(const Quaternion* q1);
void quat_normalize(Quaternion* q1);
float quat_length(const Quaternion* q1);
int quat_is_normalized(const Quaternion* q1);
Vector3 quat_rotate_point(Quaternion q, Vector3 point);
Vector3 quat_rotate_point_axis(Vector3 axis, float angle, Vector3 point);
Vector3 quat_to_euler(const Quaternion* q1);

#endif /*__QUATERNIONS_H__*/
