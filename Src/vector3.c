#include <stdio.h>
#include <math.h>

#include "vector3.h"

#define EPSILON 0.00001f
#define VEC3_PI_FLOAT 3.14159265f

Vector3 vec3_create(float x, float y, float z)
{
    Vector3 ret;
    ret.data[0] = x;
    ret.data[1] = y;
    ret.data[2] = z;
    return ret;
}

Vector3 vec3_normalize(Vector3 vector)
{
	float len = vec3_length(vector);
    if (len >= EPSILON)
        return vec3_create(
			vector.data[0]/len, 
			vector.data[1]/len, 
			vector.data[2]/len);
    return vector;
}

Vector3 vec3_cross_product(Vector3 v1, Vector3 v2)
{
    Vector3 product = vec3_create(
		(v1.data[1]*v2.data[2] - v1.data[2]*v2.data[1]),
		(v1.data[2]*v2.data[0] - v1.data[0]*v2.data[2]),
		(v1.data[0]*v2.data[1] - v1.data[1]*v2.data[0]));
    return product;
}

Vector3 vec3_add_vector(Vector3 v1, Vector3 v2)
{
    Vector3 ret;
    ret.data[0] = v1.data[0] + v2.data[0];
    ret.data[1] = v1.data[1] + v2.data[1];
    ret.data[2] = v1.data[2] + v2.data[2];
    return ret;
}

Vector3 vec3_subtract_vector(Vector3 v1, Vector3 v2)
{
    Vector3 ret;
    ret.data[0] = v1.data[0] - v2.data[0];
    ret.data[1] = v1.data[1] - v2.data[1];
    ret.data[2] = v1.data[2] - v2.data[2];
    return ret;
}

Vector3 vec3_multiply_scalar(Vector3 v1, float scalar)
{
    Vector3 ret;
    ret.data[0] = v1.data[0] * scalar;
    ret.data[1] = v1.data[1] * scalar;
    ret.data[2] = v1.data[2] * scalar;
    return ret;
}

Vector3 Vec3_DivideScalar(Vector3 vector, float scalar)
{
    return vec3_multiply_scalar(vector, 1.0f/scalar);
}

float vec3_length(Vector3 vector)
{
  return sqrt((vector.data[0]*vector.data[0])+
              (vector.data[1]*vector.data[1])+
              (vector.data[2]*vector.data[2]));
}

float vec3_dot_product(Vector3 v1, Vector3 v2)
{
    return v1.data[0]*v2.data[0] + 
		v1.data[1]*v2.data[1] + 
		v1.data[2]*v2.data[2];
}

float  vec3_angle(Vector3 v1, Vector3 v2)
{
    return vec3_rad_to_deg(acosf(
		vec3_dot_product(v1, v2) / 
		(vec3_length(v1)*vec3_length(v2))));
}

float vec3_deg_to_rad(float degrees)
{
    return degrees*VEC3_PI_FLOAT/180.0f;
}

float vec3_rad_to_deg(float radians)
{
    return radians*180.0f/VEC3_PI_FLOAT;
}