#include <stdlib.h>
#include <math.h>
#include <stdint.h>

#include "quaternion.h"
#include "vector3.h"


#define EPSILON 0.00001f

Quaternion quat_create(Vector3 axis, float scalar)
{
	Quaternion quat;
	quat.scalar = scalar;
	quat.v = axis;

	return quat;
}
Quaternion quat_create_from_axis_angle(Vector3 axis, float angle)
{
    Quaternion quat;
    angle = vec3_deg_to_rad(angle);
    quat.scalar = cosf(angle/2.0f);
    quat.v = vec3_multiply_scalar(axis, sinf(angle/2.0f));

    return quat;
}


Quaternion quat_create_identity()
{
	Quaternion quat;
	quat.scalar = 1.0f;
	quat.v = vec3_create(0,0,0);

	return quat;
}

Quaternion quat_multiply_quaternion(const Quaternion* q1, const Quaternion* q2)
{
    Quaternion ret;
    ret.scalar = q1->scalar*q2->scalar - vec3_dot_product(q1->v, q2->v);
    Vector3 q1q2_cross = vec3_cross_product(q1->v, q2->v);

    Vector3 tmp = vec3_multiply_scalar (q2->v, q1->scalar);
    q1q2_cross = vec3_add_vector(q1q2_cross, tmp);
    tmp = vec3_multiply_scalar(q1->v, q2->scalar);
    ret.v = vec3_add_vector(q1q2_cross, tmp);
    return ret;
}

Quaternion quat_multiply_scalar(const Quaternion* q1, float scalar)
{
    Quaternion q2;

    q2.scalar = scalar;
    q2.v = vec3_create(0,0,0);

    return quat_multiply_quaternion(q1, &q2);
}

Quaternion quat_add_quaternion(const Quaternion* q1, const Quaternion* q2)
{
    Quaternion ret;
    ret.scalar = q1->scalar + q2->scalar;
    ret.v = vec3_add_vector(q1->v, q2->v);
    return ret;
}

Quaternion quat_subtract_quaternion(const Quaternion* q1, const Quaternion* q2)
{
    Quaternion ret;
    ret.scalar = q1->scalar - q2->scalar;
    ret.v = vec3_subtract_vector(q1->v, q2->v);
    return ret;
}

Quaternion quat_conjugate(const Quaternion* q1)
{
    Quaternion ret;
    ret.scalar = q1->scalar;
    ret.v = vec3_multiply_scalar(q1->v, -1.0f);
    return ret;
}

Quaternion quat_inverse(const Quaternion* q1)
{
    float q_len = powf(quat_length(q1), 2.0f);

    Quaternion tmp = quat_conjugate(q1);

    return quat_multiply_scalar(&tmp, 1.0f/q_len);
}

void quat_normalize(Quaternion* q1)
{
    float q_len = quat_length(q1);

    q1->scalar /= q_len;
    q1->v = vec3_multiply_scalar(q1->v, 1.0f/q_len);
}

float quat_length(const Quaternion* q1)
{
    return sqrtf(q1->scalar*q1->scalar + 
		q1->v.data[0]*q1->v.data[0] + 
		q1->v.data[1]*q1->v.data[1] + 
		q1->v.data[2]*q1->v.data[2]);
}

int quat_is_normalized(const Quaternion* q1)
{
    float ret = q1->scalar*q1->scalar + 
		q1->v.data[0]*q1->v.data[0] + 
		q1->v.data[1]*q1->v.data[1] + 
		q1->v.data[2]*q1->v.data[2];
    return (ret + EPSILON >= 1.0f) && (ret - EPSILON <= 1.0f);
}


Vector3 quat_rotate_point(Quaternion q, Vector3 point)
{
    quat_normalize(&q);

    Quaternion p;
    p.scalar    = 0.0f;
    p.v = point;

    Quaternion q_inverse = quat_inverse(&q);
    Quaternion ret = quat_multiply_quaternion(&q, &p);
    ret = quat_multiply_quaternion(&ret, &q_inverse);

    return ret.v;
}

Vector3 quat_rotate_point_axis(Vector3 axis, float angle, Vector3 point)
{
    Quaternion q;
    q.scalar = cosf(angle/2.0f);
    q.v = vec3_multiply_scalar(axis, sinf(angle/2.0f));

    return quat_rotate_point(q, point);
}

int _is_zero(float f)
{
	return fabsf(f) <= EPSILON;
}

// by wiki
Vector3 quat_to_euler2(const Quaternion* q1)
{
	Vector3 out;

	float x, y, z, w;
	x = q1->v.data[0];
	y = q1->v.data[1];
	z = q1->v.data[2];
	w = q1->scalar;

	float sinr_cosp = 2.0f *(w*x + y*z);
	float cosr_cosp = 1.0f -2.0f*(x*x +y*y);
	out.data[0] = atan2f(sinr_cosp, cosr_cosp);

	float sinp = 2.0f *(w*y -z*x);
	if(fabsf(sinp) >= 1)
	{
		out.data[1] = copysign(M_PI_2, sinp);
	}else
	{
		out.data[1] = asinf(sinp);
	}

	float siny_cosp = 2.0f*(w*z+x*y);
	float cosy_cosp = 1.0f - 2.0f *(y*y+z*z);
	out.data[2] = atan2f(siny_cosp, cosy_cosp);

    for(int8_t i=0; i<3; i++)
    {
    	out.data[i] = vec3_rad_to_deg(out.data[i]);
    }

	return out;
}

Vector3 quat_to_euler(const Quaternion* q1)
{
	Vector3 out;

	const float* d = q1->v.data;
    float wp = q1->scalar;

	float xx = d[0] * d[0];
    float xy = d[0] * d[1];
    float xz = d[0] * d[2];
    float xw = d[0] * wp;
    float yy = d[1] * d[1];
    float yz = d[1] * d[2];
    float yw = d[1] * wp;
    float zz = d[2] * d[2];
    float zw = d[2] * wp;

    const float lengthSquared = xx + yy + zz + wp * wp;
    if(!_is_zero(lengthSquared - 1.0f) && !_is_zero(lengthSquared))
    {
        xx /= lengthSquared;
        xy /= lengthSquared; // same as (xp / length) * (yp / length)
        xz /= lengthSquared;
        xw /= lengthSquared;
        yy /= lengthSquared;
        yz /= lengthSquared;
        yw /= lengthSquared;
        zz /= lengthSquared;
        zw /= lengthSquared;
    }
    out.data[1] = asinf(-2.0f * (yz - xw));
    if(out.data[1] <  M_PI_2){
    	if(out.data[1] > -M_PI_2){
    		out.data[2] = atan2f(2.0f * (xz + yw), 1.0f - 2.0f * (xx + yy));
    		out.data[0] = atan2f(2.0f * (xy + zw), 1.0f - 2.0f * (xx + zz));
    	}else {
            // not a unique solution
    		out.data[0] = 0.0f;
    		out.data[2]= -atan2f(-2.0f * (xy - zw), 1.0f - 2.0f * (yy + zz));
        }
    }else {
        // not a unique solution
    	out.data[0] = 0.0f;
    	out.data[2] = atan2f(-2.0f * (xy - zw), 1.0f - 2.0f * (yy + zz));
    }

    for(int8_t i=0;i<3;i++)
    	out.data[i] = vec3_rad_to_deg(out.data[i]);

    return out;
}
