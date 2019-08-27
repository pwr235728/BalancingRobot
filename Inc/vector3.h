#ifndef __VECTOR3_H__
#define __VECTOR3_H__

typedef struct{
	float data[3];
} Vector3;

Vector3 vec3_create(float x, float y, float z);

Vector3 vec3_normalize(Vector3 vector);
Vector3 vec3_cross_product(Vector3 v1, Vector3 v2);
Vector3 vec3_add_vector(Vector3 v1, Vector3 v2);
Vector3 vec3_subtract_vector(Vector3 v1, Vector3 v2);
Vector3 vec3_multiply_scalar(Vector3 v1, float scalar);
Vector3 vec3_divide_scalar(Vector3 vector, float scalar);
float vec3_length(Vector3 vector);
float vec3_dot_product(Vector3 v1, Vector3 v2);
float vec3_angle(Vector3 v1, Vector3 v2);
float vec3_rad_to_deg(float radians);
float vec3_deg_to_rad(float degrees);

#endif /* __VECTOR3_H__ */