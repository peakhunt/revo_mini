#ifndef __MATH_HELPER_DEF_H__
#define __MATH_HELPER_DEF_H__

#include "app_common.h"
#include <math.h>

#define M_PIf       3.14159265358979323846f
#define M_LN2f      0.69314718055994530942f
#define M_Ef        2.71828182845904523536f
    
#define RAD         (M_PIf / 180.0f)

#define F_EPSILON       0.000000001f

typedef struct
{
  double      x;
  double      y;
  double      z;
} VectorDouble;

typedef struct
{
  int16_t     x;
  int16_t     y;
  int16_t     z;
} VectorInt16;

typedef struct
{
  float       x;
  float       y;
  float       z;
} VectorFloat;

static inline float
invSqrt(float x)
{
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;

  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  y = y * (1.5f - (halfx * y * y));
  return y;

}

static inline bool
float_zero(float x)
{
  if(fabs(x) < F_EPSILON)
  {
    return true;
  }
  return false;
}

#define sq(x)   ((x)*(x))

static inline float
vector_norm_squared(const float v[3])
{
    return sq(v[0]) + sq(v[1]) + sq(v[2]);
}

static inline void
vector_normalize(const float v[3], float result[3])
{
  float length = sqrtf(vector_norm_squared(v));
  
  if(length != 0)
  {
    result[0] = v[0] / length;
    result[1] = v[1] / length;
    result[2] = v[2] / length;
  }
  else
  {
    result[0] = 0.0f;
    result[1] = 0.0f;
    result[2] = 0.0f;
  }
}

////////////////////////////////////////////////////////////////////////////////
//
// quaternion related
//
////////////////////////////////////////////////////////////////////////////////
static inline void
quaternion_conjugate(const float q[4], float conj[4])
{
  conj[0] =   q[0];
  conj[1] =  -q[1];
  conj[2] =  -q[2];
  conj[3] =  -q[3];
}

static inline void
quaternion_multiply(const float a[4], const float b[4], float result[4])
{
  float r[4];

  r[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
  r[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
  r[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
  r[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];

  result[0] = r[0];
  result[1] = r[1];
  result[2] = r[2];
  result[3] = r[3];
}

static inline void
quaternion_rotate_invert(const float q[4], const float org[3], float result[3])
{
  float     vq[4];
  float     q_conj[4];

  vq[0] = 0;
  vq[1] = org[0];
  vq[2] = org[1];
  vq[3] = org[2];

  quaternion_conjugate(q, q_conj);
  quaternion_multiply(q, vq, vq);
  quaternion_multiply(vq, q_conj, vq);

  result[0] = vq[1];
  result[1] = vq[2];
  result[2] = vq[3];
}

static inline float
lerp(float x, float x0, float x1, float y0, float y1)
{
  float a = (y1 - y0) / (x1 - x0);
  float b = -a*x0 + y0;
  float y = a * x + b;

  return y;
}

static inline void
clamp(float* x, float min, float max)
{
  if(*x < min)
  {
    *x = min;
  }

  if(*x > max)
  {
    *x = max;
  }
}

#endif //!__MATH_HELPER_DEF_H__
