#pragma once
//#define USE_MY_TRIG 

#ifndef USE_MY_TRIG
#include <cmath>
#endif

namespace mtrig 
{

#ifdef USE_MY_TRIG
// from http://http.developer.nvidia.com/Cg/atan2.html
float atan2(float y, float x)
{
    float t0, t1, t2, t3, t4;

    t3 = iabs(x);
    t1 = iabs(y);
    t0 = imax(t3, t1);
    t1 = imin(t3, t1);
    t3 = float(1) / t0;
    t3 = t1 * t3;

    t4 = t3 * t3;
    t0 =         - float(0.013480470);
    t0 = t0 * t4 + float(0.057477314);
    t0 = t0 * t4 - float(0.121239071);
    t0 = t0 * t4 + float(0.195635925);
    t0 = t0 * t4 - float(0.332994597);
    t0 = t0 * t4 + float(0.999995630);
    t3 = t0 * t3;

    t3 = (abs(y) > abs(x)) ? float(1.570796327) - t3 : t3;
    t3 = (x < 0) ?  float(3.141592654) - t3 : t3;
    t3 = (y < 0) ? -t3 : t3;

    return t3;
}

float asin(float x) {
    float negate = float(x < 0);
    x = iabs(x);
    float ret = -0.0187293;
    ret *= x;
    ret += 0.0742610;
    ret *= x;
    ret -= 0.2121144;
    ret *= x;
    ret += 1.5707288;
    ret = 3.14159265358979*0.5 - sqrt(1.0 - x)*ret;
    return ret - 2 * negate * ret;
}


// from http://www.flipcode.com/archives/Fast_Trigonometry_Functions_Using_Lookup_Tables.shtml
#define MAX_CIRCLE_ANGLE      512
#define HALF_MAX_CIRCLE_ANGLE (MAX_CIRCLE_ANGLE/2)
#define QUARTER_MAX_CIRCLE_ANGLE (MAX_CIRCLE_ANGLE/4)
#define MASK_MAX_CIRCLE_ANGLE (MAX_CIRCLE_ANGLE - 1)
#define PI 3.14159265358979323846f

static bool g_wasInited = false;
static float fast_cossin_table[MAX_CIRCLE_ANGLE];

void build_table() {
    // Build cossin table
    for (int i = 0 ; i < MAX_CIRCLE_ANGLE ; i++)
    {
        fast_cossin_table[i] = (float)sin((double)i * PI / HALF_MAX_CIRCLE_ANGLE);
    }
}

inline float cos(float n)
{
    float f = n * HALF_MAX_CIRCLE_ANGLE / PI;
    int i = (int)f;
    if (i < 0)
    {
        return fast_cossin_table[((-i) + QUARTER_MAX_CIRCLE_ANGLE) & MASK_MAX_CIRCLE_ANGLE];
    }
    else
    {
        return fast_cossin_table[(i + QUARTER_MAX_CIRCLE_ANGLE) & MASK_MAX_CIRCLE_ANGLE];
    }
}

inline float sin(float n)
{
    float f = n * HALF_MAX_CIRCLE_ANGLE / PI;
    int i = (int)f;
    if (i < 0)
    {
        return fast_cossin_table[(-((-i) & MASK_MAX_CIRCLE_ANGLE)) + MAX_CIRCLE_ANGLE];
    }
    else
    {
        return fast_cossin_table[i & MASK_MAX_CIRCLE_ANGLE];
    }
}
#else

using std::atan2;
using std::asin;
using std::cos;
using std::sin;

#endif

}