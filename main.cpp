#include "NavDialog.h"
#include <QtWidgets/QApplication>

#include <windows.h>


#define MAX_CIRCLE_ANGLE      512
#define HALF_MAX_CIRCLE_ANGLE (MAX_CIRCLE_ANGLE/2)
#define QUARTER_MAX_CIRCLE_ANGLE (MAX_CIRCLE_ANGLE/4)
#define MASK_MAX_CIRCLE_ANGLE (MAX_CIRCLE_ANGLE - 1)
#define PI 3.14159265358979323846f

float fast_cossin_table[MAX_CIRCLE_ANGLE];           // Declare table of fast cosinus and sinus

                                                     // Copied from NVidia web site

inline float fastcos(float n)
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

inline float fastsin(float n)
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

const long iMaxTests = 100000000;

void xmain()
{
    long i;
    float s, c;
    float sc, scr = 0;
    unsigned long dwTickStart, dwTickEnd, dwDuration;

    // Build cossin table
    for (i = 0 ; i < MAX_CIRCLE_ANGLE ; i++)
    {
        fast_cossin_table[i] = (float)sin((double)i * PI / HALF_MAX_CIRCLE_ANGLE);
    }

    double d;
    dwTickStart = GetTickCount();
    for (i = - (iMaxTests/2) ; i < iMaxTests/2 ; i++)
    {
        d = (double)i;
        s = (float)sin(d);
        c = (float)cos(d);

        // This exist only to force optimiser to not delete code
        sc = s * c;
        if (sc > scr)
        {
            scr = sc;
        }
    }
    dwTickEnd = GetTickCount();
    dwDuration = dwTickEnd - dwTickStart;
    printf("%d sin and cos computed in %d ticks with standard math funcs\n", iMaxTests, dwDuration);

    float f;
    dwTickStart = GetTickCount();
    for (i = - (iMaxTests/2) ; i < iMaxTests/2 ; i++)
    {
        f = (float)i;
        s = fastsin(f);
        c = fastcos(f);

        // This exist only to force optimiser to not delete code
        sc = s * c;
        if (sc > scr)
        {
            scr = sc;
        }
    }
    dwTickEnd = GetTickCount();
    dwDuration = dwTickEnd - dwTickStart;
    printf("%d sin and cos computed in %d ticks with fast[cos/sin]\n", iMaxTests, dwDuration);

}



int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    NavDialog w;
    w.show();
    return a.exec();
}
