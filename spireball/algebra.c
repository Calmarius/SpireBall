#include "algebra.h"

void ALG_translate(double *p, const double *v)
{
    p[0] += v[0];
    p[1] += v[1];
    p[2] += v[2];
}

void ALG_scale(double *v, double c)
{
    v[0] *= c;
    v[1] *= c;
    v[2] *= c;
}

double ALG_dotProduct(const double *a, const double *b)
{
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

void ALG_crossProduct(double *result, const double *a, const double *b)
{
    assert(result != a);
    assert(result != b);
    result[0] = a[1]*b[2] - a[2]*b[1];
    result[1] = a[2]*b[0] - a[0]*b[2];
    result[2] = a[0]*b[1] - a[1]*b[0];
}

void ALG_transform(double *result, const double *v, const double *matrix)
{
    assert(result != v);
    result[0] = matrix[0]*v[0] + matrix[1]*v[1] + matrix[2]*v[2];
    result[1] = matrix[3]*v[0] + matrix[4]*v[1] + matrix[5]*v[2];
    result[2] = matrix[6]*v[0] + matrix[7]*v[1] + matrix[8]*v[2];
}

void ALG_multiplyMatrix(double *result, const double *a, const double *b)
{
    int i,j;
    assert(result != a);
    assert(result != b);
    for (i = 0; i < 3; i++)
    {
        for (j = 0; j < 3; j++)
        {
            result[3*i + j] =
                a[3*i + 0]*b[j] +
                a[3*i + 1]*b[j + 3] +
                a[3*i + 2]*b[j + 6];
        }
    }
}

void ALG_translateMatrix(double *M, double *T)
{
    int i;
    for (i = 0; i < 9; i++)
    {
        M[i] += T[i];
    }
}

void ALG_scaleMatrix(double *M, double f)
{
    int i;
    for (i = 0; i < 9; i++)
    {
        M[i] *= f;
    }
}

double ALG_getVectorLength(const double *v)
{
    return sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

void ALG_normalizeVector(double *v)
{
    double length = ALG_getVectorLength(v);
    if (length > 0)
    {
        ALG_scale(v, 1.0/length);
    }
}

void ALG_createRotationMatrix(double *M, const double *axis, double angle)
{
    double cosAlpha = cos(angle);
    double omCosAlpha = 1 - cosAlpha;
    double sinAlpha = sin(angle);
    double x2 = axis[0]*axis[0];
    double y2 = axis[1]*axis[1];
    double z2 = axis[2]*axis[2];
    double xy = axis[0]*axis[1];
    double xz = axis[0]*axis[2];
    double yz = axis[1]*axis[2];
    // First row
    M[0] = x2*omCosAlpha + cosAlpha;
    M[1] = xy*omCosAlpha - axis[2]*sinAlpha;
    M[2] = xz*omCosAlpha + axis[1]*sinAlpha;
    // Second row
    M[3] = xy*omCosAlpha + axis[2]*sinAlpha;
    M[4] = y2*omCosAlpha + cosAlpha;
    M[5] = yz*omCosAlpha - axis[0]*sinAlpha;
    // Third row
    M[6] = xz*omCosAlpha - axis[1]*sinAlpha;
    M[7] = yz*omCosAlpha + axis[0]*sinAlpha;
    M[8] = z2*omCosAlpha + cosAlpha;
}

