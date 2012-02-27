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
    result[0] = a[1]*b[2] - a[2]*b[1];
    result[1] = a[2]*b[0] - a[0]*b[2];
    result[2] = a[0]*b[1] - a[1]*b[0];
}

void ALG_transform(double *result, const double *v, const double *matrix)
{
    result[0] = matrix[0]*v[0] + matrix[1]*v[1] + matrix[2]*v[2];
    result[1] = matrix[3]*v[0] + matrix[4]*v[1] + matrix[5]*v[2];
    result[2] = matrix[6]*v[0] + matrix[7]*v[1] + matrix[8]*v[2];
}

void ALG_multiplyMatrix(double *result, const double *a, const double *b)
{
    int i,j;
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



