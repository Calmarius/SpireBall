#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "../spireball/algebra.h"

/// ALGEBRA TESTCASES

typedef struct AlgebraMatrixMultiplicationTestCase
{
    double expected[9];
    double A[9];
    double B[9];

} AlgebraMatrixMultiplicationTestCase;

typedef struct AlgebraTransformTestCase
{
    double expected[3];
    double M[9];
    double v[3];
} AlgebraTransformTestCase;

typedef struct AlgebraCrossProductTestCase
{
    double expected[3];
    double a[3];
    double b[3];
} AlgebraCrossProductTestCase;

typedef struct AlgebraDotProductTestCase
{
    double expected;
    double a[3];
    double b[3];
} AlgebraDotProductTestCase;

typedef struct AlgebraTranslateTestCase
{
    double expected[3];
    double subject[3];
    double modificator[3];
} AlgebraTranslateTestCase;

AlgebraTranslateTestCase translateTests[] =
{
    {
        {
            22, 44, 66
        },
        {
            10, 21, 30
        },
        {
            12, 23, 36
        }
    }
};

const AlgebraDotProductTestCase dotProductTests[] =
{
    {
        0,
        {
            1, 0, 0
        },
        {
            0, 1, 0
        },
    },
    {
        265114,
        {
            123, 34, 2134
        },
        {
            12, 34, 123
        },
    }
};

const AlgebraCrossProductTestCase crossProductTests[] =
{
    {
        {
            -3, 6, -3,
        },
        {
            1, 2, 3
        },
        {
            4, 5, 6
        }
    },
    {
        {
            0, 0, 1
        },
        {
            1, 0, 0
        },
        {
            0, 1, 0
        }
    },
    {
        {
            0, 0, -1
        },
        {
            0, 1, 0
        },
        {
            1, 0, 0
        }
    }
};

const AlgebraTransformTestCase transfromTests[] =
{
    {
        {
            36, 81, 126
        },
        {
            1, 2, 3,
            4, 5, 6,
            7, 8, 9
        },
        {
            2, 5, 8
        }
    }
};

const AlgebraMatrixMultiplicationTestCase matrixMultiplicationTests[] =
{
    {
        {
            1, 2, 3,
            4, 5, 6,
            7, 8, 9
        },
        {
            1, 2, 3,
            4, 5, 6,
            7, 8, 9
        },
        {
            1, 0, 0,
            0, 1, 0,
            0, 0, 1,
        }
    },
    {
        {
            30, 36, 42,
            66, 81, 96,
            102, 126, 150
        },
        {
            1, 2, 3,
            4, 5, 6,
            7, 8, 9
        },
        {
            1, 2, 3,
            4, 5, 6,
            7, 8, 9
        }
    }
};

void dumpMatrix(const double *M)
{
    int i;
    for (i = 0; i < 3; i++)
    {
        printf("%10g%10g%10g\n", M[3*i], M[3*i + 1], M[3*i + 2]);
    }
}

void dumpVector(const double *v)
{
    printf("%10g%10g%10g\n", v[0], v[1], v[2]);
}

void runMatrixMultiplicationTests()
{
    int N = sizeof (matrixMultiplicationTests) / sizeof (matrixMultiplicationTests[0]);
    int i;

    for (i = 0; i < N; i++)
    {
        const AlgebraMatrixMultiplicationTestCase *current = &matrixMultiplicationTests[i];
        double result[9];
        int j;

        ALG_multiplyMatrix(result, current->A, current->B);
        // check
        for (j = 0; j < 9; j++)
        {
            if (fabs(result[j] - current->expected[j]) > 1e-9)
            {
                printf("Multiplcation test #%d FAILED!\n", i + 1);
                printf("A:\n");
                dumpMatrix(current->A);
                printf("B:\n");
                dumpMatrix(current->B);
                printf("expected:\n");
                dumpMatrix(current->expected);
                printf("Result:\n");
                dumpMatrix(result);
            }
        }
        printf("Multiplcation test #%d passed.\n", i + 1);
    }
}

void runTransformTests()
{
    int N = sizeof (transfromTests) / sizeof (transfromTests[0]);
    int i;

    for (i = 0; i < N; i++)
    {
        const AlgebraTransformTestCase *current = &transfromTests[i];
        double result[3];
        int j;
        char passed = 1;

        ALG_transform(result, current->v, current->M);
        // check
        for (j = 0; j < 3; j++)
        {
            if (fabs(result[j] - current->expected[j]) > 1e-9)
            {
                printf("Transform test #%d FAILED!\n", i + 1);
                printf("M:\n");
                dumpMatrix(current->M);
                printf("v:\n");
                dumpVector(current->v);
                printf("expected:\n");
                dumpVector(current->expected);
                printf("Result:\n");
                dumpVector(result);
                passed = 0;
            }
        }
        if (passed) printf("Transform test #%d passed.\n", i + 1);
    }
}

void runCrossProductTests()
{
    int N = sizeof (crossProductTests) / sizeof (crossProductTests[0]);
    int i;

    for (i = 0; i < N; i++)
    {
        const AlgebraCrossProductTestCase *current = &crossProductTests[i];
        double result[3];
        int j;
        char passed = 1;

        ALG_crossProduct(result, current->a, current->b);
        // check
        for (j = 0; j < 3; j++)
        {
            if (fabs(result[j] - current->expected[j]) > 1e-9)
            {
                printf("Cross product test #%d FAILED!\n", i + 1);
                printf("a:\n");
                dumpVector(current->a);
                printf("b:\n");
                dumpVector(current->b);
                printf("expected:\n");
                dumpVector(current->expected);
                printf("Result:\n");
                dumpVector(result);
                passed = 0;
            }
        }
        if (passed) printf("Cross product test #%d passed.\n", i + 1);
    }
}

void runDotProductTests()
{
    int N = sizeof (dotProductTests) / sizeof (dotProductTests[0]);
    int i;

    for (i = 0; i < N; i++)
    {
        const AlgebraDotProductTestCase *current = &dotProductTests[i];
        double result;
        char passed = 1;

        result = ALG_dotProduct(current->a, current->b);
        // check
        if (fabs(result- current->expected) > 1e-9)
        {
            printf("Dot product test #%d FAILED!\n", i + 1);
            printf("a:\n");
            dumpVector(current->a);
            printf("b:\n");
            dumpVector(current->b);
            printf("expected: %g\n", current->expected);
            printf("Result: %g\n", result);
            passed = 0;
        }
        if (passed) printf("Dot product test #%d passed.\n", i + 1);
    }
}

void runTranslateTests()
{
    int N = sizeof (translateTests) / sizeof (translateTests[0]);
    int i;

    for (i = 0; i < N; i++)
    {
        AlgebraTranslateTestCase *current = &translateTests[i];
        int j;
        char passed = 1;
        double before[3];

        memcpy(before, current->subject, sizeof (before));

        ALG_translate(current->subject, current->modificator);
        // check
        for (j = 0; j < 3; j++)
        {
            if (fabs(current->subject[j] - current->expected[j]) > 1e-9)
            {
                printf("Translate test #%d FAILED!\n", i + 1);
                printf("before:\n");
                dumpVector(before);
                printf("modificator:\n");
                dumpVector(current->modificator);
                printf("expected:\n");
                dumpVector(current->expected);
                printf("Result:\n");
                dumpVector(current->subject);
                passed = 0;
            }
        }
        if (passed) printf("Translate test #%d passed.\n", i + 1);
    }
}
//
int main()
{
    printf("Running matrix multiplication tests:\n");
    runMatrixMultiplicationTests();
    printf("Running transform tests:\n");
    runTransformTests();
    printf("Running cross product tests:\n");
    runCrossProductTests();
    printf("Running dot product tests:\n");
    runDotProductTests();
    printf("Running translate tests:\n");
    runTranslateTests();
    printf("Testing finished.\n");
    fgetc(stdin);
    return 0;
}
