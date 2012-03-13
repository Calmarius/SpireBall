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

typedef struct AlgebraDeterminantTestCase
{
    double matrix[9];
    double expected;
} AlgebraDeterminantTestCase;

typedef struct AlgebraSystem3TestCase
{
    double matrix[9];
    double rightSide[3];
    double expectedSolution[3];
    int expectedReturn;
} AlgebraSystem3TestCase;

typedef struct AlgebraTransposeTestCase
{
    double matrix[9];
    double expected[9];
} AlgebraTransposeTestCase;

typedef struct AlgebraRotationTestCase
{
    double axis[3];
    double angle;
    double expected[9];
} AlgebraRotationTestCase;

int isPointInTetraHedron(const double *point,const double *tetrahedron);

typedef struct CollisionPointInTetraHedronTestCase
{
    double tetrahedron[12];
    double point[3];
    int expected; // Zero or nonzero.
} CollisionPointInTetraHedronTestCase;

int getSupportVectorOfTriangle(const double *triangle, double *supportVector);

typedef struct CollisionTriangleSupportVectorTestCase
{
    double triangle[9];
    double expectedVector[3];
    int expectedReturn;
} CollisionTriangleSupportVectorTestCase;

int getSupportVectorOfLineSegment(const double *endPoints, double *supportVector);

typedef struct CollisionLineSegmentSupportTestCase
{
    double lineSegment[6];
    double expectedVector[3];
    int expectedReturn;
} CollisionLineSegmentSupportTestCase;

CollisionLineSegmentSupportTestCase lineSegmentSupportVectorTests[] =
{
    {
        {
            -1, 5, 0,
            -1, -5, 0
        },
        {
            1, 0, 0
        },
        1
    },
    {
        {
            -1, 5, 10,
            -1, -5, -10
        },
        {
            1, 0, 0
        },
        1
    },
    {
        {
            -5, -3, 0,
            3, 5, 0
        },
        {
            1, -1, 0
        },
        1
    },
    {
        {
            -1, 15, 0,
            -1, 5, 0
        },
        {
            1, 0, 0
        },
        0
    }
};

CollisionTriangleSupportVectorTestCase triangleSupportVectorTests[] =
{
    {
        {
            -1, -1, 5,
            -3, 1, -5,
            1, -3, -5
        },
        {
            1, 1, 0
        },
        1
    },
    {
        {
            -1, 1, 5,
            -3, -1, -5,
            1, 3, -5
        },
        {
            1, -1, 0
        },
        1
    },
    {
        {
            -1, 1, 5,
            -1, -1, -5,
            -1, 3, -5
        },
        {
            1, 0, 0
        },
        1
    },
    {
        {
            -1, 11, 5,
            -1, 9, -5,
            -1, 13, -5
        },
        {
            1, 0, 0
        },
        0
    }
};

CollisionPointInTetraHedronTestCase pointInTetraHedronTests[] =
{
    {
        {
            0, 5, 0,
            0, 0, -5,
            -5, 0, 5,
            5, 0, 5
        },
        {
            0, 1, 0
        },
        1
    },
    {
        {
            0, 5, 0,
            0, 0, 5,
            -5, 0, 5,
            5, 0, 5
        },
        {
            0, -0.1, 0
        },
        0
    },
    {
        {
            0, 5, 0,
            0, 0, 5,
            -5, 0, 5,
            5, 0, 5
        },
        {
            0, 6, 0
        },
        0
    },
    {
        {
            0, 5, 0,
            0, 0, 5,
            -5, 0, 5,
            5, 0, 5
        },
        {
            0, 0, 6
        },
        0
    }
};



AlgebraRotationTestCase rotationTests[] =
{
    {
        {
            0.267261241912424, 0.534522483824849, 0.801783725737273
        },
        0,
        {
            1, 0, 0,
            0, 1, 0,
            0, 0, 1
        }
    },
    {
        {
            1, 0, 0
        },
        0.523598775598299,
        {
            1, 0, 0,
            0, 0.866025403784439, -0.5,
            0, 0.5, 0.866025403784439
        }
    },
    {
        {
            0, 1, 0
        },
        0.523598775598299,
        {
            0.866025403784439, 0, 0.5,
            0, 1, 0,
            -0.5, 0, 0.866025403784439
        }
    },
    {
        {
            0, 1, 0
        },
        0.523598775598299,
        {
            0.866025403784439, 0, 0.5,
            0, 1, 0,
            -0.5, 0, 0.866025403784439
        }
    },
    {
        {
            0.168688971241019, 0.947253453891875, 0.2724975689278
        },
        1.30899693899575,
        {
            0.279910067392077, -0.14477823734084, 0.949046688084966,
            0.381646641515855, 0.923872641440063, 0.0283757540556784,
            -0.8809064622264, 0.354257821919346, 0.313855381372901
        }
    }
};

AlgebraTransposeTestCase transposeTests[] =
{
    {
        {
            1, 2, 3,
            4, 5, 6,
            7, 8, 9
        },
        {
            1, 4, 7,
            2, 5, 8,
            3, 6, 9
        }
    }
};

AlgebraSystem3TestCase system3Tests[] =
{
    {
        {
            5, 7, 9,
            9, 11, 15,
            34, 5, 10
        },
        {
            13, 20, 66
        },
        {
            351.0/154.0,
            368.0/77.0,
            -545.0/154.0
        },
        1
    },
    {
        {
            5, 17, 9,
            19, 11, 15,
            34, 5, 10
        },
        {
            93, 40, 66
        },
        {
            8551.0/3104.0,
            808.0/97.0,
            -21515.0/3104.0
        },
        1
    },
    {
        {
            1, 2, 3,
            4, 5, 6,
            7, 8, 9
        },
        {
            6, 7, 8
        },
        {
            0,
            0,
            0
        },
        0
    }
};

AlgebraDeterminantTestCase determinantTests[] =
{
    {
        {
            1, 5, 5,
            5, 4, 4,
            7, 5, 3
        },
        42
    },
    {
        {
            1, 5, 5,
            5,	1, 2,
            7, 1, 1
        },
        34
    },
    {
        {
            1, 0, 0,
            0,	1, 0,
            0, 0, 1
        },
        1
    },
    {
        {
            1, 4, 7,
            2, 5, 8,
            3, 6, 9
        },
        0
    }
};

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
    printf("%15g%15g%15g\n", v[0], v[1], v[2]);
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

void runDeterminantTests()
{
    int N = sizeof (determinantTests) / sizeof (determinantTests[0]);
    int i;

    for (i = 0; i < N; i++)
    {
        AlgebraDeterminantTestCase *current = &determinantTests[i];
        char passed = 1;
        double result;

        result = ALG_getDeterminant(current->matrix);
        // check
        if (fabs(result - current->expected) > 1e-9)
        {
            printf("Determinant test #%d FAILED!\n", i + 1);
            printf("expected: %g\n", current->expected);
            printf("Result: %g\n", result);
            passed = 0;
        }
        if (passed) printf("Determinant test #%d passed.\n", i + 1);
    }
}

void runSystem3Tests()
{
    int N = sizeof (system3Tests) / sizeof (system3Tests[0]);
    int i;

    for (i = 0; i < N; i++)
    {
        AlgebraSystem3TestCase *current = &system3Tests[i];
        char passed = 1;
        double retVal;
        double result[3];

        retVal = ALG_solveSystem3(result, current->matrix, current->rightSide);
        if (!!retVal == !!current->expectedReturn)
        {
            if (retVal)
            {
                int j;
                for (j = 0; j < 3; j++)
                {
                    if (fabs(result[j] - current->expectedSolution[j]) > 1e-9)
                    {
                        printf("System3 test #%d FAILED!\n", i + 1);
                        printf("Matrix:\n");
                        dumpMatrix(current->matrix);
                        printf("right side vector:\n");
                        dumpVector(current->rightSide);
                        printf("expected result:\n");
                        dumpVector(current->expectedSolution);
                        printf("Result:\n");
                        dumpVector(result);
                        passed = 0;
                        break;
                    }
                }
            }
        }
        else
        {
            printf("System3 test #%d FAILED!\n", i + 1);
            printf("Wrong return!\n");
            passed = 0;
        }
        // check
        if (passed) printf("System3 test #%d passed.\n", i + 1);
    }
}

void runTransposeTests()
{
    int N = sizeof (transposeTests) / sizeof (transposeTests[0]);
    int i;

    for (i = 0; i < N; i++)
    {
        AlgebraTransposeTestCase *current = &transposeTests[i];
        int j;
        char passed = 1;
        double before[9];

        memcpy(before, current->matrix, sizeof(before));

        ALG_transposeMatrix(current->matrix);

        // check
        for (j = 0; j < 9; j++)
        {
            if (fabs(current->matrix[j] - current->expected[j]) > 1e-9)
            {
                printf("Transpose test #%d FAILED!\n", i + 1);
                printf("before:\n");
                dumpMatrix(before);
                printf("expected:\n");
                dumpMatrix(current->expected);
                printf("Result:\n");
                dumpMatrix(current->matrix);
                passed = 0;
            }
        }
        if (passed) printf("Transpose test #%d passed.\n", i + 1);
    }
}

void runRotationTests()
{
    int N = sizeof (rotationTests) / sizeof (rotationTests[0]);
    int i;

    for (i = 0; i < N; i++)
    {
        AlgebraRotationTestCase *current = &rotationTests[i];
        int j;
        char passed = 1;
        double result[9];

        ALG_createRotationMatrix(result, current->axis, current->angle);
        // check
        for (j = 0; j < 9; j++)
        {
            if (fabs(result[j] - current->expected[j]) > 1e-9)
            {
                printf("Rotation test #%d FAILED!\n", i + 1);
                printf("Axis:\n");
                dumpVector(current->axis);
                printf("Angle: %g\n", current->angle);
                printf("Expected:\n");
                dumpMatrix(current->expected);
                printf("Result:\n");
                dumpMatrix(result);
                passed = 0;
            }
        }
        if (passed) printf("Rotation test #%d passed.\n", i + 1);
    }
}

void runTriangleSupportVectorTests()
{
    int N = sizeof (triangleSupportVectorTests) / sizeof (triangleSupportVectorTests[0]);
    int i;

    for (i = 0; i < N; i++)
    {
        CollisionTriangleSupportVectorTestCase *current = &triangleSupportVectorTests[i];
        int j;
        char passed = 1;
        double result[3];
        int retVal;

        retVal = getSupportVectorOfTriangle(current->triangle, result);

        if (!!retVal != !!current->expectedReturn)
        {
            passed = 0;
        }
        else
        {
            if (retVal)
            {
                for (j = 0; j < 3; j++)
                {
                    if (fabs(result[j] - current->expectedVector[j]) > 1e-9)
                    {
                        passed = 0;
                        break;
                    }
                }
            }
        }
        // check
        if (!passed)
        {
            printf("Triangle support vector test #%d FAILED!\n", i + 1);
            printf("Triangle:\n");
            dumpVector(&current->triangle[0]);
            dumpVector(&current->triangle[3]);
            dumpVector(&current->triangle[6]);
            printf("Expected vector: \n");
            dumpVector(current->expectedVector);
            printf("Expected return: %d \n", !!current->expectedReturn);
            printf("Result vector: \n");
            dumpVector(result);
            printf("Actual return: %d \n", !!retVal);
        }
        else
        {
            printf("Triangle support vector test #%d passed.\n", i + 1);
        }
    }
}

void runPointInTetrahedronTests()
{
    int N = sizeof (pointInTetraHedronTests) / sizeof (pointInTetraHedronTests[0]);
    int i;

    for (i = 0; i < N; i++)
    {
        CollisionPointInTetraHedronTestCase *current = &pointInTetraHedronTests[i];
        char passed = 1;
        int result;

        result = isPointInTetraHedron(current->point, current->tetrahedron);

        // check
        if (!!result != !!current->expected)
        {
            printf("Point in tetrahedron test #%d FAILED!\n", i + 1);
            printf("Point:\n");
            dumpVector(current->point);
            printf("Point:\n");
            dumpVector(&current->tetrahedron[0]);
            dumpVector(&current->tetrahedron[3]);
            dumpVector(&current->tetrahedron[6]);
            dumpVector(&current->tetrahedron[9]);
            printf("Expected: %d\n", current->expected);
            printf("Result: %d\n", result);
            passed = 0;
        }
        if (passed) printf("Point in tetrahedrontest #%d passed.\n", i + 1);
    }
}

void runLineSegmentSupportVectorTests()
{
    int N = sizeof (lineSegmentSupportVectorTests) / sizeof (lineSegmentSupportVectorTests[0]);
    int i;

    for (i = 0; i < N; i++)
    {
        CollisionLineSegmentSupportTestCase *current = &lineSegmentSupportVectorTests[i];
        int j;
        char passed = 1;
        double result[3];
        int retVal;

        retVal = getSupportVectorOfLineSegment(current->lineSegment, result);

        if (!!retVal != !!current->expectedReturn)
        {
            passed = 0;
        }
        else
        {
            if (retVal)
            {
                for (j = 0; j < 3; j++)
                {
                    if (fabs(result[j] - current->expectedVector[j]) > 1e-9)
                    {
                        passed = 0;
                        break;
                    }
                }
            }
        }
        // check
        if (!passed)
        {
            printf("Line segment support vector test #%d FAILED!\n", i + 1);
            printf("Line segment endpoints:\n");
            dumpVector(&current->lineSegment[0]);
            dumpVector(&current->lineSegment[3]);
            printf("Expected vector: \n");
            dumpVector(current->expectedVector);
            printf("Expected return: %d \n", !!current->expectedReturn);
            printf("Result vector: \n");
            dumpVector(result);
            printf("Actual return: %d \n", !!retVal);
        }
        else
        {
            printf("Line segment support vector test #%d passed.\n", i + 1);
        }
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
    printf("Running determinant tests:\n");
    runDeterminantTests();
    printf("Running sytem3 tests:\n");
    runSystem3Tests();
    printf("Running transpose tests:\n");
    runTransposeTests();
    printf("Running transpose tests:\n");
    runRotationTests();
    printf("Running point in tetrahedron tests:\n");
    runPointInTetrahedronTests();
    printf("Triangle support vector tests:\n");
    runTriangleSupportVectorTests();
    printf("Line segment support vector tests:\n");
    runLineSegmentSupportVectorTests();
    printf("Testing finished.\n");
    fgetc(stdin);
    return 0;
}
