#include <gtest/gtest.h>
#include <kinematics-library/errors.h>
#include <kinematics-library/rotations.h>
#include <stdint.h>
#include <string>
#include <universal-constants/constants.h>

struct QMulTestParams
{
    std::string      name;
    bool             input1Null;
    bool             input2Null;
    bool             resultNull;
    quaternion_t     input1;
    quaternion_t     input2;
    quaternion_t     want;
    precision_type_t near;
    int              wantError;
};

class MulTest : public testing::TestWithParam<QMulTestParams>
{};

TEST_P(MulTest, Compute)
{
    QMulTestParams params = GetParam();

    quaternion_t *input1 = &params.input1;
    quaternion_t *input2 = &params.input2;

    if (params.input1Null)
    {
        input1 = NULL;
    }

    if (params.input2Null)
    {
        input1 = NULL;
    }

    int error;

    quaternion_t result;

    quaternion_t *resultPtr = &result;

    if (params.resultNull)
    {
        resultPtr = NULL;
    }

    error = quaternion_mul(input1, input2, resultPtr);
    EXPECT_EQ(error, params.wantError);
    if (error == KL_NO_ERROR)
    {
        if (params.near == 0)
        {
            EXPECT_DOUBLE_EQ(params.want.a, resultPtr->a);
            EXPECT_DOUBLE_EQ(params.want.b, resultPtr->b);
            EXPECT_DOUBLE_EQ(params.want.c, resultPtr->c);
            EXPECT_DOUBLE_EQ(params.want.d, resultPtr->d);
        }
        else
        {
            EXPECT_NEAR(params.want.a, resultPtr->a, params.near);
            EXPECT_NEAR(params.want.b, resultPtr->b, params.near);
            EXPECT_NEAR(params.want.c, resultPtr->c, params.near);
            EXPECT_NEAR(params.want.d, resultPtr->d, params.near);
        }
    }
}

INSTANTIATE_TEST_SUITE_P(
    Quaternion,
    MulTest,
    testing::Values(
        QMulTestParams{
            "Input 1 null",        // name
            true,                  // input1Null
            false,                 // input2Null
            false,                 // resultNull
            {1.0, 2.0, 3.0, 4.0},  // input1
            {1.0, 2.0, 3.0, 4.0},  // input2
            {0.0, 0.0, 0.0, 0.0},  // want
            0.0,                   // near
            KL_ERROR_NULL_ARGUMENT // wantError
        },
        QMulTestParams{
            "Input 2 null",        // name
            false,                 // input1Null
            true,                  // input2Null
            false,                 // resultNull
            {1.0, 2.0, 3.0, 4.0},  // input1
            {1.0, 2.0, 3.0, 4.0},  // input2
            {0.0, 0.0, 0.0, 0.0},  // want
            0.0,                   // near
            KL_ERROR_NULL_ARGUMENT // wantError
        },
        QMulTestParams{
            "Result null",         // name
            false,                 // input1Null
            false,                 // input2Null
            true,                  // resultNull
            {1.0, 2.0, 3.0, 4.0},  // input1
            {1.0, 2.0, 3.0, 4.0},  // input2
            {0.0, 0.0, 0.0, 0.0},  // want
            0.0,                   // near
            KL_ERROR_NULL_ARGUMENT // wantError
        },
        QMulTestParams{
            "Multiplication",          // name
            false,                     // input1Null
            false,                     // input2Null
            false,                     // resultNull
            {1.0, 2.0, 3.0, 4.0},      // input1
            {5.0, 6.0, 7.0, 8.0},      // input2
            {-60.0, 12.0, 30.0, 24.0}, // want
            0.0,                       // near
            KL_NO_ERROR                // wantError
        }),
    [](const ::testing::TestParamInfo<QMulTestParams> &info) -> std::string {
        std::string name = info.param.name;
        std::replace(name.begin(), name.end(), ' ', '_');
        return name;
    });
