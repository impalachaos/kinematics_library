#include <gtest/gtest.h>
#include <kinematics-library/errors.h>
#include <kinematics-library/rotations.h>
#include <stdint.h>
#include <string>
#include <universal-constants/constants.h>

struct QInverseTestParams
{
    std::string      name;
    bool             input1Null;
    bool             resultNull;
    quaternion_t     input1;
    quaternion_t     want;
    precision_type_t near;
    int              wantError;
};

class InverseTest : public testing::TestWithParam<QInverseTestParams>
{};

TEST_P(InverseTest, Compute)
{
    QInverseTestParams params = GetParam();

    quaternion_t *input1 = &params.input1;

    if (params.input1Null)
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

    error = quaternion_inverse(input1, resultPtr);
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
    InverseTest,
    testing::Values(
        QInverseTestParams{
            "Input 1 null",        // name
            true,                  // input1Null
            false,                 // resultNull
            {1.0, 2.0, 3.0, 4.0},  // input1
            {0.0, 0.0, 0.0, 0.0},  // want
            0.0,                   // near
            KL_ERROR_NULL_ARGUMENT // wantError
        },
        QInverseTestParams{
            "Result null",         // name
            false,                 // input1Null
            true,                  // resultNull
            {1.0, 2.0, 3.0, 4.0},  // input1
            {0.0, 0.0, 0.0, 0.0},  // want
            0.0,                   // near
            KL_ERROR_NULL_ARGUMENT // wantError
        },
        QInverseTestParams{
            "Calcuate inverse",                                                      // name
            false,                                                                   // input1Null
            false,                                                                   // resultNull
            {1.0, 2.0, 3.0, 4.0},                                                    // input1
            {0.03333333333333333, -0.06666666666666667, -0.1, -0.13333333333333333}, // want
            0.0,                                                                     // near
            KL_NO_ERROR                                                              // wantError
        }),
    [](const ::testing::TestParamInfo<QInverseTestParams> &info) -> std::string {
        std::string name = info.param.name;
        std::replace(name.begin(), name.end(), ' ', '_');
        return name;
    });
