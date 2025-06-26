#include <gtest/gtest.h>
#include <kinematics-library/errors.h>
#include <kinematics-library/rotations.h>
#include <stdint.h>
#include <string>
#include <universal-constants/constants.h>

struct DCMMulTestParams
{
    std::string      name;
    bool             input1Null;
    bool             input2Null;
    bool             resultNull;
    dcm_t            input1;
    dcm_t            input2;
    dcm_t            want;
    precision_type_t near;
    int              wantError;
};

class MulTest : public testing::TestWithParam<DCMMulTestParams>
{};

TEST_P(MulTest, Compute)
{
    DCMMulTestParams params = GetParam();

    dcm_t *input1 = &params.input1;
    dcm_t *input2 = &params.input2;

    if (params.input1Null)
    {
        input1 = NULL;
    }

    if (params.input2Null)
    {
        input2 = NULL;
    }

    dcm_t result;

    dcm_t *resultPtr = &result;

    if (params.resultNull)
    {
        resultPtr = NULL;
    }

    int error = dcm_mul(input1, input2, resultPtr);
    EXPECT_EQ(error, params.wantError);
    if (error == KL_NO_ERROR)
    {
        if (params.near == 0)
        {
            EXPECT_DOUBLE_EQ(params.want.e00, resultPtr->e00);
            EXPECT_DOUBLE_EQ(params.want.e01, resultPtr->e01);
            EXPECT_DOUBLE_EQ(params.want.e02, resultPtr->e02);
            EXPECT_DOUBLE_EQ(params.want.e10, resultPtr->e10);
            EXPECT_DOUBLE_EQ(params.want.e11, resultPtr->e11);
            EXPECT_DOUBLE_EQ(params.want.e12, resultPtr->e12);
            EXPECT_DOUBLE_EQ(params.want.e20, resultPtr->e20);
            EXPECT_DOUBLE_EQ(params.want.e21, resultPtr->e21);
            EXPECT_DOUBLE_EQ(params.want.e22, resultPtr->e22);
        }
        else
        {
            EXPECT_NEAR(params.want.e00, resultPtr->e00, params.near);
            EXPECT_NEAR(params.want.e01, resultPtr->e01, params.near);
            EXPECT_NEAR(params.want.e02, resultPtr->e02, params.near);
            EXPECT_NEAR(params.want.e10, resultPtr->e10, params.near);
            EXPECT_NEAR(params.want.e11, resultPtr->e11, params.near);
            EXPECT_NEAR(params.want.e12, resultPtr->e12, params.near);
            EXPECT_NEAR(params.want.e20, resultPtr->e20, params.near);
            EXPECT_NEAR(params.want.e21, resultPtr->e21, params.near);
            EXPECT_NEAR(params.want.e22, resultPtr->e22, params.near);
        }
    }
}

INSTANTIATE_TEST_SUITE_P(
    DCM,
    MulTest,
    testing::Values(
        DCMMulTestParams{
            "NULL input 1",                                // name
            true,                                          // input1Null
            false,                                         // input2Null
            false,                                         // resultNull
            {1.0, 0.0, 0.0, 2.0, 1.0, 0.0, 3.0, 2.0, 1.0}, // input1
            {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0}, // input2
            {1.0, 2.0, 3.0, 0.0, 1.0, 2.0, 0.0, 0.0, 1.0}, // want
            0.0,                                           // near
            KL_ERROR_NULL_ARGUMENT                         // wantError
        },
        DCMMulTestParams{
            "NULL input 2",                                // name
            false,                                         // input1Null
            true,                                          // input2Null
            false,                                         // resultNull
            {1.0, 0.0, 0.0, 2.0, 1.0, 0.0, 3.0, 2.0, 1.0}, // input1
            {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0}, // input2
            {1.0, 2.0, 3.0, 0.0, 1.0, 2.0, 0.0, 0.0, 1.0}, // want
            0.0,                                           // near
            KL_ERROR_NULL_ARGUMENT                         // wantError
        },
        DCMMulTestParams{
            "NULL result",                                 // name
            false,                                         // input1Null
            false,                                         // input2Null
            true,                                          // resultNull
            {1.0, 0.0, 0.0, 2.0, 1.0, 0.0, 3.0, 2.0, 1.0}, // input1
            {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0}, // input2
            {1.0, 2.0, 3.0, 0.0, 1.0, 2.0, 0.0, 0.0, 1.0}, // want
            0.0,                                           // near
            KL_ERROR_NULL_ARGUMENT                         // wantError
        },
        DCMMulTestParams{
            "Multiplication",                                          // name
            false,                                                     // input1Null
            false,                                                     // input2Null
            true,                                                      // resultNull
            {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0},             // input1
            {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0},             // input2
            {30.0, 36.0, 42.0, 66.0, 81.0, 96.0, 102.0, 126.0, 150.0}, // want
            0.0,                                                       // near
            KL_ERROR_NULL_ARGUMENT                                     // wantError
        }),
    [](const ::testing::TestParamInfo<DCMMulTestParams> &info) -> std::string {
        std::string name = info.param.name;
        std::replace(name.begin(), name.end(), ' ', '_');
        return name;
    });
