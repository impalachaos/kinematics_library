#include <gtest/gtest.h>
#include <kinematics-library/errors.h>
#include <kinematics-library/rotations.h>
#include <stdint.h>
#include <string>
#include <universal-constants/constants.h>

struct KvTransposeDCMTestParams
{
    std::string      name;
    bool             input1Null;
    bool             resultNull;
    dcm_t            input1;
    bool             inplace;
    dcm_t            want;
    precision_type_t near;
    int              wantError;
};

class TransposeDCMTest : public testing::TestWithParam<KvTransposeDCMTestParams>
{};

TEST_P(TransposeDCMTest, Compute)
{
    KvTransposeDCMTestParams params = GetParam();

    dcm_t *input1 = &params.input1;

    if (params.input1Null)
    {
        input1 = NULL;
    }

    dcm_t result;

    dcm_t *resultPtr = &result;

    if (params.inplace)
    {
        resultPtr = &params.input1;
    }

    if (params.resultNull)
    {
        resultPtr = NULL;
    }

    int error = dcm_transpose(input1, resultPtr);
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
    TransposeDCMTest,
    testing::Values(
        KvTransposeDCMTestParams{
            "NULL input 1",                                // name
            true,                                          // input1Null
            false,                                         // resultNull
            {1.0, 0.0, 0.0, 2.0, 1.0, 0.0, 3.0, 2.0, 1.0}, // input1
            false,                                         // inplace
            {1.0, 2.0, 3.0, 0.0, 1.0, 2.0, 0.0, 0.0, 1.0}, // want
            0.0,                                           // near
            KL_ERROR_NULL_ARGUMENT                         // wantError
        },
        KvTransposeDCMTestParams{
            "NULL result",                                 // name
            false,                                         // input1Null
            true,                                          // resultNull
            {1.0, 0.0, 0.0, 2.0, 1.0, 0.0, 3.0, 2.0, 1.0}, // input1
            false,                                         // inplace
            {1.0, 2.0, 3.0, 0.0, 1.0, 2.0, 0.0, 0.0, 1.0}, // want
            0.0,                                           // near
            KL_ERROR_NULL_ARGUMENT                         // wantError
        },
        KvTransposeDCMTestParams{
            "Transpose",                                   // name
            false,                                         // input1Null
            false,                                         // resultNull
            {1.0, 0.0, 0.0, 2.0, 1.0, 0.0, 3.0, 2.0, 1.0}, // input1
            false,                                         // inplace
            {1.0, 2.0, 3.0, 0.0, 1.0, 2.0, 0.0, 0.0, 1.0}, // want
            0.0,                                           // near
            KL_NO_ERROR                                    // wantError
        },
        KvTransposeDCMTestParams{
            "Transpose in place",                          // name
            false,                                         // input1Null
            false,                                         // resultNull
            {1.0, 0.0, 0.0, 2.0, 1.0, 0.0, 3.0, 2.0, 1.0}, // input1
            true,                                          // inplace
            {1.0, 2.0, 3.0, 0.0, 1.0, 2.0, 0.0, 0.0, 1.0}, // want
            0.0,                                           // near
            KL_NO_ERROR                                    // wantError
        }),
    [](const ::testing::TestParamInfo<KvTransposeDCMTestParams> &info) -> std::string {
        std::string name = info.param.name;
        std::replace(name.begin(), name.end(), ' ', '_');
        return name;
    });
