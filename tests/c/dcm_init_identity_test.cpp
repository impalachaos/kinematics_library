#include <gtest/gtest.h>
#include <kinematics-library/errors.h>
#include <kinematics-library/rotations.h>
#include <stdint.h>
#include <string>
#include <universal-constants/constants.h>

struct DCMInitIdentityTestParams
{
    std::string      name;
    bool             input1Null;
    dcm_t            input1;
    dcm_t            want;
    precision_type_t near;
    int              wantError;
};

class InitIdentityTest : public testing::TestWithParam<DCMInitIdentityTestParams>
{};

TEST_P(InitIdentityTest, Compute)
{
    DCMInitIdentityTestParams params = GetParam();

    dcm_t *input1 = &params.input1;

    int error;

    if (params.input1Null)
    {
        input1 = NULL;
    }

    error = dcm_init_identity(input1);
    EXPECT_EQ(error, params.wantError);
    if (error == KL_NO_ERROR)
    {
        if (params.near == 0)
        {
            EXPECT_DOUBLE_EQ(params.want.e00, params.input1.e00);
            EXPECT_DOUBLE_EQ(params.want.e01, params.input1.e01);
            EXPECT_DOUBLE_EQ(params.want.e02, params.input1.e02);
            EXPECT_DOUBLE_EQ(params.want.e10, params.input1.e10);
            EXPECT_DOUBLE_EQ(params.want.e11, params.input1.e11);
            EXPECT_DOUBLE_EQ(params.want.e12, params.input1.e12);
            EXPECT_DOUBLE_EQ(params.want.e20, params.input1.e20);
            EXPECT_DOUBLE_EQ(params.want.e21, params.input1.e21);
            EXPECT_DOUBLE_EQ(params.want.e22, params.input1.e22);
        }
        else
        {
            EXPECT_NEAR(params.want.e00, params.input1.e00, params.near);
            EXPECT_NEAR(params.want.e01, params.input1.e01, params.near);
            EXPECT_NEAR(params.want.e02, params.input1.e02, params.near);
            EXPECT_NEAR(params.want.e10, params.input1.e10, params.near);
            EXPECT_NEAR(params.want.e11, params.input1.e11, params.near);
            EXPECT_NEAR(params.want.e12, params.input1.e12, params.near);
            EXPECT_NEAR(params.want.e20, params.input1.e20, params.near);
            EXPECT_NEAR(params.want.e21, params.input1.e21, params.near);
            EXPECT_NEAR(params.want.e22, params.input1.e22, params.near);
        }
    }
}

INSTANTIATE_TEST_SUITE_P(
    DCM,
    InitIdentityTest,
    testing::Values(
        DCMInitIdentityTestParams{
            "NULL input",                                  // name
            true,                                          // input1Null
            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, // input1
            {1.0, 2.0, 3.0, 0.0, 1.0, 2.0, 0.0, 0.0, 1.0}, // want
            0.0,                                           // near
            KL_ERROR_NULL_ARGUMENT                         // wantError
        },
        DCMInitIdentityTestParams{
            "Initialize",                                  // name
            false,                                         // input1Null
            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, // input1
            {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}, // want
            0.0,                                           // near
            KL_NO_ERROR                                    // wantError
        }),
    [](const ::testing::TestParamInfo<DCMInitIdentityTestParams> &info) -> std::string {
        std::string name = info.param.name;
        std::replace(name.begin(), name.end(), ' ', '_');
        return name;
    });
