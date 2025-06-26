#include <gtest/gtest.h>
#include <kinematics-library/errors.h>
#include <kinematics-library/rotations.h>
#include <stdint.h>
#include <string>
#include <universal-constants/constants.h>

struct QInitIdentityTestParams
{
    std::string      name;
    bool             input1Null;
    quaternion_t     input1;
    quaternion_t     want;
    precision_type_t near;
    int              wantError;
};

class InitIdentityTest : public testing::TestWithParam<QInitIdentityTestParams>
{};

TEST_P(InitIdentityTest, Compute)
{
    QInitIdentityTestParams params = GetParam();

    quaternion_t *input1 = &params.input1;

    int error;

    if (params.input1Null)
    {
        input1 = NULL;
    }

    error = quaternion_init_identity(input1);
    EXPECT_EQ(error, params.wantError);
    if (error == KL_NO_ERROR)
    {
        if (params.near == 0)
        {
            EXPECT_DOUBLE_EQ(params.want.a, params.input1.a);
            EXPECT_DOUBLE_EQ(params.want.b, params.input1.b);
            EXPECT_DOUBLE_EQ(params.want.c, params.input1.c);
            EXPECT_DOUBLE_EQ(params.want.d, params.input1.d);
        }
        else
        {
            EXPECT_NEAR(params.want.a, params.input1.a, params.near);
            EXPECT_NEAR(params.want.b, params.input1.b, params.near);
            EXPECT_NEAR(params.want.c, params.input1.c, params.near);
            EXPECT_NEAR(params.want.d, params.input1.d, params.near);
        }
    }
}

INSTANTIATE_TEST_SUITE_P(
    Quaternion,
    InitIdentityTest,
    testing::Values(
        QInitIdentityTestParams{
            "NULL input",          // name
            true,                  // input1Null
            {0.0, 0.0, 0.0, 0.0},  // input1
            {1.0, 0.0, 0.0, 0.0},  // want
            0.0,                   // near
            KL_ERROR_NULL_ARGUMENT // wantError
        },
        QInitIdentityTestParams{
            "Identity",           // name
            false,                // input1Null
            {0.0, 0.0, 0.0, 0.0}, // input1
            {1.0, 0.0, 0.0, 0.0}, // want
            0.0,                  // near
            KL_NO_ERROR           // wantError
        }),
    [](const ::testing::TestParamInfo<QInitIdentityTestParams> &info) -> std::string {
        std::string name = info.param.name;
        std::replace(name.begin(), name.end(), ' ', '_');
        return name;
    });
