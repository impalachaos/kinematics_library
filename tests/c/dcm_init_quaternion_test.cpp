#include <gtest/gtest.h>
#include <kinematics-library/errors.h>
#include <kinematics-library/rotations.h>
#include <stdint.h>
#include <string>
#include <universal-constants/constants.h>

struct DCMInitQuaternionTestParams
{
    std::string      name;
    bool             input1Null;
    bool             input2Null;
    dcm_t            input1;
    quaternion_t     input2;
    dcm_t            want;
    precision_type_t near;
    int              wantError;
};

class InitQuaternionTest : public testing::TestWithParam<DCMInitQuaternionTestParams>
{};

TEST_P(InitQuaternionTest, Compute)
{
    DCMInitQuaternionTestParams params = GetParam();

    dcm_t        *input1 = &params.input1;
    quaternion_t *input2 = &params.input2;

    int error;

    if (params.input1Null)
    {
        input1 = NULL;
    }

    if (params.input2Null)
    {
        input2 = NULL;
    }

    error = dcm_init_quaternion(input1, input2);
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
    InitQuaternionTest,
    testing::Values(
        DCMInitQuaternionTestParams{
            "NULL input 1",                                // name
            true,                                          // input1Null
            false,                                         // input2Null
            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, // input1
            {0.0, 0.0, 0.0, 0.0},                          // input2
            {1.0, 2.0, 3.0, 0.0, 1.0, 2.0, 0.0, 0.0, 1.0}, // want
            0.0,                                           // near
            KL_ERROR_NULL_ARGUMENT                         // wantError
        },
        DCMInitQuaternionTestParams{
            "NULL input 2",                                // name
            false,                                         // input1Null
            true,                                          // input2Null
            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, // input1
            {0.0, 0.0, 0.0, 0.0},                          // input2
            {1.0, 2.0, 3.0, 0.0, 1.0, 2.0, 0.0, 0.0, 1.0}, // want
            0.0,                                           // near
            KL_ERROR_NULL_ARGUMENT                         // wantError
        },
        DCMInitQuaternionTestParams{
            "Sequence body xyx",                                                               // name
            false,                                                                             // input1Null
            false,                                                                             // input2Null
            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},                                     // input1
            {0.4109822630778004, -0.2307466279496697, 0.8123369342780543, 0.3434505471433091}, // input2,
            {-0.5556991462506127,
             -0.09258385044673502,
             -0.8262122545041296,
             -0.6571921829277988,
             0.6575954307136367,
             0.3683295863803792,
             0.5092120320209791,
             0.7476606717896851,
             -0.4262706022048224}, // want
            1.0E-14,               // near
            KL_NO_ERROR            // wantError
        }),
    [](const ::testing::TestParamInfo<DCMInitQuaternionTestParams> &info) -> std::string {
        std::string name = info.param.name;
        std::replace(name.begin(), name.end(), ' ', '_');
        return name;
    });
