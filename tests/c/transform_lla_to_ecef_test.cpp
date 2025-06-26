#include <gtest/gtest.h>
#include <kinematics-library/errors.h>
#include <kinematics-library/transforms.h>
#include <stdint.h>
#include <string>
#include <universal-constants/constants.h>

struct LlaToEcefTestParams
{
    std::string name;
    bool        input1Null;
    bool        input2Null;
    kvector_t   input1;
    kvector_t   input2;
    kvector_t   want;
    kvector_t   near;
    int         wantError;
};

class LlaToEcefTest : public testing::TestWithParam<LlaToEcefTestParams>
{};

TEST_P(LlaToEcefTest, Compute)
{
    LlaToEcefTestParams params = GetParam();

    kvector_t *input1 = &params.input1;
    kvector_t *input2 = &params.input2;

    if (params.input1Null)
    {
        input1 = NULL;
    }

    if (params.input2Null)
    {
        input2 = NULL;
    }

    int error = transform_lla_to_ecef(input1, input2);
    EXPECT_EQ(error, params.wantError);
    if (error == KL_NO_ERROR)
    {
        if (params.near.x == 0)
        {
            EXPECT_DOUBLE_EQ(params.want.x, params.input2.x);
        }
        else
        {
            EXPECT_NEAR(params.want.x, params.input2.x, params.near.x);
        }

        if (params.near.y == 0)
        {
            EXPECT_DOUBLE_EQ(params.want.y, params.input2.y);
        }
        else
        {
            EXPECT_NEAR(params.want.y, params.input2.y, params.near.y);
        }

        if (params.near.z == 0)
        {
            EXPECT_DOUBLE_EQ(params.want.z, params.input2.z);
        }
        else
        {
            EXPECT_NEAR(params.want.z, params.input2.z, params.near.z);
        }
    }
}

INSTANTIATE_TEST_SUITE_P(
    Transforms,
    LlaToEcefTest,
    testing::Values(
        LlaToEcefTestParams{
            "NULL input 1",        // name
            true,                  // input1Null
            false,                 // input2Null
            {1.0, 1.0, 1.0},       // input1
            {0.0, 0.0, 0.0},       // input2
            {0.0, 0.0, 0.0},       // want
            {0.0, 0.0, 0.0},       // near
            KL_ERROR_NULL_ARGUMENT // wantError
        },
        LlaToEcefTestParams{
            "NULL input 2",        // name
            false,                 // input1Null
            true,                  // input2Null
            {1.0, 1.0, 1.0},       // input1
            {0.0, 0.0, 0.0},       // input2
            {0.0, 0.0, 0.0},       // want
            {0.0, 0.0, 0.0},       // near
            KL_ERROR_NULL_ARGUMENT // wantError
        },
        LlaToEcefTestParams{
            "Transform",                                             // name
            false,                                                   // input1Null
            false,                                                   // input2Null
            {-0.8, 1.7, 4750.0},                                     // input1
            {0.0, 0.0, 0.0},                                         // input2
            {-573960.23482409, 4417543.57131158, -4556021.90733023}, // want
            {1.0E-8, 1.0E-8, 1.0E-8},                                // near
            KL_NO_ERROR                                              // wantError
        }),
    [](const ::testing::TestParamInfo<LlaToEcefTestParams> &info) -> std::string {
        std::string name = info.param.name;
        std::replace(name.begin(), name.end(), ' ', '_');
        return name;
    });
