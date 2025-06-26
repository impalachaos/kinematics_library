#include <gtest/gtest.h>
#include <kinematics-library/errors.h>
#include <kinematics-library/transforms.h>
#include <stdint.h>
#include <string>
#include <universal-constants/constants.h>

struct LlaToNedTestParams
{
    std::string name;
    bool        input1Null;
    bool        input2Null;
    bool        input3Null;
    kvector_t   input1;
    kvector_t   input2;
    kvector_t   input3;
    kvector_t   want;
    kvector_t   near;
    int         wantError;
};

class LlaToNedTest : public testing::TestWithParam<LlaToNedTestParams>
{};

TEST_P(LlaToNedTest, Compute)
{
    LlaToNedTestParams params = GetParam();

    kvector_t *input1 = &params.input1;
    kvector_t *input2 = &params.input2;
    kvector_t *input3 = &params.input3;

    if (params.input1Null)
    {
        input1 = NULL;
    }

    if (params.input2Null)
    {
        input2 = NULL;
    }

    if (params.input3Null)
    {
        input3 = NULL;
    }

    int error = transform_lla_to_ned(input1, input2, input3);
    EXPECT_EQ(error, params.wantError);
    if (error == KL_NO_ERROR)
    {
        if (params.near.x == 0)
        {
            EXPECT_DOUBLE_EQ(params.want.x, params.input3.x);
        }
        else
        {
            EXPECT_NEAR(params.want.x, params.input3.x, params.near.x);
        }

        if (params.near.y == 0)
        {
            EXPECT_DOUBLE_EQ(params.want.y, params.input3.y);
        }
        else
        {
            EXPECT_NEAR(params.want.y, params.input3.y, params.near.y);
        }

        if (params.near.z == 0)
        {
            EXPECT_DOUBLE_EQ(params.want.z, params.input3.z);
        }
        else
        {
            EXPECT_NEAR(params.want.z, params.input3.z, params.near.z);
        }
    }
}

INSTANTIATE_TEST_SUITE_P(
    Transforms,
    LlaToNedTest,
    testing::Values(
        LlaToNedTestParams{
            "NULL input 1",        // name
            true,                  // input1Null
            false,                 // input2Null
            false,                 // input3Null
            {1.0, 1.0, 1.0},       // input1
            {0.0, 0.0, 0.0},       // input2
            {0.0, 0.0, 0.0},       // input3
            {0.0, 0.0, 0.0},       // want
            {0.0, 0.0, 0.0},       // near
            KL_ERROR_NULL_ARGUMENT // wantError
        },
        LlaToNedTestParams{
            "NULL input 2",        // name
            false,                 // input1Null
            true,                  // input2Null
            false,                 // input3Null
            {1.0, 1.0, 1.0},       // input1
            {0.0, 0.0, 0.0},       // input2
            {0.0, 0.0, 0.0},       // input3
            {0.0, 0.0, 0.0},       // want
            {0.0, 0.0, 0.0},       // near
            KL_ERROR_NULL_ARGUMENT // wantError
        },
        LlaToNedTestParams{
            "NULL input 3",        // name
            false,                 // input1Null
            false,                 // input2Null
            true,                  // input3Null
            {1.0, 1.0, 1.0},       // input1
            {0.0, 0.0, 0.0},       // input2
            {0.0, 0.0, 0.0},       // input3
            {0.0, 0.0, 0.0},       // want
            {0.0, 0.0, 0.0},       // near
            KL_ERROR_NULL_ARGUMENT // wantError
        },
        LlaToNedTestParams{
            "Transform",                                                 // name
            false,                                                       // input1Null
            false,                                                       // input2Null
            false,                                                       // input3Null
            {-0.707, 0.496, 2400.0},                                     // input1
            {-0.7, 0.5, 6000.0},                                         // input2
            {0.0, 0.0, 0.0},                                             // input3
            {-44576.5574572138, -19432.24609638221, 3785.6580070712043}, // want
            {1.0E-14, 1.0E-14, 1.0E-14},                                 // near
            KL_NO_ERROR                                                  // wantError
        }),
    [](const ::testing::TestParamInfo<LlaToNedTestParams> &info) -> std::string {
        std::string name = info.param.name;
        std::replace(name.begin(), name.end(), ' ', '_');
        return name;
    });
