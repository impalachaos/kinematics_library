#include <gtest/gtest.h>
#include <kinematics-library/errors.h>
#include <kinematics-library/transforms.h>
#include <stdint.h>
#include <string>
#include <universal-constants/constants.h>

struct EcefToNedTestParams
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

class EcefToNedTest : public testing::TestWithParam<EcefToNedTestParams>
{};

TEST_P(EcefToNedTest, Compute)
{
    EcefToNedTestParams params = GetParam();

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

    int error = transform_ecef_to_ned(input1, input2, input3);
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
    EcefToNedTest,
    testing::Values(
        EcefToNedTestParams{
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
        EcefToNedTestParams{
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
        EcefToNedTestParams{
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
        EcefToNedTestParams{
            "Transform",                                             // name
            false,                                                   // input1Null
            false,                                                   // input2Null
            false,                                                   // input3Null
            {-1465000.0, 2880000.0, 5483500.0},                      // input1
            {2.1, -1.1, 1250.0},                                     // input2
            {0.0, 0.0, 0.0},                                         // input3
            {2220.90854391709, 738.047215659648, -1205.58533167388}, // want
            {1.0E-8, 1.0E-8, 1.0E-8},                                // near
            KL_NO_ERROR                                              // wantError
        }),
    [](const ::testing::TestParamInfo<EcefToNedTestParams> &info) -> std::string {
        std::string name = info.param.name;
        std::replace(name.begin(), name.end(), ' ', '_');
        return name;
    });
