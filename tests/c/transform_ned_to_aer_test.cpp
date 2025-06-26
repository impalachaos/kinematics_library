#include <gtest/gtest.h>
#include <kinematics-library/errors.h>
#include <kinematics-library/transforms.h>
#include <stdint.h>
#include <string>
#include <universal-constants/constants.h>

struct NedToAerTestParams
{
    std::string      name;
    bool             input1Null;
    bool             input2Null;
    kvector_t        input1;
    kvector_t        input2;
    kvector_t        want;
    precision_type_t near;
    int              wantError;
};

class NedToAerTest : public testing::TestWithParam<NedToAerTestParams>
{};

TEST_P(NedToAerTest, Compute)
{
    NedToAerTestParams params = GetParam();

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

    int error = transform_ned_to_aer(input1, input2);
    EXPECT_EQ(error, params.wantError);
    if (error == KL_NO_ERROR)
    {
        if (params.near == 0)
        {
            EXPECT_DOUBLE_EQ(params.want.x, params.input2.x);
            EXPECT_DOUBLE_EQ(params.want.y, params.input2.y);
            EXPECT_DOUBLE_EQ(params.want.z, params.input2.z);
        }
        else
        {
            EXPECT_NEAR(params.want.x, params.input2.x, params.near);
            EXPECT_NEAR(params.want.y, params.input2.y, params.near);
            EXPECT_NEAR(params.want.z, params.input2.z, params.near);
        }
    }
}

INSTANTIATE_TEST_SUITE_P(
    Transforms,
    NedToAerTest,
    testing::Values(
        NedToAerTestParams{
            "NULL input 1",        // name
            true,                  // input1Null
            false,                 // input2Null
            {1.0, 1.0, 1.0},       // input1
            {0.0, 0.0, 0.0},       // input2
            {0.0, 0.0, 0.0},       // want
            0.0,                   // near
            KL_ERROR_NULL_ARGUMENT // wantError
        },
        NedToAerTestParams{
            "NULL input 2",        // name
            false,                 // input1Null
            true,                  // input2Null
            {1.0, 1.0, 1.0},       // input1
            {0.0, 0.0, 0.0},       // input2
            {0.0, 0.0, 0.0},       // want
            0.0,                   // near
            KL_ERROR_NULL_ARGUMENT // wantError
        },
        NedToAerTestParams{
            "Transform",                                // name
            false,                                      // input1Null
            false,                                      // input2Null
            {-46.43897628, -26.81155573, 44.99513264},  // input1
            {0.0, 0.0, 0.0},                            // input2
            {3.665191434 - (2 * PI), -0.6981317, 70.0}, // want
            1.0E-6,                                     // near
            KL_NO_ERROR                                 // wantError
        }),
    [](const ::testing::TestParamInfo<NedToAerTestParams> &info) -> std::string {
        std::string name = info.param.name;
        std::replace(name.begin(), name.end(), ' ', '_');
        return name;
    });
