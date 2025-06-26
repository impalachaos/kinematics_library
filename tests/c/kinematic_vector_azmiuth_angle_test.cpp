#include <gtest/gtest.h>
#include <kinematics-library/errors.h>
#include <kinematics-library/kinematic_vector.h>
#include <math.h>
#include <stdint.h>
#include <string>
#include <universal-constants/constants.h>

struct KvAzmiuthAngleTestParams
{
    std::string      name;
    bool             input1Null;
    bool             resultNull;
    kvector_t        input1;
    precision_type_t want;
    precision_type_t near;
    int              wantError;
};

class AzmiuthAngleTest : public testing::TestWithParam<KvAzmiuthAngleTestParams>
{};

TEST_P(AzmiuthAngleTest, Compute)
{
    KvAzmiuthAngleTestParams params = GetParam();

    kvector_t *input1 = &params.input1;

    if (params.input1Null)
    {
        input1 = NULL;
    }

    precision_type_t out;

    precision_type_t *result = &out;

    if (params.resultNull)
    {
        result = NULL;
    }

    int error = kvector_azimuth_angle(input1, result);
    EXPECT_EQ(error, params.wantError);
    if (error == KL_NO_ERROR)
    {
        if (params.near == 0)
        {
            EXPECT_DOUBLE_EQ(params.want, out);
        }
        else
        {
            EXPECT_NEAR(params.want, out, params.near);
        }
    }
}

INSTANTIATE_TEST_SUITE_P(
    KinematicVector,
    AzmiuthAngleTest,
    testing::Values(
        KvAzmiuthAngleTestParams{
            "NULL input 1",        // name
            true,                  // input1Null
            false,                 // resultNull
            {1.0, 1.0, 1.0},       // input1
            0.0,                   // want
            0.0,                   // near
            KL_ERROR_NULL_ARGUMENT // wantError
        },
        KvAzmiuthAngleTestParams{
            "NULL result",         // name
            false,                 // input1Null
            true,                  // resultNull
            {1.0, 1.0, 1.0},       // input1
            0.0,                   // want
            0.0,                   // near
            KL_ERROR_NULL_ARGUMENT // wantError
        },
        KvAzmiuthAngleTestParams{
            "Quadrant 1",           // name
            false,                  // input1Null
            false,                  // resultNull
            {sqrt(3.0), 1.0, 10.0}, // input1
            30.0 * DEG_TO_RAD,      // want
            1.0E-7,                 // near
            KL_NO_ERROR             // wantError
        },
        KvAzmiuthAngleTestParams{
            "Quadrant 2",            // name
            false,                   // input1Null
            false,                   // resultNull
            {-sqrt(3.0), 1.0, 10.0}, // input1
            150.0 * DEG_TO_RAD,      // want
            0.0,                     // near
            KL_NO_ERROR              // wantError
        },
        KvAzmiuthAngleTestParams{
            "Quadrant 3",             // name
            false,                    // input1Null
            false,                    // resultNull
            {-1.0, -sqrt(3.0), 10.0}, // input1
            -120.0 * DEG_TO_RAD,      // want
            0.0,                      // near
            KL_NO_ERROR               // wantError
        },
        KvAzmiuthAngleTestParams{
            "Quadrant 4",            // name
            false,                   // input1Null
            false,                   // resultNull
            {1.0, -sqrt(3.0), 10.0}, // input1
            -60.0 * DEG_TO_RAD,      // want
            0.0,                     // near
            KL_NO_ERROR              // wantError
        }),
    [](const ::testing::TestParamInfo<KvAzmiuthAngleTestParams> &info) -> std::string {
        std::string name = info.param.name;
        std::replace(name.begin(), name.end(), ' ', '_');
        return name;
    });
