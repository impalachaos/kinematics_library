#include <gtest/gtest.h>
#include <kinematics-library/errors.h>
#include <kinematics-library/kinematic_vector.h>
#include <stdint.h>
#include <string>
#include <universal-constants/constants.h>

struct KvElevationAngleTestParams
{
    std::string      name;
    bool             input1Null;
    bool             resultNull;
    kvector_t        input1;
    precision_type_t want;
    precision_type_t near;
    int              wantError;
};

class ElevationAngleTest : public testing::TestWithParam<KvElevationAngleTestParams>
{};

TEST_P(ElevationAngleTest, Compute)
{
    KvElevationAngleTestParams params = GetParam();

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

    int error = kvector_elevation_angle(input1, result);
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
    ElevationAngleTest,
    testing::Values(
        KvElevationAngleTestParams{
            "NULL input 1",        // name
            true,                  // input1Null
            false,                 // resultNull
            {1.0, 1.0, 1.0},       // input1
            0.0,                   // want
            0.0,                   // near
            KL_ERROR_NULL_ARGUMENT // wantError
        },
        KvElevationAngleTestParams{
            "NULL result",         // name
            false,                 // input1Null
            true,                  // resultNull
            {1.0, 1.0, 1.0},       // input1
            0.0,                   // want
            0.0,                   // near
            KL_ERROR_NULL_ARGUMENT // wantError
        },
        KvElevationAngleTestParams{
            "Above XY Plane",   // name
            false,              // input1Null
            false,              // resultNull
            {3.0, 4.0, 5.0},    // input1
            -45.0 * DEG_TO_RAD, // want
            0.0,                // near
            KL_NO_ERROR         // wantError
        },
        KvElevationAngleTestParams{
            "On XY Plane",    // name
            false,            // input1Null
            false,            // resultNull
            {3.0, 4.0, 0.0},  // input1
            0.0 * DEG_TO_RAD, // want
            0.0,              // near
            KL_NO_ERROR       // wantError
        },
        KvElevationAngleTestParams{
            "Below XY Plane",  // name
            false,             // input1Null
            false,             // resultNull
            {3.0, 4.0, -5.0},  // input1
            45.0 * DEG_TO_RAD, // want
            0.0,               // near
            KL_NO_ERROR        // wantError
        }),
    [](const ::testing::TestParamInfo<KvElevationAngleTestParams> &info) -> std::string {
        std::string name = info.param.name;
        std::replace(name.begin(), name.end(), ' ', '_');
        return name;
    });
