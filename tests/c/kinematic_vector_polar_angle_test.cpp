#include <gtest/gtest.h>
#include <kinematics-library/errors.h>
#include <kinematics-library/kinematic_vector.h>
#include <stdint.h>
#include <string>
#include <universal-constants/constants.h>

struct KvPolarAngleTestParams
{
    std::string      name;
    bool             input1Null;
    bool             resultNull;
    kvector_t        input1;
    precision_type_t want;
    precision_type_t near;
    int              wantError;
};

class PolarAngleTest : public testing::TestWithParam<KvPolarAngleTestParams>
{};

TEST_P(PolarAngleTest, Compute)
{
    KvPolarAngleTestParams params = GetParam();

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

    int error = kvector_polar_angle(input1, result);
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
    PolarAngleTest,
    testing::Values(
        KvPolarAngleTestParams{
            "NULL input 1",        // name
            true,                  // input1Null
            false,                 // resultNull
            {1.0, 1.0, 1.0},       // input1
            0.0,                   // want
            0.0,                   // near
            KL_ERROR_NULL_ARGUMENT // wantError
        },
        KvPolarAngleTestParams{
            "NULL result",         // name
            false,                 // input1Null
            true,                  // resultNull
            {1.0, 1.0, 1.0},       // input1
            0.0,                   // want
            0.0,                   // near
            KL_ERROR_NULL_ARGUMENT // wantError
        },
        KvPolarAngleTestParams{
            "on z axis",     // name
            false,           // input1Null
            false,           // resultNull
            {0.0, 0.0, 5.0}, // input1
            0.0,             // want
            1.0E-7,          // near
            KL_NO_ERROR      // wantError
        },
        KvPolarAngleTestParams{
            "above XY plane but off z axis", // name
            false,                           // input1Null
            false,                           // resultNull
            {5.0, 0.0, 5.0},                 // input1
            45.0 * DEG_TO_RAD,               // want
            0.0,                             // near
            KL_NO_ERROR                      // wantError
        },
        KvPolarAngleTestParams{
            "in XY plane",     // name
            false,             // input1Null
            false,             // resultNull
            {5.0, 0.0, 0.0},   // input1
            90.0 * DEG_TO_RAD, // want
            0.0,               // near
            KL_NO_ERROR        // wantError
        },
        KvPolarAngleTestParams{
            "below XY plane but off z axis", // name
            false,                           // input1Null
            false,                           // resultNull
            {5.0, 0.0, -5.0},                // input1
            135.0 * DEG_TO_RAD,              // want
            0.0,                             // near
            KL_NO_ERROR                      // wantError
        },
        KvPolarAngleTestParams{
            "along negative z axis", // name
            false,                   // input1Null
            false,                   // resultNull
            {0.0, 0.0, -5.0},        // input1
            180.0 * DEG_TO_RAD,      // want
            1.0E-7,                  // near
            KL_NO_ERROR              // wantError
        }),
    [](const ::testing::TestParamInfo<KvPolarAngleTestParams> &info) -> std::string {
        std::string name = info.param.name;
        std::replace(name.begin(), name.end(), ' ', '_');
        return name;
    });
