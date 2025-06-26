#include <gtest/gtest.h>
#include <kinematics-library/errors.h>
#include <kinematics-library/kinematic_vector.h>
#include <stdint.h>
#include <string>
#include <universal-constants/constants.h>

struct KvAngleBetweenTestParams
{
    std::string      name;
    bool             input1Null;
    bool             input2Null;
    bool             resultNull;
    kvector_t        input1;
    kvector_t        input2;
    precision_type_t want;
    precision_type_t near;
    int              wantError;
};

class AngleBetweenTest : public testing::TestWithParam<KvAngleBetweenTestParams>
{};

TEST_P(AngleBetweenTest, Compute)
{
    KvAngleBetweenTestParams params = GetParam();

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

    precision_type_t out;

    precision_type_t *result = &out;

    if (params.resultNull)
    {
        result = NULL;
    }

    int error = kvector_angle_between(input1, input2, result);
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
    AngleBetweenTest,
    testing::Values(
        KvAngleBetweenTestParams{
            "NULL input 1",        // name
            true,                  // input1Null
            false,                 // input2Null
            false,                 // resultNull
            {1.0, 1.0, 1.0},       // input1
            {1.0, 0.0, 0.0},       // input2
            0.0,                   // want
            0.0,                   // near
            KL_ERROR_NULL_ARGUMENT // wantError
        },
        KvAngleBetweenTestParams{
            "NULL input 2",        // name
            false,                 // input1Null
            true,                  // input2Null
            false,                 // resultNull
            {1.0, 1.0, 1.0},       // input1
            {1.0, 0.0, 0.0},       // input2
            0.0,                   // want
            0.0,                   // near
            KL_ERROR_NULL_ARGUMENT // wantError
        },
        KvAngleBetweenTestParams{
            "NULL result",         // name
            false,                 // input1Null
            false,                 // input2Null
            true,                  // resultNull
            {1.0, 1.0, 1.0},       // input1
            {1.0, 0.0, 0.0},       // input2
            0.0,                   // want
            0.0,                   // near
            KL_ERROR_NULL_ARGUMENT // wantError
        },
        KvAngleBetweenTestParams{
            "0 angle",       // name
            false,           // input1Null
            false,           // input2Null
            false,           // resultNull
            {1.0, 1.0, 1.0}, // input1
            {1.0, 1.0, 1.0}, // input2
            0.0,             // want
            1.0E-7,          // near
            KL_NO_ERROR      // wantError
        },
        KvAngleBetweenTestParams{
            "90 angle 1",    // name
            false,           // input1Null
            false,           // input2Null
            false,           // resultNull
            {0.0, 1.0, 0.0}, // input1
            {1.0, 0.0, 0.0}, // input2
            PI / 2.0,        // want
            0.0,             // near
            KL_NO_ERROR      // wantError
        },
        KvAngleBetweenTestParams{
            "90 angle 2",    // name
            false,           // input1Null
            false,           // input2Null
            false,           // resultNull
            {1.0, 0.0, 0.0}, // input1
            {0.0, 1.0, 0.0}, // input2
            PI / 2.0,        // want
            0.0,             // near
            KL_NO_ERROR      // wantError
        },
        KvAngleBetweenTestParams{
            "complex 3d 1",                  // name
            false,                           // input1Null
            false,                           // input2Null
            false,                           // resultNull
            {1.0, -4.0, 0.5},                // input1
            {-1.2, 2.0, -3.0},               // input2
            132.68462504216254 * PI / 180.0, // want
            0.0,                             // near
            KL_NO_ERROR                      // wantError
        },
        KvAngleBetweenTestParams{
            "complex 3d 2",                  // name
            false,                           // input1Null
            false,                           // input2Null
            false,                           // resultNull
            {-1.2, 2.0, -3.0},               // input1
            {1.0, -4.0, 0.5},                // input2
            132.68462504216254 * PI / 180.0, // want
            0.0,                             // near
            KL_NO_ERROR                      // wantError
        },
        KvAngleBetweenTestParams{
            "0 magnitude 1",    // name
            false,              // input1Null
            false,              // input2Null
            false,              // resultNull
            {0.0, 0.0, 0.0},    // input1
            {-1.2, 2.0, -3.0},  // input2
            1.5707963267948966, // want
            0.0,                // near
            KL_NO_ERROR         // wantError
        },
        KvAngleBetweenTestParams{
            "0 magnitude 2",    // name
            false,              // input1Null
            false,              // input2Null
            false,              // resultNull
            {0.0, 0.0, 0.0},    // input1
            {0.0, 0.0, 0.0},    // input2
            1.5707963267948966, // want
            0.0,                // near
            KL_NO_ERROR         // wantError
        }),
    [](const ::testing::TestParamInfo<KvAngleBetweenTestParams> &info) -> std::string {
        std::string name = info.param.name;
        std::replace(name.begin(), name.end(), ' ', '_');
        return name;
    });
