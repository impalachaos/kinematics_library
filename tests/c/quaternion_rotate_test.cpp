#include <gtest/gtest.h>
#include <kinematics-library/errors.h>
#include <kinematics-library/rotations.h>
#include <stdint.h>
#include <string>
#include <universal-constants/constants.h>

struct QRotateTestParams
{
    std::string      name;
    bool             input1Null;
    bool             input2Null;
    bool             resultNull;
    quaternion_t     input1;
    precision_type_t input1a1;
    precision_type_t input1a2;
    precision_type_t input1a3;
    int              input1Rotation;
    kvector_t        input2;
    kvector_t        want;
    precision_type_t near;
    int              wantError;
};

class RotateTest : public testing::TestWithParam<QRotateTestParams>
{};

TEST_P(RotateTest, Compute)
{
    QRotateTestParams params = GetParam();

    quaternion_t *input1 = &params.input1;
    kvector_t    *input2 = &params.input2;

    int error;

    error = quaternion_init(input1, params.input1a1, params.input1a2, params.input1a3, params.input1Rotation);
    EXPECT_EQ(error, KL_NO_ERROR);

    if (params.input1Null)
    {
        input1 = NULL;
    }

    if (params.input2Null)
    {
        input2 = NULL;
    }

    kvector_t result;

    kvector_t *resultPtr = &result;

    if (params.resultNull)
    {
        resultPtr = NULL;
    }

    error = quaternion_rotate(input1, input2, resultPtr);
    EXPECT_EQ(error, params.wantError);
    if (error == KL_NO_ERROR)
    {
        if (params.near == 0)
        {
            EXPECT_DOUBLE_EQ(params.want.x, resultPtr->x);
            EXPECT_DOUBLE_EQ(params.want.y, resultPtr->y);
            EXPECT_DOUBLE_EQ(params.want.z, resultPtr->z);
        }
        else
        {
            EXPECT_NEAR(params.want.x, resultPtr->x, params.near);
            EXPECT_NEAR(params.want.y, resultPtr->y, params.near);
            EXPECT_NEAR(params.want.z, resultPtr->z, params.near);
        }
    }
}

INSTANTIATE_TEST_SUITE_P(
    Quaternion,
    RotateTest,
    testing::Values(
        QRotateTestParams{
            "Input 1 null",        // name
            true,                  // input1Null
            false,                 // input2Null
            false,                 // resultNull
            {1.0, 1.0, 1.0, 1.0},  // input1
            -1.28,                 // input1a1
            1.54,                  // input1a2
            -1.95,                 // input1a3
            ROTATION_SEQ_BODY_XYX, // input1Rotation
            {0.0, 0.0, 0.0},       // input2
            {0.0, 0.0, 0.0},       // want
            0.0,                   // near
            KL_ERROR_NULL_ARGUMENT // wantError
        },
        QRotateTestParams{
            "Input 2 null",        // name
            false,                 // input1Null
            true,                  // input2Null
            false,                 // resultNull
            {1.0, 1.0, 1.0, 1.0},  // input1
            -1.28,                 // input1a1
            1.54,                  // input1a2
            -1.95,                 // input1a3
            ROTATION_SEQ_BODY_XYX, // input1Rotation
            {0.0, 0.0, 0.0},       // input2
            {0.0, 0.0, 0.0},       // want
            0.0,                   // near
            KL_ERROR_NULL_ARGUMENT // wantError
        },
        QRotateTestParams{
            "Result null",         // name
            false,                 // input1Null
            false,                 // input2Null
            true,                  // resultNull
            {1.0, 1.0, 1.0, 1.0},  // input1
            -1.28,                 // input1a1
            1.54,                  // input1a2
            -1.95,                 // input1a3
            ROTATION_SEQ_BODY_XYX, // input1Rotation
            {0.0, 0.0, 0.0},       // input2
            {0.0, 0.0, 0.0},       // want
            0.0,                   // near
            KL_ERROR_NULL_ARGUMENT // wantError
        },
        QRotateTestParams{
            "Rotate",                                                   // name
            false,                                                      // input1Null
            false,                                                      // input2Null
            false,                                                      // resultNull
            {1.0, 1.0, 1.0, 1.0},                                       // input1
            2.88,                                                       // input1a1
            -1.63,                                                      // input1a2
            1.11,                                                       // input1a3
            ROTATION_SEQ_BODY_ZYZ,                                      // input1Rotation
            {-2.1, 1.7, 2.0},                                           // input2
            {2.079128892722728, 0.608025132455055, -2.570511327683381}, // want
            1.0E-14,                                                    // near
            KL_NO_ERROR                                                 // wantError
        }),
    [](const ::testing::TestParamInfo<QRotateTestParams> &info) -> std::string {
        std::string name = info.param.name;
        std::replace(name.begin(), name.end(), ' ', '_');
        return name;
    });
