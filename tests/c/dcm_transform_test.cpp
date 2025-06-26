#include <gtest/gtest.h>
#include <kinematics-library/errors.h>
#include <kinematics-library/rotations.h>
#include <stdint.h>
#include <string>
#include <universal-constants/constants.h>

struct DCMTransformTestParams
{
    std::string      name;
    bool             input1Null;
    bool             input2Null;
    bool             resultNull;
    dcm_t            input1;
    kvector_t        input2;
    kvector_t        want;
    precision_type_t near;
    int              wantError;
};

class TransformTest : public testing::TestWithParam<DCMTransformTestParams>
{};

TEST_P(TransformTest, Compute)
{
    DCMTransformTestParams params = GetParam();

    dcm_t     *input1 = &params.input1;
    kvector_t *input2 = &params.input2;

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

    int error = dcm_transform(input1, input2, resultPtr);
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
    DCM,
    TransformTest,
    testing::Values(
        DCMTransformTestParams{
            "NULL input 1",                                // name
            true,                                          // input1Null
            false,                                         // input2Null
            false,                                         // resultNull
            {1.0, 0.0, 0.0, 2.0, 1.0, 0.0, 3.0, 2.0, 1.0}, // input1
            {1.0, 1.0, 1.0},                               // input2
            {1.0, 2.0, 3.0},                               // want
            0.0,                                           // near
            KL_ERROR_NULL_ARGUMENT                         // wantError
        },
        DCMTransformTestParams{
            "NULL input 2",                                // name
            false,                                         // input1Null
            true,                                          // input2Null
            false,                                         // resultNull
            {1.0, 0.0, 0.0, 2.0, 1.0, 0.0, 3.0, 2.0, 1.0}, // input1
            {1.0, 1.0, 1.0},                               // input2
            {1.0, 2.0, 3.0},                               // want
            0.0,                                           // near
            KL_ERROR_NULL_ARGUMENT                         // wantError
        },
        DCMTransformTestParams{
            "NULL result",                                 // name
            false,                                         // input1Null
            false,                                         // input2Null
            true,                                          // resultNull
            {1.0, 0.0, 0.0, 2.0, 1.0, 0.0, 3.0, 2.0, 1.0}, // input1
            {1.0, 1.0, 1.0},                               // input2
            {1.0, 2.0, 3.0},                               // want
            0.0,                                           // near
            KL_ERROR_NULL_ARGUMENT                         // wantError
        },
        DCMTransformTestParams{
            "Transform",                                   // name
            false,                                         // input1Null
            false,                                         // input2Null
            false,                                         // resultNull
            {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0}, // input1
            {1.0, 2.0, 3.0},                               // input2
            {14.0, 32.0, 50.0},                            // want
            0.0,                                           // near
            KL_NO_ERROR                                    // wantError
        }),
    [](const ::testing::TestParamInfo<DCMTransformTestParams> &info) -> std::string {
        std::string name = info.param.name;
        std::replace(name.begin(), name.end(), ' ', '_');
        return name;
    });
