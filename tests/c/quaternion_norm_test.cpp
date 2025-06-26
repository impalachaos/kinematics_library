#include <gtest/gtest.h>
#include <kinematics-library/errors.h>
#include <kinematics-library/rotations.h>
#include <stdint.h>
#include <string>
#include <universal-constants/constants.h>

struct QNormTestParams
{
    std::string      name;
    bool             input1Null;
    bool             resultNull;
    quaternion_t     input1;
    precision_type_t want;
    precision_type_t near;
    int              wantError;
};

class NormTest : public testing::TestWithParam<QNormTestParams>
{};

TEST_P(NormTest, Compute)
{
    QNormTestParams params = GetParam();

    quaternion_t *input1 = &params.input1;

    if (params.input1Null)
    {
        input1 = NULL;
    }

    int error;

    precision_type_t result;

    precision_type_t *resultPtr = &result;

    if (params.resultNull)
    {
        resultPtr = NULL;
    }

    error = quaternion_norm(input1, resultPtr);
    EXPECT_EQ(error, params.wantError);
    if (error == KL_NO_ERROR)
    {
        if (params.near == 0)
        {
            EXPECT_DOUBLE_EQ(params.want, result);
        }
        else
        {
            EXPECT_NEAR(params.want, result, params.near);
        }
    }
}

INSTANTIATE_TEST_SUITE_P(
    Quaternion,
    NormTest,
    testing::Values(
        QNormTestParams{
            "Input 1 null",        // name
            true,                  // input1Null
            false,                 // resultNull
            {1.0, 2.0, 3.0, 4.0},  // input1
            0.0,                   // want
            0.0,                   // near
            KL_ERROR_NULL_ARGUMENT // wantError
        },
        QNormTestParams{
            "Result null",         // name
            false,                 // input1Null
            true,                  // resultNull
            {1.0, 2.0, 3.0, 4.0},  // input1
            0.0,                   // want
            0.0,                   // near
            KL_ERROR_NULL_ARGUMENT // wantError
        },
        QNormTestParams{
            "Normal",             // name
            false,                // input1Null
            false,                // resultNull
            {1.0, 2.0, 3.0, 4.0}, // input1
            5.4772255750516612,   // want
            0.0,                  // near
            KL_NO_ERROR           // wantError
        }),
    [](const ::testing::TestParamInfo<QNormTestParams> &info) -> std::string {
        std::string name = info.param.name;
        std::replace(name.begin(), name.end(), ' ', '_');
        return name;
    });
