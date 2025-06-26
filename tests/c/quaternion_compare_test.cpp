#include <gtest/gtest.h>
#include <kinematics-library/errors.h>
#include <kinematics-library/rotations.h>
#include <stdint.h>
#include <string>
#include <universal-constants/constants.h>

struct QCompareQuaternionsTestParams
{
    std::string  name;
    bool         input1Null;
    bool         input2Null;
    quaternion_t input1;
    quaternion_t input2;
    int          wantError;
};

class CompareQuaternionsTest : public testing::TestWithParam<QCompareQuaternionsTestParams>
{};

TEST_P(CompareQuaternionsTest, Compute)
{
    QCompareQuaternionsTestParams params = GetParam();

    quaternion_t *input1 = &params.input1;
    quaternion_t *input2 = &params.input2;

    if (params.input1Null)
    {
        input1 = NULL;
    }

    if (params.input2Null)
    {
        input2 = NULL;
    }

    int error = quaternion_compare(input1, input2);
    EXPECT_EQ(error, params.wantError);
}

INSTANTIATE_TEST_SUITE_P(
    Quaternion,
    CompareQuaternionsTest,
    testing::Values(
        QCompareQuaternionsTestParams{
            "Both NULL",          // name
            true,                 // input1Null
            true,                 // input2Null
            {1.0, 1.0, 1.0, 0.0}, // input1
            {1.0, 0.0, 0.0, 0.0}, // input2
            0                     // wantError
        },
        QCompareQuaternionsTestParams{
            "NULL input 1",       // name
            true,                 // input1Null
            false,                // input2Null
            {1.0, 1.0, 1.0, 0.0}, // input1
            {1.0, 0.0, 0.0, 0.0}, // input2
            1                     // wantError
        },
        QCompareQuaternionsTestParams{
            "NULL input 2",       // name
            false,                // input1Null
            true,                 // input2Null
            {1.0, 1.0, 1.0, 0.0}, // input1
            {1.0, 0.0, 0.0, 0.0}, // input2
            1                     // wantError
        },
        QCompareQuaternionsTestParams{
            "Equal",              // name
            false,                // input1Null
            false,                // input2Null
            {1.0, 1.0, 1.0, 1.0}, // input1
            {1.0, 1.0, 1.0, 1.0}, // input2
            0                     // wantError
        },
        QCompareQuaternionsTestParams{
            "a not equal",        // name
            false,                // input1Null
            false,                // input2Null
            {1.0, 1.0, 1.0, 1.0}, // input1
            {2.0, 1.0, 1.0, 1.0}, // input2
            1                     // wantError
        },
        QCompareQuaternionsTestParams{
            "b not equal",        // name
            false,                // input1Null
            false,                // input2Null
            {1.0, 1.0, 1.0, 1.0}, // input1
            {1.0, 2.0, 1.0, 1.0}, // input2
            1                     // wantError
        },
        QCompareQuaternionsTestParams{
            "c not equal",        // name
            false,                // input1Null
            false,                // input2Null
            {1.0, 1.0, 1.0, 1.0}, // input1
            {1.0, 1.0, 2.0, 1.0}, // input2
            1                     // wantError
        },
        QCompareQuaternionsTestParams{
            "d not equal",        // name
            false,                // input1Null
            false,                // input2Null
            {1.0, 1.0, 1.0, 1.0}, // input1
            {1.0, 1.0, 1.0, 2.0}, // input2
            1                     // wantError
        }),
    [](const ::testing::TestParamInfo<QCompareQuaternionsTestParams> &info) -> std::string {
        std::string name = info.param.name;
        std::replace(name.begin(), name.end(), ' ', '_');
        return name;
    });
