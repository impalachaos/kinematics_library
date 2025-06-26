#include <gtest/gtest.h>
#include <kinematics-library/errors.h>
#include <kinematics-library/rotations.h>
#include <stdint.h>
#include <string>
#include <universal-constants/constants.h>

struct DCMCompareDCMsTestParams
{
    std::string name;
    bool        input1Null;
    bool        input2Null;
    dcm_t       input1;
    dcm_t       input2;
    int         wantError;
};

class CompareDCMsTest : public testing::TestWithParam<DCMCompareDCMsTestParams>
{};

TEST_P(CompareDCMsTest, Compute)
{
    DCMCompareDCMsTestParams params = GetParam();

    dcm_t *input1 = &params.input1;
    dcm_t *input2 = &params.input2;

    if (params.input1Null)
    {
        input1 = NULL;
    }

    if (params.input2Null)
    {
        input2 = NULL;
    }

    int error = dcm_compare(input1, input2);
    EXPECT_EQ(error, params.wantError);
}

INSTANTIATE_TEST_SUITE_P(
    DCM,
    CompareDCMsTest,
    testing::Values(
        DCMCompareDCMsTestParams{
            "Both NULL",                                   // name
            true,                                          // input1Null
            true,                                          // input2Null
            {1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, // input1
            {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, // input2
            0                                              // wantError
        },
        DCMCompareDCMsTestParams{
            "NULL input 1",                                // name
            true,                                          // input1Null
            false,                                         // input2Null
            {1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, // input1
            {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, // input2
            1                                              // wantError
        },
        DCMCompareDCMsTestParams{
            "NULL input 2",                                // name
            false,                                         // input1Null
            true,                                          // input2Null
            {1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, // input1
            {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, // input2
            1                                              // wantError
        },
        DCMCompareDCMsTestParams{
            "Equal",                                       // name
            false,                                         // input1Null
            false,                                         // input2Null
            {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0}, // input1
            {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0}, // input2
            0                                              // wantError
        },
        DCMCompareDCMsTestParams{
            "00 not equal",                                // name
            false,                                         // input1Null
            false,                                         // input2Null
            {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0}, // input1
            {2.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0}, // input2
            1                                              // wantError
        },
        DCMCompareDCMsTestParams{
            "01 not equal",                                // name
            false,                                         // input1Null
            false,                                         // input2Null
            {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0}, // input1
            {1.0, 2.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0}, // input2
            1                                              // wantError
        },
        DCMCompareDCMsTestParams{
            "02 not equal",                                // name
            false,                                         // input1Null
            false,                                         // input2Null
            {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0}, // input1
            {1.0, 1.0, 2.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0}, // input2
            1                                              // wantError
        },
        DCMCompareDCMsTestParams{
            "10 not equal",                                // name
            false,                                         // input1Null
            false,                                         // input2Null
            {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0}, // input1
            {1.0, 1.0, 1.0, 2.0, 1.0, 1.0, 1.0, 1.0, 1.0}, // input2
            1                                              // wantError
        },
        DCMCompareDCMsTestParams{
            "11 not equal",                                // name
            false,                                         // input1Null
            false,                                         // input2Null
            {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0}, // input1
            {1.0, 1.0, 1.0, 1.0, 2.0, 1.0, 1.0, 1.0, 1.0}, // input2
            1                                              // wantError
        },
        DCMCompareDCMsTestParams{
            "12 not equal",                                // name
            false,                                         // input1Null
            false,                                         // input2Null
            {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0}, // input1
            {1.0, 1.0, 1.0, 1.0, 1.0, 2.0, 1.0, 1.0, 1.0}, // input2
            1                                              // wantError
        },
        DCMCompareDCMsTestParams{
            "20 not equal",                                // name
            false,                                         // input1Null
            false,                                         // input2Null
            {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0}, // input1
            {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 2.0, 1.0, 1.0}, // input2
            1                                              // wantError
        },
        DCMCompareDCMsTestParams{
            "21 not equal",                                // name
            false,                                         // input1Null
            false,                                         // input2Null
            {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0}, // input1
            {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 2.0, 1.0}, // input2
            1                                              // wantError
        },
        DCMCompareDCMsTestParams{
            "22 not equal",                                // name
            false,                                         // input1Null
            false,                                         // input2Null
            {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0}, // input1
            {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 2.0}, // input2
            1                                              // wantError
        }),
    [](const ::testing::TestParamInfo<DCMCompareDCMsTestParams> &info) -> std::string {
        std::string name = info.param.name;
        std::replace(name.begin(), name.end(), ' ', '_');
        return name;
    });
