#include <gtest/gtest.h>
#include <kinematics-library/errors.h>
#include <kinematics-library/kinematic_vector.h>
#include <stdint.h>
#include <string>
#include <universal-constants/constants.h>

struct KvCompareVectorsTestParams
{
    std::string name;
    bool        input1Null;
    bool        input2Null;
    kvector_t   input1;
    kvector_t   input2;
    int         wantError;
};

class CompareVectorsTest : public testing::TestWithParam<KvCompareVectorsTestParams>
{};

TEST_P(CompareVectorsTest, Compute)
{
    KvCompareVectorsTestParams params = GetParam();

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

    int error = kvector_compare(input1, input2);
    EXPECT_EQ(error, params.wantError);
}

INSTANTIATE_TEST_SUITE_P(
    KinematicVector,
    CompareVectorsTest,
    testing::Values(
        KvCompareVectorsTestParams{
            "Both NULL",     // name
            true,            // input1Null
            true,            // input2Null
            {1.0, 1.0, 1.0}, // input1
            {1.0, 0.0, 0.0}, // input2
            0                // wantError
        },
        KvCompareVectorsTestParams{
            "NULL input 1",  // name
            true,            // input1Null
            false,           // input2Null
            {1.0, 1.0, 1.0}, // input1
            {1.0, 0.0, 0.0}, // input2
            1                // wantError
        },
        KvCompareVectorsTestParams{
            "NULL input 2",  // name
            false,           // input1Null
            true,            // input2Null
            {1.0, 1.0, 1.0}, // input1
            {1.0, 0.0, 0.0}, // input2
            1                // wantError
        },
        KvCompareVectorsTestParams{
            "Equal",         // name
            false,           // input1Null
            false,           // input2Null
            {1.0, 1.0, 1.0}, // input1
            {1.0, 1.0, 1.0}, // input2
            0                // wantError
        },
        KvCompareVectorsTestParams{
            "X not equal",   // name
            false,           // input1Null
            false,           // input2Null
            {1.0, 1.0, 1.0}, // input1
            {2.0, 1.0, 1.0}, // input2
            1                // wantError
        },
        KvCompareVectorsTestParams{
            "Y not equal",   // name
            false,           // input1Null
            false,           // input2Null
            {1.0, 1.0, 1.0}, // input1
            {1.0, 2.0, 1.0}, // input2
            1                // wantError
        },
        KvCompareVectorsTestParams{
            "Z not equal",   // name
            false,           // input1Null
            false,           // input2Null
            {1.0, 1.0, 1.0}, // input1
            {1.0, 1.0, 2.0}, // input2
            1                // wantError
        }),
    [](const ::testing::TestParamInfo<KvCompareVectorsTestParams> &info) -> std::string {
        std::string name = info.param.name;
        std::replace(name.begin(), name.end(), ' ', '_');
        return name;
    });
