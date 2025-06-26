#include <gtest/gtest.h>
#include <kinematics-library/errors.h>
#include <kinematics-library/kinematic_vector.h>
#include <stdint.h>
#include <string>
#include <universal-constants/constants.h>

struct KvCrossTestParams
{
    std::string      name;
    bool             input1Null;
    bool             input2Null;
    bool             resultNull;
    kvector_t        input1;
    kvector_t        input2;
    kvector_t        want;
    precision_type_t near;
    int              wantError;
};

class CrossTest : public testing::TestWithParam<KvCrossTestParams>
{};

TEST_P(CrossTest, Compute)
{
    KvCrossTestParams params = GetParam();

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

    kvector_t out;

    kvector_t *result = &out;

    if (params.resultNull)
    {
        result = NULL;
    }

    int error = kvector_cross(input1, input2, result);
    EXPECT_EQ(error, params.wantError);
    if (error == KL_NO_ERROR)
    {
        if (params.near == 0)
        {
            EXPECT_DOUBLE_EQ(params.want.x, out.x);
            EXPECT_DOUBLE_EQ(params.want.y, out.y);
            EXPECT_DOUBLE_EQ(params.want.z, out.z);
        }
        else
        {
            EXPECT_NEAR(params.want.x, out.x, params.near);
            EXPECT_NEAR(params.want.y, out.y, params.near);
            EXPECT_NEAR(params.want.z, out.z, params.near);
        }
    }
}

INSTANTIATE_TEST_SUITE_P(
    KinematicVector,
    CrossTest,
    testing::Values(
        KvCrossTestParams{
            "NULL input 1",        // name
            true,                  // input1Null
            false,                 // input2Null
            false,                 // resultNull
            {1.0, 1.0, 1.0},       // input1
            {1.0, 0.0, 0.0},       // input2
            {0.0, 0.0, 0.0},       // want
            0.0,                   // near
            KL_ERROR_NULL_ARGUMENT // wantError
        },
        KvCrossTestParams{
            "NULL input 2",        // name
            false,                 // input1Null
            true,                  // input2Null
            false,                 // resultNull
            {1.0, 1.0, 1.0},       // input1
            {1.0, 0.0, 0.0},       // input2
            {0.0, 0.0, 0.0},       // want
            0.0,                   // near
            KL_ERROR_NULL_ARGUMENT // wantError
        },
        KvCrossTestParams{
            "NULL result",         // name
            false,                 // input1Null
            false,                 // input2Null
            true,                  // resultNull
            {1.0, 1.0, 1.0},       // input1
            {1.0, 0.0, 0.0},       // input2
            {0.0, 0.0, 0.0},       // want
            0.0,                   // near
            KL_ERROR_NULL_ARGUMENT // wantError
        },
        KvCrossTestParams{
            "Cross 1",           // name
            false,               // input1Null
            false,               // input2Null
            false,               // resultNull
            {2.0, -3.0, 1.0},    // input1
            {4.0, -1.0, 5.0},    // input2
            {-14.0, -6.0, 10.0}, // want
            1.0E-7,              // near
            KL_NO_ERROR          // wantError
        },
        KvCrossTestParams{
            "Cross 2",          // name
            false,              // input1Null
            false,              // input2Null
            false,              // resultNull
            {4.0, -1.0, 5.0},   // input1
            {2.0, -3.0, 1.0},   // input2
            {14.0, 6.0, -10.0}, // want
            0.0,                // near
            KL_NO_ERROR         // wantError
        }),
    [](const ::testing::TestParamInfo<KvCrossTestParams> &info) -> std::string {
        std::string name = info.param.name;
        std::replace(name.begin(), name.end(), ' ', '_');
        return name;
    });
