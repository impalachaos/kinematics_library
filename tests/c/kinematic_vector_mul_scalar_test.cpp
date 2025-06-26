#include <gtest/gtest.h>
#include <kinematics-library/errors.h>
#include <kinematics-library/kinematic_vector.h>
#include <stdint.h>
#include <string>
#include <universal-constants/constants.h>

struct KvMulScalarTestParams
{
    std::string      name;
    bool             input1Null;
    bool             resultNull;
    kvector_t        input1;
    precision_type_t input2;
    kvector_t        want;
    precision_type_t near;
    int              wantError;
};

class MulScalarTest : public testing::TestWithParam<KvMulScalarTestParams>
{};

TEST_P(MulScalarTest, Compute)
{
    KvMulScalarTestParams params = GetParam();

    kvector_t        *input1 = &params.input1;
    precision_type_t *input2 = &params.input2;

    if (params.input1Null)
    {
        input1 = NULL;
    }

    kvector_t out;

    kvector_t *result = &out;

    if (params.resultNull)
    {
        result = NULL;
    }

    int error;
    error = kvector_mulf(input1, params.input2, result);
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
    MulScalarTest,
    testing::Values(
        KvMulScalarTestParams{
            "NULL input 1 mulf",   // name
            true,                  // input1Null
            false,                 // resultNull
            {1.0, 1.0, 1.0},       // input1
            0.0,                   // input2
            {0.0, 0.0, 0.0},       // want
            0.0,                   // near
            KL_ERROR_NULL_ARGUMENT // wantError
        },
        KvMulScalarTestParams{
            "NULL result mulf",    // name
            false,                 // input1Null
            true,                  // resultNull
            {1.0, 1.0, 1.0},       // input1
            0.0,                   // input2
            {0.0, 0.0, 0.0},       // want
            0.0,                   // near
            KL_ERROR_NULL_ARGUMENT // wantError
        },
        KvMulScalarTestParams{
            "mulf",           // name
            false,            // input1Null
            false,            // resultNull
            {1.0, 2.0, 3.0},  // input1
            4,                // input2
            {4.0, 8.0, 12.0}, // want
            0.0,              // near
            KL_NO_ERROR       // wantError
        }),
    [](const ::testing::TestParamInfo<KvMulScalarTestParams> &info) -> std::string {
        std::string name = info.param.name;
        std::replace(name.begin(), name.end(), ' ', '_');
        return name;
    });
