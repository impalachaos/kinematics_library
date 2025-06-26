#include <gtest/gtest.h>
#include <kinematics-library/errors.h>
#include <kinematics-library/kinematic_vector.h>
#include <stdint.h>
#include <string>
#include <universal-constants/constants.h>

struct KvSubScalarTestParams
{
    std::string      name;
    bool             input1Null;
    bool             resultNull;
    kvector_t        input1;
    precision_type_t input2;
    bool             order;
    kvector_t        want;
    precision_type_t near;
    int              wantError;
};

class SubScalarTest : public testing::TestWithParam<KvSubScalarTestParams>
{};

TEST_P(SubScalarTest, Compute)
{
    KvSubScalarTestParams params = GetParam();

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
    if (params.order)
    {
        error = kvector_subf(input1, params.input2, result);
    }
    else
    {
        error = kvector_subf2(params.input2, input1, result);
    }
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
    SubScalarTest,
    testing::Values(
        KvSubScalarTestParams{
            "NULL input 1 subf",   // name
            true,                  // input1Null
            false,                 // resultNull
            {1.0, 1.0, 1.0},       // input1
            0.0,                   // input2
            false,                 // order
            {0.0, 0.0, 0.0},       // want
            0.0,                   // near
            KL_ERROR_NULL_ARGUMENT // wantError
        },
        KvSubScalarTestParams{
            "NULL result subf",    // name
            false,                 // input1Null
            true,                  // resultNull
            {1.0, 1.0, 1.0},       // input1
            0.0,                   // input2
            false,                 // order
            {0.0, 0.0, 0.0},       // want
            0.0,                   // near
            KL_ERROR_NULL_ARGUMENT // wantError
        },
        KvSubScalarTestParams{
            "NULL input 1 subf2",  // name
            true,                  // input1Null
            false,                 // resultNull
            {1.0, 1.0, 1.0},       // input1
            0.0,                   // input2
            true,                  // order
            {0.0, 0.0, 0.0},       // want
            0.0,                   // near
            KL_ERROR_NULL_ARGUMENT // wantError
        },
        KvSubScalarTestParams{
            "NULL result subf2",   // name
            false,                 // input1Null
            true,                  // resultNull
            {1.0, 1.0, 1.0},       // input1
            0.0,                   // input2
            true,                  // order
            {0.0, 0.0, 0.0},       // want
            0.0,                   // near
            KL_ERROR_NULL_ARGUMENT // wantError
        },
        KvSubScalarTestParams{
            "subf",             // name
            false,              // input1Null
            false,              // resultNull
            {1.0, 2.0, 3.0},    // input1
            4,                  // input2
            true,               // order
            {-3.0, -2.0, -1.0}, // want
            0.0,                // near
            KL_NO_ERROR         // wantError
        },
        KvSubScalarTestParams{
            "subf2",         // name
            false,           // input1Null
            false,           // resultNull
            {1.0, 2.0, 3.0}, // input1
            4,               // input2
            false,           // order
            {3.0, 2.0, 1.0}, // want
            0.0,             // near
            KL_NO_ERROR      // wantError
        }),
    [](const ::testing::TestParamInfo<KvSubScalarTestParams> &info) -> std::string {
        std::string name = info.param.name;
        std::replace(name.begin(), name.end(), ' ', '_');
        return name;
    });
