#include <gtest/gtest.h>
#include <kinematics-library/errors.h>
#include <kinematics-library/kinematic_vector.h>
#include <stdint.h>
#include <string>
#include <universal-constants/constants.h>

struct KvAddVectorsTestParams
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

class AddVectorsTest : public testing::TestWithParam<KvAddVectorsTestParams>
{};

TEST_P(AddVectorsTest, Compute)
{
    KvAddVectorsTestParams params = GetParam();

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

    int error = kvector_add(input1, input2, result);
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
    AddVectorsTest,
    testing::Values(
        KvAddVectorsTestParams{
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
        KvAddVectorsTestParams{
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
        KvAddVectorsTestParams{
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
        KvAddVectorsTestParams{
            "Add 1",          // name
            false,            // input1Null
            false,            // input2Null
            false,            // resultNull
            {1.0, 2.0, 3.0},  // input1
            {5.0, 6.0, 7.0},  // input2
            {6.0, 8.0, 10.0}, // want
            0.0,              // near
            KL_NO_ERROR       // wantError
        }),
    [](const ::testing::TestParamInfo<KvAddVectorsTestParams> &info) -> std::string {
        std::string name = info.param.name;
        std::replace(name.begin(), name.end(), ' ', '_');
        return name;
    });
