#include <gtest/gtest.h>
#include <kinematics-library/errors.h>
#include <kinematics-library/geometry.h>
#include <kinematics-library/kinematic_vector.h>
#include <universal-constants/constants.h>

#define ABS_TOL 1.0E-13
#define NEAR    1.0E-9

TEST(test_vincenty_direct, null_lla_a)
{
    kvector_t        lla_b = {0.0};
    precision_type_t range_m(0.0), bearing_rad(0.0);

    int rv = vincenty_direct(NULL, range_m, bearing_rad, ABS_TOL, &lla_b);
    EXPECT_EQ(rv, KL_ERROR_NULL_ARGUMENT) << "Unexpected rv for NULL lla_a";
}

TEST(test_vincenty_direct, invalid_lla_a_latitude)
{
    kvector_t        lla_a = {90.1, 0.0, 0.0};
    kvector_t        lla_b = {0.0};
    precision_type_t range_m(0.0), bearing_rad(0.0);

    int rv = vincenty_direct(&lla_a, range_m, bearing_rad, ABS_TOL, &lla_b);
    EXPECT_EQ(rv, KL_ERROR_INVALID_ARGUMENT) << "Unexpected rv for invalid latitude";
}

TEST(test_vincenty_direct, invalid_lla_a_longitude)
{
    kvector_t        lla_a = {0.0, 180.1, 0.0};
    kvector_t        lla_b = {0.0};
    precision_type_t range_m(0.0), bearing_rad(0.0);

    int rv = vincenty_direct(&lla_a, range_m, bearing_rad, ABS_TOL, &lla_b);
    EXPECT_EQ(rv, KL_ERROR_INVALID_ARGUMENT) << "Unexpected rv for invalid longitude";
}

TEST(test_vincenty_direct, invalid_range)
{
    kvector_t        lla_a, lla_b = {0.0};
    precision_type_t range_m(-0.1), bearing_rad(0.0);

    int rv = vincenty_direct(&lla_a, range_m, bearing_rad, ABS_TOL, &lla_b);
    EXPECT_EQ(rv, KL_ERROR_INVALID_ARGUMENT) << "Unexpected rv for invalid range_m";
}

TEST(test_vincenty_direct, invalid_abs_tol)
{
    kvector_t        lla_a, lla_b = {0.0};
    precision_type_t range_m(0.0), bearing_rad(0.0);

    int rv = vincenty_direct(&lla_a, range_m, bearing_rad, -0.1, &lla_b);
    EXPECT_EQ(rv, KL_ERROR_INVALID_ARGUMENT) << "Unexpected rv for invalid abs_tol";
}

TEST(test_vincenty_direct, null_lla_b)
{
    kvector_t        lla_a = {0.0};
    precision_type_t range_m(0.0), bearing_rad(0.0);

    int rv = vincenty_direct(&lla_a, range_m, bearing_rad, ABS_TOL, NULL);
    EXPECT_EQ(rv, KL_ERROR_NULL_ARGUMENT) << "Unexpected rv for NULL lla_a";
}

struct VincentyDirectEvalParams
{
    std::string      name;
    kvector_t        lla_a;
    precision_type_t range_m;
    precision_type_t bearing_deg;
    precision_type_t abs_tol;
    kvector_t        lla_b_correct;
    precision_type_t near;
};

class VincentyDirectEvalTest : public testing::TestWithParam<VincentyDirectEvalParams>
{};

TEST_P(VincentyDirectEvalTest, eval)
{
    VincentyDirectEvalParams params = GetParam();

    kvector_t lla_a, lla_b = {0.0};
    kvector_mulf(&params.lla_a, DEG_TO_RAD, &lla_a);
    int rv = vincenty_direct(&lla_a, params.range_m, params.bearing_deg * DEG_TO_RAD, params.abs_tol, &lla_b);

    EXPECT_NEAR(lla_b.x * RAD_TO_DEG, params.lla_b_correct.x, params.near) << "Incorrect latitude";
    EXPECT_NEAR(lla_b.y * RAD_TO_DEG, params.lla_b_correct.y, params.near) << "Incorrect longitude";
    EXPECT_NEAR(lla_b.z, params.lla_b_correct.z, params.near) << "Incorrect altitude";
}

INSTANTIATE_TEST_SUITE_P(
    VincentyDirect,
    VincentyDirectEvalTest,
    testing::Values(
        VincentyDirectEvalParams{
            "Eval same location", // name
            {45.0, 120.0, 0.0},   // lla_a [deg-deg-m]
            0.0,                  // range_m
            0.0,                  // bearing_deg
            ABS_TOL,              // abs_tol
            {45.0, 120.0, 0.0},   // lla_b_correct [deg-deg-m]
            NEAR                  // near
        },
        VincentyDirectEvalParams{
            "Eval input set 1",                          // name
            {45.0, 120.0, 0.0},                          // lla_a [deg-deg-m]
            3000.0,                                      // range_m
            30.0,                                        // bearing_deg
            ABS_TOL,                                     // abs_tol
            {45.02337670419947, 120.0190319660863, 0.0}, // lla_b_correct [deg-deg-m]
            NEAR                                         // near
        },
        VincentyDirectEvalParams{
            "Eval input set 2",                             // name
            {-10.0, -80.0, 0.0},                            // lla_a [deg-deg-m]
            840000.0,                                       // range_m
            -113.0,                                         // bearing_rad
            ABS_TOL,                                        // abs_tol
            {-12.884359520148694, -87.12194168465778, 0.0}, // lla_b_correct [deg-deg-m]
            NEAR                                            // near
        },
        VincentyDirectEvalParams{
            "Eval input set 3",                           // name
            {-60.0, 24.0, 0.0},                           // lla_a [deg-deg-m]
            12000000.0,                                   // range_m
            -45.0,                                        // bearing_deg
            ABS_TOL,                                      // abs_tol
            {37.34976350706342, -33.49808323545486, 0.0}, // lla_b_correct [deg-deg-m]
            NEAR                                          // near
        }),
    [](const ::testing::TestParamInfo<VincentyDirectEvalParams> &info) -> std::string {
        std::string name = info.param.name;
        std::replace(name.begin(), name.end(), ' ', '_');
        return name;
    });
