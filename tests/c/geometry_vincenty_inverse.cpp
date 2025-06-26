#include <gtest/gtest.h>
#include <kinematics-library/errors.h>
#include <kinematics-library/geometry.h>
#include <kinematics-library/kinematic_vector.h>
#include <universal-constants/constants.h>

#define ABS_TOL 1.0E-13
#define NEAR    1.0E-9

TEST(test_vincenty_inverse, null_lla_a)
{
    kvector_t        lla_b = {0.0};
    precision_type_t range_m(0.0), bearing_ab_rad(0.0), bearing_ba_rad(0.0);

    int rv = vincenty_inverse(NULL, &lla_b, ABS_TOL, &range_m, &bearing_ab_rad, &bearing_ba_rad);
    EXPECT_EQ(rv, KL_ERROR_NULL_ARGUMENT) << "Unexpected rv for NULL lla_a";
}

TEST(test_vincenty_inverse, invalid_lla_a_lat)
{
    kvector_t        lla_a = {90.1, 0.0, 0.0};
    kvector_t        lla_b = {0.0};
    precision_type_t range_m(0.0), bearing_ab_rad(0.0), bearing_ba_rad(0.0);

    int rv = vincenty_inverse(&lla_a, &lla_b, ABS_TOL, &range_m, &bearing_ab_rad, &bearing_ba_rad);
    EXPECT_EQ(rv, KL_ERROR_INVALID_ARGUMENT) << "Unexpected rv for invalid lla_a latitude";
}

TEST(test_vincenty_inverse, invalid_lla_a_lon)
{
    kvector_t        lla_a = {0.0, 180.1, 0.0};
    kvector_t        lla_b = {0.0};
    precision_type_t range_m(0.0), bearing_ab_rad(0.0), bearing_ba_rad(0.0);

    int rv = vincenty_inverse(&lla_a, &lla_b, ABS_TOL, &range_m, &bearing_ab_rad, &bearing_ba_rad);
    EXPECT_EQ(rv, KL_ERROR_INVALID_ARGUMENT) << "Unexpected rv for invalid lla_a longitude";
}

TEST(test_vincenty_inverse, null_lla_b)
{
    kvector_t        lla_a = {0.0};
    precision_type_t range_m(0.0), bearing_ab_rad(0.0), bearing_ba_rad(0.0);

    int rv = vincenty_inverse(&lla_a, NULL, ABS_TOL, &range_m, &bearing_ab_rad, &bearing_ba_rad);
    EXPECT_EQ(rv, KL_ERROR_NULL_ARGUMENT) << "Unexpected rv for NULL lla_b";
}

TEST(test_vincenty_inverse, invalid_lla_b_lat)
{
    kvector_t        lla_a = {0.0};
    kvector_t        lla_b = {90.1, 0.0, 0.0};
    precision_type_t range_m(0.0), bearing_ab_rad(0.0), bearing_ba_rad(0.0);

    int rv = vincenty_inverse(&lla_a, &lla_b, ABS_TOL, &range_m, &bearing_ab_rad, &bearing_ba_rad);
    EXPECT_EQ(rv, KL_ERROR_INVALID_ARGUMENT) << "Unexpected rv for invalid lla_b latitude";
}

TEST(test_vincenty_inverse, invalid_lla_b_lon)
{
    kvector_t        lla_a = {0.0};
    kvector_t        lla_b = {0.0, 180.1, 0.0};
    precision_type_t range_m(0.0), bearing_ab_rad(0.0), bearing_ba_rad(0.0);

    int rv = vincenty_inverse(&lla_a, &lla_b, ABS_TOL, &range_m, &bearing_ab_rad, &bearing_ba_rad);
    EXPECT_EQ(rv, KL_ERROR_INVALID_ARGUMENT) << "Unexpected rv for invalid lla_b longitude";
}

TEST(test_vincenty_inverse, invalid_abs_tol)
{
    kvector_t        lla_a = {0.0}, lla_b = {0.0};
    precision_type_t range_m(0.0), bearing_ab_rad(0.0), bearing_ba_rad(0.0);

    int rv = vincenty_inverse(&lla_a, &lla_b, -0.1, &range_m, &bearing_ab_rad, &bearing_ba_rad);
    EXPECT_EQ(rv, KL_ERROR_INVALID_ARGUMENT) << "Unexpected rv for invalid abs_tol";
}

TEST(test_vincenty_inverse, null_range_m)
{
    kvector_t        lla_a = {0.0}, lla_b = {0.0};
    precision_type_t bearing_ab_rad(0.0), bearing_ba_rad(0.0);

    int rv = vincenty_inverse(&lla_a, &lla_b, ABS_TOL, NULL, &bearing_ab_rad, &bearing_ba_rad);
    EXPECT_EQ(rv, KL_ERROR_NULL_ARGUMENT) << "Unexpected rv for NULL range_m";
}

TEST(test_vincenty_inverse, null_bearing_ab_rad)
{
    kvector_t        lla_a = {0.0}, lla_b = {0.0};
    precision_type_t range_m(0.0), bearing_ba_rad(0.0);

    int rv = vincenty_inverse(&lla_a, &lla_b, ABS_TOL, &range_m, NULL, &bearing_ba_rad);
    EXPECT_EQ(rv, KL_ERROR_NULL_ARGUMENT) << "Unexpected rv for NULL bearing_ab_rad";
}

TEST(test_vincenty_inverse, null_bearing_ba_rad)
{
    kvector_t        lla_a = {0.0}, lla_b = {0.0};
    precision_type_t range_m(0.0), bearing_ab_rad(0.0);

    int rv = vincenty_inverse(&lla_a, &lla_b, ABS_TOL, &range_m, &bearing_ab_rad, NULL);
    EXPECT_EQ(rv, KL_ERROR_NULL_ARGUMENT) << "Unexpected rv for NULL bearing_ba_rad";
}

struct VincentyInverseEvalParams
{
    std::string      name;
    kvector_t        lla_a;
    kvector_t        lla_b;
    precision_type_t abs_tol;
    precision_type_t correct_range_m;
    precision_type_t correct_bearing_ab_deg;
    precision_type_t correct_bearing_ba_deg;
    precision_type_t near;
};

class VincentyInverseEvalTest : public testing::TestWithParam<VincentyInverseEvalParams>
{};

TEST_P(VincentyInverseEvalTest, eval)
{
    VincentyInverseEvalParams params = GetParam();

    kvector_t        lla_a, lla_b = {0.0};
    precision_type_t range_m(0.0), bearing_ab_rad(0.0), bearing_ba_rad(0.0);

    kvector_mulf(&params.lla_a, DEG_TO_RAD, &lla_a);
    kvector_mulf(&params.lla_b, DEG_TO_RAD, &lla_b);
    int rv = vincenty_inverse(&lla_a, &lla_b, params.abs_tol, &range_m, &bearing_ab_rad, &bearing_ba_rad);

    EXPECT_NEAR(range_m, params.correct_range_m, params.near) << "Incorrect range";
    EXPECT_NEAR(bearing_ab_rad * RAD_TO_DEG, params.correct_bearing_ab_deg, params.near) << "Incorrect bearing AB";
    EXPECT_NEAR(bearing_ba_rad * RAD_TO_DEG, params.correct_bearing_ba_deg, params.near) << "Incorrect bearing BA";
}

INSTANTIATE_TEST_SUITE_P(
    VincentyInverse,
    VincentyInverseEvalTest,
    testing::Values(
        VincentyInverseEvalParams{
            "Eval same location", // name
            {45.0, 120.0, 0.0},   // lla_a [deg-deg-m]
            {45.0, 120.0, 0.0},   // lla_b [deg-deg-m]
            ABS_TOL,              // abs_tol
            0.0,                  // correct_range_m
            0.0,                  // correct_bearing_ab_deg
            0.0,                  // correct_bearing_ba_deg
            NEAR                  // near
        },
        VincentyInverseEvalParams{
            "Eval anti podal",  // name
            {90.0, 0.0, 0.0},   // lla_a [deg-deg-m]
            {-90.0, 0.0, 0.0},  // lla_b [deg-deg-m]
            ABS_TOL,            // abs_tol
            20003931.457914233, // correct_range_m
            180.0,              // correct_bearing_ab_deg
            0.0,                // correct_bearing_ba_deg
            NEAR                // near
        },
        VincentyInverseEvalParams{
            "Eval both on equator", // name
            {0.0, -10.0, 0.0},      // lla_a [deg-deg-m]
            {0.0, 10.0, 0.0},       // lla_b [deg-deg-m]
            ABS_TOL,                // abs_tol
            2226389.8158654678,     // correct_range_m
            90.0,                   // correct_bearing_ab_deg
            -90.0,                  // correct_bearing_ba_deg
            NEAR                    // near
        },
        VincentyInverseEvalParams{
            "Eval input set 1",  // name
            {-60.0, 24.0, 0.0},  // lla_a [deg-deg-m]
            {-64.0, 26.0, 0.0},  // lla_b [deg-deg-m]
            ABS_TOL,             // abs_tol
            457876.09259014280,  // correct_range_m
            167.65057682653136,  // correct_bearing_ab_deg
            -14.116435240448425, // correct_bearing_ba_deg
            NEAR                 // near
        },
        VincentyInverseEvalParams{
            "Eval input set 2",  // name
            {18.0, -175.0, 0.0}, // lla_a [deg-deg-m]
            {-30.0, 150.0, 0.0}, // lla_b [deg-deg-m]
            ABS_TOL,             // abs_tol
            6503644.0543462737,  // correct_range_m
            -144.26832642124467, // correct_bearing_ab_deg
            39.866234335863595,  // correct_bearing_ba_deg
            NEAR                 // near
        }),
    [](const ::testing::TestParamInfo<VincentyInverseEvalParams> &info) -> std::string {
        std::string name = info.param.name;
        std::replace(name.begin(), name.end(), ' ', '_');
        return name;
    });
