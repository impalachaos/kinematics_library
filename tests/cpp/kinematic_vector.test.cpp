/***********************************************************************************************************************
 * Unit tests for `kinematic_vector.h` and `kinematic_vector.cpp`
 **********************************************************************************************************************/

#include <gtest/gtest.h>
#include <math.h>
#include <kinematics-library/kinematic_vector.hpp>
#include <universal-constants/constants.hpp>

TEST(kinematic_vector_tests, test_constructors)
{
    KinematicVector kv_a;
    EXPECT_DOUBLE_EQ(kv_a.get_x(), 0.0) << "Failed to zero initialize x-component";
    EXPECT_DOUBLE_EQ(kv_a.get_y(), 0.0) << "Failed to zero initialize y-component";
    EXPECT_DOUBLE_EQ(kv_a.get_z(), 0.0) << "Failed to zero initialize z-component";

    const double    x(1.0), y(2.0), z(3.0);
    KinematicVector kv_b(x, y, z);
    EXPECT_DOUBLE_EQ(kv_b.get_x(), x) << "Failed to initialize x-component";
    EXPECT_DOUBLE_EQ(kv_b.get_y(), y) << "Failed to initialize y-component";
    EXPECT_DOUBLE_EQ(kv_b.get_z(), z) << "Failed to initialize z-component";

    KinematicVector kv_c(kv_b);
    EXPECT_DOUBLE_EQ(kv_c.get_x(), kv_b.get_x()) << "Failed to initialize x-component";
    EXPECT_DOUBLE_EQ(kv_c.get_y(), kv_b.get_y()) << "Failed to initialize y-component";
    EXPECT_DOUBLE_EQ(kv_c.get_z(), kv_b.get_z()) << "Failed to initialize z-component";
}

TEST(kinematic_vector_tests, test_angle_between)
{
    // Intermediate variables used for debugging purposes
    KinematicVector kv_a(1.0, 1.0, 1.0);
    KinematicVector kv_b(1.0, 0.0, 0.0);
    double          expect_angle(0.0);
    double          actual_angle_a(kv_a.angle_between(kv_a));
    double          actual_angle_b(0.0);

    // Identical vector, 0 angle
    EXPECT_NEAR(actual_angle_a, expect_angle, 1.0E-7) << "Incorrect angle";

    // Simple 2d case, 90 degree
    kv_a.set(0.0, 1.0, 0.0);
    expect_angle   = math::pi / 2.0;
    actual_angle_a = kv_a.angle_between(kv_b);
    actual_angle_b = kv_b.angle_between(kv_a);
    EXPECT_DOUBLE_EQ(actual_angle_a, expect_angle) << "Incorrect angle";
    EXPECT_DOUBLE_EQ(actual_angle_b, expect_angle) << "Incorrect angle";

    // Complex 3d case
    kv_a.set(1.0, -4.0, 0.5);
    kv_b.set(-1.2, 2.0, -3.0);
    expect_angle   = 132.68462504216254 * math::pi / 180.0;
    actual_angle_a = kv_a.angle_between(kv_b);
    actual_angle_b = kv_b.angle_between(kv_a);
    EXPECT_DOUBLE_EQ(actual_angle_a, expect_angle) << "Incorrect angle";
    EXPECT_DOUBLE_EQ(actual_angle_b, expect_angle) << "Incorrect angle";

    // 0 magnitude, one vector
    kv_a.set(0.0, 0.0, 0.0);
    actual_angle_a = kv_a.angle_between(kv_b);
    actual_angle_b = kv_b.angle_between(kv_a);
    // Test fails if exception is thrown, no assert necessary

    // 0 magnitude, both vectors
    kv_b.set(0.0, 0.0, 0.0);
    actual_angle_a = kv_a.angle_between(kv_b);
    actual_angle_b = kv_b.angle_between(kv_a);
    // Test fails if exception is thrown, no assert necessary
}

TEST(kinematic_vector_tests, test_azmiuth_angle)
{
    double          az_rad(0.0);
    const double    sqrt_3(sqrt(3.0)), one(1.0), z(10.0);
    KinematicVector kv(sqrt_3, one, z);

    // Check azimuth for quadrant 1
    az_rad = kv.azimuth_angle();
    EXPECT_DOUBLE_EQ(az_rad, 30.0 * conversions::deg_to_rad) << "Incorrect azimuth angle in quadrant 1";

    // Check azimuth for quadrant 2
    kv.set_x(-sqrt_3);
    kv.set_y(one);
    az_rad = kv.azimuth_angle();
    EXPECT_DOUBLE_EQ(az_rad, 150.0 * conversions::deg_to_rad) << "Incorrect azimuth angle in quadrant 2";

    // Check azimuth for quadrant 3
    kv.set_x(-one);
    kv.set_y(-sqrt_3);
    az_rad = kv.azimuth_angle();
    EXPECT_DOUBLE_EQ(az_rad, -120.0 * conversions::deg_to_rad) << "Incorrect azimuth angle in quadrant 3";

    // Check azimuth for quadrant 4
    kv.set_x(one);
    kv.set_y(-sqrt_3);
    az_rad = kv.azimuth_angle();
    EXPECT_DOUBLE_EQ(az_rad, -60.0 * conversions::deg_to_rad) << "Incorrect azimuth angle in quadrant 4";
}

TEST(kinematic_vector_tests, test_cross_product)
{
    KinematicVector kv_a(2.0, -3.0, 1.0);
    KinematicVector kv_b(4.0, -1.0, 5.0);

    KinematicVector result = kv_a.cross(kv_b);
    EXPECT_DOUBLE_EQ(result.get_x(), -14.0) << "Incorrect cross product x-component";
    EXPECT_DOUBLE_EQ(result.get_y(), -6.0) << "Incorrect cross product y-component";
    EXPECT_DOUBLE_EQ(result.get_z(), 10.0) << "Incorrect cross product z-component";

    result = kv_b.cross(kv_a);
    EXPECT_DOUBLE_EQ(result.get_x(), 14.0) << "Incorrect cross product x-component";
    EXPECT_DOUBLE_EQ(result.get_y(), 6.0) << "Incorrect cross product y-component";
    EXPECT_DOUBLE_EQ(result.get_z(), -10.0) << "Incorrect cross product z-component";
}

TEST(kinematic_vector_tests, test_dot_product)
{
    KinematicVector kv_a(2.0, -7.0, -1.0);
    KinematicVector kv_b(8.0, 2.0, -4.0);

    EXPECT_DOUBLE_EQ(kv_a.dot(kv_b), 6.0) << "Incorrect dot product";
    EXPECT_DOUBLE_EQ(kv_b.dot(kv_a), 6.0) << "Incorrect dot product";
}

TEST(kinematic_vector_tests, test_elevation_angle)
{
    double          el_rad(0.0);
    KinematicVector kv(3.0, 4.0, 5.0);

    // Check elevation above the xy-plane
    el_rad = kv.elevation_angle();
    EXPECT_DOUBLE_EQ(el_rad, -45.0 * conversions::deg_to_rad) << "Incorrect elevation angle above the xy-plane";

    // Check elevation on the xy-plane
    kv.set_z(0.0);
    el_rad = kv.elevation_angle();
    EXPECT_DOUBLE_EQ(el_rad, 0.0 * conversions::deg_to_rad) << "Incorrect elevation angle on the xy-plane";

    // Check elevation below the xy-plane
    kv.set_z(-5.0);
    el_rad = kv.elevation_angle();
    EXPECT_DOUBLE_EQ(el_rad, 45.0 * conversions::deg_to_rad) << "Incorrect elevation angle below the xy-plane";
}

TEST(kinematic_vector_tests, test_operator_add)
{
    const double    x_a(1.0), y_a(2.0), z_a(3.0);
    KinematicVector kv_a(x_a, y_a, z_a);

    const double    x_b(5.0), y_b(6.0), z_b(7.0);
    KinematicVector kv_b(x_b, y_b, z_b);

    KinematicVector kv_c = kv_a + kv_b;
    EXPECT_DOUBLE_EQ(kv_c.get_x(), x_a + x_b) << "Incorrect vector-vector addition x-component";
    EXPECT_DOUBLE_EQ(kv_c.get_y(), y_a + y_b) << "Incorrect vector-vector addition y-component";
    EXPECT_DOUBLE_EQ(kv_c.get_z(), z_a + z_b) << "Incorrect vector-vector addition z-component";

    const double    scalar(10.0);
    KinematicVector kv_d = kv_a + scalar;
    EXPECT_DOUBLE_EQ(kv_d.get_x(), x_a + scalar) << "Incorrect vector-scalar addition x-component";
    EXPECT_DOUBLE_EQ(kv_d.get_y(), y_a + scalar) << "Incorrect vector-scalar addition y-component";
    EXPECT_DOUBLE_EQ(kv_d.get_z(), z_a + scalar) << "Incorrect vector-scalar addition z-component";

    KinematicVector kv_e = scalar + kv_a;
    EXPECT_DOUBLE_EQ(kv_e.get_x(), scalar + x_a) << "Incorrect scalar-vector addition x-component";
    EXPECT_DOUBLE_EQ(kv_e.get_y(), scalar + y_a) << "Incorrect scalar-vector addition y-component";
    EXPECT_DOUBLE_EQ(kv_e.get_z(), scalar + z_a) << "Incorrect scalar-vector addition z-component";

    KinematicVector kv_f(kv_a);
    kv_f += kv_b;
    EXPECT_DOUBLE_EQ(kv_f.get_x(), x_a + x_b) << "Incorrect vector-vector in-place addition x-component";
    EXPECT_DOUBLE_EQ(kv_f.get_y(), y_a + y_b) << "Incorrect vector-vector in-place addition y-component";
    EXPECT_DOUBLE_EQ(kv_f.get_z(), z_a + z_b) << "Incorrect vector-vector in-place addition z-component";
}

TEST(kinematic_vector_tests, test_operator_subtract)
{
    const double    x_a(1.0), y_a(2.0), z_a(3.0);
    KinematicVector kv_a(x_a, y_a, z_a);

    const double    x_b(5.0), y_b(6.0), z_b(7.0);
    KinematicVector kv_b(x_b, y_b, z_b);

    KinematicVector kv_c = kv_a - kv_b;
    EXPECT_DOUBLE_EQ(kv_c.get_x(), x_a - x_b) << "Incorrect vector-vector subtraction x-component";
    EXPECT_DOUBLE_EQ(kv_c.get_y(), y_a - y_b) << "Incorrect vector-vector subtraction y-component";
    EXPECT_DOUBLE_EQ(kv_c.get_z(), z_a - z_b) << "Incorrect vector-vector subtraction z-component";

    const double    scalar(10.0);
    KinematicVector kv_d = kv_a - scalar;
    EXPECT_DOUBLE_EQ(kv_d.get_x(), x_a - scalar) << "Incorrect vector-scalar subtraction x-component";
    EXPECT_DOUBLE_EQ(kv_d.get_y(), y_a - scalar) << "Incorrect vector-scalar subtraction y-component";
    EXPECT_DOUBLE_EQ(kv_d.get_z(), z_a - scalar) << "Incorrect vector-scalar subtraction z-component";

    KinematicVector kv_e = scalar - kv_a;
    EXPECT_DOUBLE_EQ(kv_e.get_x(), scalar - x_a) << "Incorrect scalar-vector subtraction x-component";
    EXPECT_DOUBLE_EQ(kv_e.get_y(), scalar - y_a) << "Incorrect scalar-vector subtraction y-component";
    EXPECT_DOUBLE_EQ(kv_e.get_z(), scalar - z_a) << "Incorrect scalar-vector subtraction z-component";

    KinematicVector kv_f(kv_a);
    kv_f -= kv_b;
    EXPECT_DOUBLE_EQ(kv_f.get_x(), x_a - x_b) << "Incorrect vector-vector in-place addition x-component";
    EXPECT_DOUBLE_EQ(kv_f.get_y(), y_a - y_b) << "Incorrect vector-vector in-place addition y-component";
    EXPECT_DOUBLE_EQ(kv_f.get_z(), z_a - z_b) << "Incorrect vector-vector in-place addition z-component";
}

TEST(kinematic_vector_tests, test_operator_multiply)
{
    const double    x_a(1.0), y_a(2.0), z_a(3.0);
    KinematicVector kv_a(x_a, y_a, z_a);

    const double    scalar(10.0);
    KinematicVector kv_b = kv_a * scalar;
    EXPECT_DOUBLE_EQ(kv_b.get_x(), x_a * scalar) << "Incorrect vector-scalar multiply x-component";
    EXPECT_DOUBLE_EQ(kv_b.get_y(), y_a * scalar) << "Incorrect vector-scalar multiply y-component";
    EXPECT_DOUBLE_EQ(kv_b.get_z(), z_a * scalar) << "Incorrect vector-scalar multiply z-component";

    KinematicVector kv_c = scalar * kv_a;
    EXPECT_DOUBLE_EQ(kv_c.get_x(), scalar * x_a) << "Incorrect scalar-vector multiply x-component";
    EXPECT_DOUBLE_EQ(kv_c.get_y(), scalar * y_a) << "Incorrect scalar-vector multiply y-component";
    EXPECT_DOUBLE_EQ(kv_c.get_z(), scalar * z_a) << "Incorrect scalar-vector multiply z-component";
}

TEST(kinematic_vector_tests, test_operator_divide)
{
    const double    x_a(1.0), y_a(2.0), z_a(3.0);
    KinematicVector kv_a(x_a, y_a, z_a);

    const double    scalar(10.0);
    KinematicVector kv_b = kv_a / scalar;
    EXPECT_DOUBLE_EQ(kv_b.get_x(), x_a / scalar) << "Incorrect vector-scalar divide x-component";
    EXPECT_DOUBLE_EQ(kv_b.get_y(), y_a / scalar) << "Incorrect vector-scalar divide y-component";
    EXPECT_DOUBLE_EQ(kv_b.get_z(), z_a / scalar) << "Incorrect vector-scalar divide z-component";

    KinematicVector kv_c = scalar / kv_a;
    EXPECT_DOUBLE_EQ(kv_c.get_x(), scalar / x_a) << "Incorrect scalar-vector divide x-component";
    EXPECT_DOUBLE_EQ(kv_c.get_y(), scalar / y_a) << "Incorrect scalar-vector divide y-component";
    EXPECT_DOUBLE_EQ(kv_c.get_z(), scalar / z_a) << "Incorrect scalar-vector divide z-component";
}

TEST(kinematic_vector_tests, test_set_and_get_x)
{
    KinematicVector kv;

    const double x(5.0);
    kv.set_x(x);
    EXPECT_DOUBLE_EQ(kv.get_x(), x) << "Incorrect x-component";
}

TEST(kinematic_vector_tests, test_set_and_get_y)
{
    KinematicVector kv;

    const double y(5.0);
    kv.set_y(y);
    EXPECT_DOUBLE_EQ(kv.get_y(), y) << "Incorrect y-component";
}

TEST(kinematic_vector_tests, test_set_and_get_z)
{
    KinematicVector kv;

    const double z(5.0);
    kv.set_z(z);
    EXPECT_DOUBLE_EQ(kv.get_z(), z) << "Incorrect z-component";
}

TEST(kinematic_vector_tests, test_magnitude)
{
    const double    x(6.0), y(4.0), z(-12.0);
    const double    mag = sqrt(x * x + y * y + z * z);
    KinematicVector kv(x, y, z);
    EXPECT_DOUBLE_EQ(kv.magnitude(), mag) << "Incorrect vector magnitude";
}

TEST(kinematic_vector_tests, test_polar_angle)
{
    double          polar_rad(0.0);
    KinematicVector kv(0.0, 0.0, 5.0);

    // Check polar angle along z-axis
    polar_rad = kv.polar_angle();
    EXPECT_NEAR(polar_rad, 0.0, 1.0E-7) << "Incorrect polar angle along z-axis";

    // Check polar angle above xy-plane but off z-axis
    kv.set_x(5.0);
    polar_rad = kv.polar_angle();
    EXPECT_DOUBLE_EQ(polar_rad, 45.0 * conversions::deg_to_rad) << "Incorrect polar angle above xy-plane";

    // Check polar angle in xy-plane
    kv.set_z(0.0);
    polar_rad = kv.polar_angle();
    EXPECT_DOUBLE_EQ(polar_rad, 90.0 * conversions::deg_to_rad) << "Incorrect polar angle in xy-plane";

    // Check polar angle below xy-plane but off z-axis
    kv.set_z(-5.0);
    polar_rad = kv.polar_angle();
    EXPECT_DOUBLE_EQ(polar_rad, 135.0 * conversions::deg_to_rad) << "Incorrect polar angle below xy-plane";

    // Check polar angle along negative z-axis
    kv.set_x(0.0);
    polar_rad = kv.polar_angle();
    EXPECT_NEAR(polar_rad, 180.0 * conversions::deg_to_rad, 1.0E-7) << "Incorrect polar angle along negative z-axis";
}

TEST(kinematic_vector_tests, test_set)
{
    const double    x(1.0), y(2.0), z(3.0);
    KinematicVector kv_a;

    kv_a.set(x, y, z);
    EXPECT_DOUBLE_EQ(kv_a.get_x(), x) << "Failed to correctly set x-component";
    EXPECT_DOUBLE_EQ(kv_a.get_y(), y) << "Failed to correctly set y-component";
    EXPECT_DOUBLE_EQ(kv_a.get_z(), z) << "Failed to correctly set z-component";

    KinematicVector kv_b;
    kv_b.set(kv_a);
    EXPECT_DOUBLE_EQ(kv_b.get_x(), kv_a.get_x()) << "Failed to initialize x-component";
    EXPECT_DOUBLE_EQ(kv_b.get_y(), kv_a.get_y()) << "Failed to initialize y-component";
    EXPECT_DOUBLE_EQ(kv_b.get_z(), kv_a.get_z()) << "Failed to initialize z-component";
}

TEST(kinematic_vector_tests, test_to_unit)
{
    const double    x(6.0), y(4.0), z(-12.0);
    const double    mag = sqrt(x * x + y * y + z * z);
    KinematicVector kv(x, y, z);

    kv.to_unit();
    EXPECT_DOUBLE_EQ(kv.get_x(), x / mag) << "Incorrect unit vector x-component";
    EXPECT_DOUBLE_EQ(kv.get_y(), y / mag) << "Incorrect unit vector y-component";
    EXPECT_DOUBLE_EQ(kv.get_z(), z / mag) << "Incorrect unit vector z-component";
}

TEST(kinematic_vector_tests, test_unit)
{
    const double    x(6.0), y(4.0), z(-12.0);
    const double    mag = sqrt(x * x + y * y + z * z);
    KinematicVector kv(x, y, z);

    KinematicVector uv = kv.unit();
    EXPECT_DOUBLE_EQ(uv.get_x(), x / mag) << "Incorrect unit vector x-component";
    EXPECT_DOUBLE_EQ(uv.get_y(), y / mag) << "Incorrect unit vector y-component";
    EXPECT_DOUBLE_EQ(uv.get_z(), z / mag) << "Incorrect unit vector z-component";
}

TEST(kinematic_vector_tests, test_zero)
{
    const double    x(6.0), y(4.0), z(-12.0);
    KinematicVector kv(x, y, z);

    kv.zero();
    EXPECT_DOUBLE_EQ(kv.get_x(), 0.0) << "Incorrect zero-ed x-component";
    EXPECT_DOUBLE_EQ(kv.get_y(), 0.0) << "Incorrect zero-ed y-component";
    EXPECT_DOUBLE_EQ(kv.get_z(), 0.0) << "Incorrect zero-ed z-component";
}
