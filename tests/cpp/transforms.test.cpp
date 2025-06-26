/***********************************************************************************************************************
 * Unit tests for `transforms.h` and `transforms.cpp`
 **********************************************************************************************************************/

#include <gtest/gtest.h>
#include <kinematics-library/transforms.hpp>
#include <universal-constants/constants.hpp>

TEST(transforms_tests, test_aer_to_ned)
{
    KinematicVector aer(30 * math::pi / 180, 40 * math::pi / 180, -70);
    KinematicVector ned = aer_to_ned(aer);
    double          n_m(-46.43897628), e_m(-26.81155573), d_m(44.99513264);    // Matlab Result
    double          n_py(-46.43897637), e_py(-26.81155551), d_py(44.99513268); // Pybasic Result

    EXPECT_NEAR(ned.get_x(), n_m, 1.0E-6) << "Incorrect n in aer-to-ned conversion";
    EXPECT_NEAR(ned.get_x(), n_py, 1.0E-6) << "Incorrect n in aer-to-ned conversion";
    EXPECT_NEAR(ned.get_y(), e_m, 1.0E-6) << "Incorrect e in aer-to-ned conversion";
    EXPECT_NEAR(ned.get_y(), e_py, 1.0E-6) << "Incorrect e in aer-to-ned conversion";
    EXPECT_NEAR(ned.get_z(), d_m, 1.0E-6) << "Incorrect d in aer-to-ned conversion";
    EXPECT_NEAR(ned.get_z(), d_py, 1.0E-6) << "Incorrect d in aer-to-ned conversion";
}

TEST(transforms_tests, test_ecef_to_lla)
{
    KinematicVector ecef(4040000.0, 4813000.0, 1100000.0);
    KinematicVector lla_test = ecef_to_lla(ecef);
    KinematicVector lla_truth(0.174440593201424, 0.872492313234789, 1888.18501992803);
    EXPECT_NEAR(lla_test.get_x(), lla_truth.get_x(), 1.0E-12) << "Incorrect latitude in ecef-to-lla conversion";
    EXPECT_NEAR(lla_test.get_y(), lla_truth.get_y(), 1.0E-12) << "Incorrect longitude in ecef-to-lla conversion";
    EXPECT_NEAR(lla_test.get_z(), lla_truth.get_z(), 1.0E-4) << "Incorrect altitude in ecef-to-lla conversion";
}

TEST(transforms_tests, test_ecef_to_ned)
{
    KinematicVector ecef(-1465000.0, 2880000.0, 5483500.0);
    KinematicVector lla_ref(2.1, -1.1, 1250.0);
    KinematicVector ned_test = ecef_to_ned(ecef, lla_ref);
    KinematicVector ned_truth(2220.90854391709, 738.047215659648, -1205.58533167388);
    EXPECT_NEAR(ned_test.get_x(), ned_truth.get_x(), 1.0E-8) << "Incorrect n in ecef-to-ned conversion";
    EXPECT_NEAR(ned_test.get_y(), ned_truth.get_y(), 1.0E-8) << "Incorrect e in ecef-to-ned conversion";
    EXPECT_NEAR(ned_test.get_z(), ned_truth.get_z(), 1.0E-8) << "Incorrect d in ecef-to-ned conversion";
}

TEST(transforms_tests, test_lla_to_ecef)
{
    KinematicVector lla(-0.8, 1.7, 4750.0);
    KinematicVector ecef_test = lla_to_ecef(lla);
    KinematicVector ecef_truth(-573960.23482409, 4417543.57131158, -4556021.90733023);
    EXPECT_NEAR(ecef_test.get_x(), ecef_truth.get_x(), 1.0E-8) << "Incorrect x in lla-to-ecef conversion";
    EXPECT_NEAR(ecef_test.get_y(), ecef_truth.get_y(), 1.0E-8) << "Incorrect y in lla-to-ecef conversion";
    EXPECT_NEAR(ecef_test.get_z(), ecef_truth.get_z(), 1.0E-8) << "Incorrect z in lla-to-ecef conversion";
}

TEST(transforms_tests, test_lla_to_ned)
{
    KinematicVector lla(-0.707, 0.496, 2400.0);
    KinematicVector lla_ref(-0.7, 0.5, 6000.0);
    KinematicVector ned_test = lla_to_ned(lla, lla_ref);
    KinematicVector ned_truth(-44576.5574572138, -19432.24609638221, 3785.6580070712043);
    EXPECT_NEAR(ned_test.get_x(), ned_truth.get_x(), 1.0E-14) << "Incorrect n in lla-to-ned conversion";
    EXPECT_NEAR(ned_test.get_y(), ned_truth.get_y(), 1.0E-14) << "Incorrect e in lla-to-ned conversion";
    EXPECT_NEAR(ned_test.get_z(), ned_truth.get_z(), 1.0E-14) << "Incorrect d in lla-to-ned conversion";
}

TEST(transforms_tests, test_ned_to_aer)
{
    KinematicVector ned(-46.43897628, -26.81155573, 44.99513264);
    KinematicVector aer = ned_to_aer(ned);
    double          az_m(3.665191434 - (2 * math::pi)), el_m(-0.6981317), range_m(70.0); // Matlab Result
    double          az_py(-2.61799388), el_py(-0.6981317), range_py(70.0);               // Pybasic Result

    EXPECT_NEAR(aer.get_x(), az_m, 1.0E-6) << "Incorrect n in ned-to-aer conversion";
    EXPECT_NEAR(aer.get_x(), az_py, 1.0E-6) << "Incorrect n in ned-to-aer conversion";
    EXPECT_NEAR(aer.get_y(), el_m, 1.0E-6) << "Incorrect e in ned-to-aer conversion";
    EXPECT_NEAR(aer.get_y(), el_py, 1.0E-6) << "Incorrect e in ned-to-aer conversion";
    EXPECT_NEAR(aer.get_z(), range_m, 1.0E-6) << "Incorrect d in ned-to-aer conversion";
    EXPECT_NEAR(aer.get_z(), range_py, 1.0E-6) << "Incorrect d in ned-to-aer conversion";
}

TEST(transforms_tests, test_ned_to_ecef)
{
    KinematicVector ned(-6500.0, -4800.0, 2400.0);
    KinematicVector lla_ref(-2.9, -0.5, 6000.0);
    KinematicVector ecef_test = ned_to_ecef(ned, lla_ref);
    KinematicVector ecef_truth(-5442559.838214509, 2967814.4198892456, -1510590.1708055406);
    EXPECT_NEAR(ecef_test.get_x(), ecef_truth.get_x(), 1.0E-14) << "Incorrect x in ned-to-ecef conversion";
    EXPECT_NEAR(ecef_test.get_y(), ecef_truth.get_y(), 1.0E-14) << "Incorrect y in ned-to-ecef conversion";
    EXPECT_NEAR(ecef_test.get_z(), ecef_truth.get_z(), 1.0E-14) << "Incorrect z in ned-to-ecef conversion";
}

TEST(transforms_tests, test_ned_to_lla)
{
    KinematicVector ned(12300.0, -8000.0, 6500.0);
    KinematicVector lla_ref(0.1, 2.5, 9200.0);
    KinematicVector lla_test = ned_to_lla(ned, lla_ref);
    KinematicVector lla_truth(0.10194035135339555, 2.49873974993884, 2716.948501356757);
    EXPECT_NEAR(lla_test.get_x(), lla_truth.get_x(), 1.0E-14) << "Incorrect latitude in ned-to-lla conversion";
    EXPECT_NEAR(lla_test.get_y(), lla_truth.get_y(), 1.0E-14) << "Incorrect longitude in ned-to-lla conversion";
    EXPECT_NEAR(lla_test.get_z(), lla_truth.get_z(), 1.0E-4) << "Incorrect altitude in ned-to-lla conversion";
}
