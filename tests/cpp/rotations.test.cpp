/***********************************************************************************************************************
 * Unit tests for `rotations.h` and `rotations.cpp`
 **********************************************************************************************************************/

#include <gtest/gtest.h>
#include <kinematics-library/rotations.hpp>

namespace constants
{
    static const double a_tol(1.0E-14);
}

void check_dcm_near(const DCM &dcm_a, const DCM &dcm_b, const double &a_tol)
{
    EXPECT_NEAR(dcm_a.get_e00(), dcm_b.get_e00(), a_tol) << "DCM elements 00 not within tolerance";
    EXPECT_NEAR(dcm_a.get_e01(), dcm_b.get_e01(), a_tol) << "DCM elements 01 not within tolerance";
    EXPECT_NEAR(dcm_a.get_e02(), dcm_b.get_e02(), a_tol) << "DCM elements 02 not within tolerance";
    EXPECT_NEAR(dcm_a.get_e10(), dcm_b.get_e10(), a_tol) << "DCM elements 10 not within tolerance";
    EXPECT_NEAR(dcm_a.get_e11(), dcm_b.get_e11(), a_tol) << "DCM elements 11 not within tolerance";
    EXPECT_NEAR(dcm_a.get_e12(), dcm_b.get_e12(), a_tol) << "DCM elements 12 not within tolerance";
    EXPECT_NEAR(dcm_a.get_e20(), dcm_b.get_e20(), a_tol) << "DCM elements 20 not within tolerance";
    EXPECT_NEAR(dcm_a.get_e21(), dcm_b.get_e21(), a_tol) << "DCM elements 21 not within tolerance";
    EXPECT_NEAR(dcm_a.get_e22(), dcm_b.get_e22(), a_tol) << "DCM elements 22 not within tolerance";
}

void check_quaternion_near(const Quaternion &q_a, const Quaternion &q_b, const double &a_tol)
{
    EXPECT_NEAR(q_a.get_a(), q_b.get_a(), a_tol) << "Quaternion components a not within tolerance";
    EXPECT_NEAR(q_a.get_b(), q_b.get_b(), a_tol) << "Quaternion components b not within tolerance";
    EXPECT_NEAR(q_a.get_c(), q_b.get_c(), a_tol) << "Quaternion components c not within tolerance";
    EXPECT_NEAR(q_a.get_d(), q_b.get_d(), a_tol) << "Quaternion components d not within tolerance";
}

void check_kv_near(const KinematicVector &kv_a, const KinematicVector &kv_b, const double &a_tol)
{
    EXPECT_NEAR(kv_a.get_x(), kv_b.get_x(), a_tol) << "Vector components x not within tolerance";
    EXPECT_NEAR(kv_a.get_y(), kv_b.get_y(), a_tol) << "Vector components y not within tolerance";
    EXPECT_NEAR(kv_a.get_z(), kv_b.get_z(), a_tol) << "Vector components z not within tolerance";
}

void check_dcm_transform(const DCM &dcm, const KinematicVector &kv, const KinematicVector &kv_truth)
{
    KinematicVector kv_test = dcm * kv;
    check_kv_near(kv_truth, kv_test, constants::a_tol);

    kv_test.zero();
    dcm.transform(kv, kv_test);
    check_kv_near(kv_truth, kv_test, constants::a_tol);

    kv_test.set(kv);
    dcm.transform(kv_test);
    check_kv_near(kv_truth, kv_test, constants::a_tol);
}

void check_dcm_rotate(const DCM &dcm, const KinematicVector &kv, const KinematicVector &kv_truth)
{
    KinematicVector kv_test = kv * dcm;
    check_kv_near(kv_truth, kv_test, constants::a_tol);

    kv_test.zero();
    dcm.rotate(kv, kv_test);
    check_kv_near(kv_truth, kv_test, constants::a_tol);

    kv_test.set(kv);
    dcm.rotate(kv_test);
    check_kv_near(kv_truth, kv_test, constants::a_tol);
}

void check_q_transform(const Quaternion &q, const KinematicVector &kv, const KinematicVector &kv_truth)
{
    KinematicVector kv_test;
    q.transform(kv, kv_test);
    check_kv_near(kv_truth, kv_test, constants::a_tol);

    kv_test.set(kv);
    q.transform(kv_test);
    check_kv_near(kv_truth, kv_test, constants::a_tol);
}

void check_q_rotate(const Quaternion &q, const KinematicVector &kv, const KinematicVector &kv_truth)
{
    KinematicVector kv_test;
    q.rotate(kv, kv_test);
    check_kv_near(kv_truth, kv_test, constants::a_tol);

    kv_test.set(kv);
    q.rotate(kv_test);
    check_kv_near(kv_truth, kv_test, constants::a_tol);
}

TEST(rotations_tests, test_dcm_default_const)
{
    DCM dcm;
    EXPECT_DOUBLE_EQ(dcm.get_e00(), 1.0) << "Incorrect default element 00";
    EXPECT_DOUBLE_EQ(dcm.get_e01(), 0.0) << "Incorrect default element 01";
    EXPECT_DOUBLE_EQ(dcm.get_e02(), 0.0) << "Incorrect default element 02";
    EXPECT_DOUBLE_EQ(dcm.get_e10(), 0.0) << "Incorrect default element 10";
    EXPECT_DOUBLE_EQ(dcm.get_e11(), 1.0) << "Incorrect default element 11";
    EXPECT_DOUBLE_EQ(dcm.get_e12(), 0.0) << "Incorrect default element 12";
    EXPECT_DOUBLE_EQ(dcm.get_e20(), 0.0) << "Incorrect default element 20";
    EXPECT_DOUBLE_EQ(dcm.get_e21(), 0.0) << "Incorrect default element 21";
    EXPECT_DOUBLE_EQ(dcm.get_e22(), 1.0) << "Incorrect default element 22";
}

TEST(rotations_tests, test_dcm_operator_eq_and_ne)
{
    const double a1_rad(3.03), a2_rad(-2.16), a3_rad(2.23);

    DCM dcm(a1_rad, a2_rad, a3_rad, RotationSequence::BODY_XYX);
    DCM dcm_eq(a1_rad, a2_rad, a3_rad, RotationSequence::BODY_XYX);
    DCM dcm_ne(a1_rad, a2_rad, a3_rad, RotationSequence::BODY_ZYX);

    EXPECT_TRUE(dcm == dcm_eq) << "Equality operator did not return true for equal matrices";
    EXPECT_FALSE(dcm == dcm_ne) << "Equality operator did not return false for not equal matrices";

    EXPECT_TRUE(dcm != dcm_ne) << "Inequality operator did not return true for not equal matrices";
    EXPECT_FALSE(dcm != dcm_eq) << "Inequality operator did not return false for equal matrices";
}

TEST(rotations_tests, test_dcm_transpose)
{
    const double           a1_rad(3.03), a2_rad(-2.16), a3_rad(2.23);
    const RotationSequence seq(RotationSequence::BODY_ZYX);

    DCM dcm(a1_rad, a2_rad, a3_rad, seq);
    DCM dcm_a = dcm.transpose();
    EXPECT_DOUBLE_EQ(dcm.get_e00(), dcm_a.get_e00()) << "Incorrect transpose element 00";
    EXPECT_DOUBLE_EQ(dcm.get_e01(), dcm_a.get_e10()) << "Incorrect transpose element 01";
    EXPECT_DOUBLE_EQ(dcm.get_e02(), dcm_a.get_e20()) << "Incorrect transpose element 02";
    EXPECT_DOUBLE_EQ(dcm.get_e10(), dcm_a.get_e01()) << "Incorrect transpose element 10";
    EXPECT_DOUBLE_EQ(dcm.get_e11(), dcm_a.get_e11()) << "Incorrect transpose element 11";
    EXPECT_DOUBLE_EQ(dcm.get_e12(), dcm_a.get_e21()) << "Incorrect transpose element 12";
    EXPECT_DOUBLE_EQ(dcm.get_e20(), dcm_a.get_e02()) << "Incorrect transpose element 20";
    EXPECT_DOUBLE_EQ(dcm.get_e21(), dcm_a.get_e12()) << "Incorrect transpose element 21";
    EXPECT_DOUBLE_EQ(dcm.get_e22(), dcm_a.get_e22()) << "Incorrect transpose element 22";

    DCM dcm_b;
    dcm.transpose(dcm_b);
    EXPECT_DOUBLE_EQ(dcm.get_e00(), dcm_b.get_e00()) << "Incorrect transpose element 00";
    EXPECT_DOUBLE_EQ(dcm.get_e01(), dcm_b.get_e10()) << "Incorrect transpose element 01";
    EXPECT_DOUBLE_EQ(dcm.get_e02(), dcm_b.get_e20()) << "Incorrect transpose element 02";
    EXPECT_DOUBLE_EQ(dcm.get_e10(), dcm_b.get_e01()) << "Incorrect transpose element 10";
    EXPECT_DOUBLE_EQ(dcm.get_e11(), dcm_b.get_e11()) << "Incorrect transpose element 11";
    EXPECT_DOUBLE_EQ(dcm.get_e12(), dcm_b.get_e21()) << "Incorrect transpose element 12";
    EXPECT_DOUBLE_EQ(dcm.get_e20(), dcm_b.get_e02()) << "Incorrect transpose element 20";
    EXPECT_DOUBLE_EQ(dcm.get_e21(), dcm_b.get_e12()) << "Incorrect transpose element 21";
    EXPECT_DOUBLE_EQ(dcm.get_e22(), dcm_b.get_e22()) << "Incorrect transpose element 22";
}

TEST(rotations_tests, test_q_default_const)
{
    Quaternion q;
    EXPECT_DOUBLE_EQ(q.get_a(), 1.0) << "Incorrect default quaternion a";
    EXPECT_DOUBLE_EQ(q.get_b(), 0.0) << "Incorrect default quaternion b";
    EXPECT_DOUBLE_EQ(q.get_c(), 0.0) << "Incorrect default quaternion c";
    EXPECT_DOUBLE_EQ(q.get_d(), 0.0) << "Incorrect default quaternion d";
}

TEST(rotations_tests, test_q_angle)
{
    Quaternion q(-1.28, 1.54, -1.95, RotationSequence::BODY_XYX);

    EXPECT_DOUBLE_EQ(q.angle(), 3.0781340962868073) << "Incorrect quaternion angle";
}

TEST(rotations_tests, test_q_axis)
{
    Quaternion      q(-1.28, 1.54, -1.95, RotationSequence::BODY_XYX);
    KinematicVector axis;
    q.axis(axis);

    EXPECT_DOUBLE_EQ(axis.get_x(), 0.717570573501375) << "Incorrect quaternion axis x";
    EXPECT_DOUBLE_EQ(axis.get_y(), -0.657768371689232) << "Incorrect quaternion axis y";
    EXPECT_DOUBLE_EQ(axis.get_z(), -0.2289830588716213) << "Incorrect quaternion axis z";
}

TEST(rotations_tests, test_q_conjugate)
{
    Quaternion q(-2.17, 0.92, 3.02, RotationSequence::BODY_YXZ);

    Quaternion q_conj_a = q.conjugate();
    EXPECT_DOUBLE_EQ(q_conj_a.get_a(), q.get_a()) << "Incorrect conjugate quaternion a";
    EXPECT_DOUBLE_EQ(q_conj_a.get_b(), -q.get_b()) << "Incorrect conjugate quaternion b";
    EXPECT_DOUBLE_EQ(q_conj_a.get_c(), -q.get_c()) << "Incorrect conjugate quaternion c";
    EXPECT_DOUBLE_EQ(q_conj_a.get_d(), -q.get_d()) << "Incorrect conjugate quaternion d";

    Quaternion q_conj_b;
    q.conjugate(q_conj_b);
    EXPECT_DOUBLE_EQ(q_conj_b.get_a(), q.get_a()) << "Incorrect conjugate quaternion a";
    EXPECT_DOUBLE_EQ(q_conj_b.get_b(), -q.get_b()) << "Incorrect conjugate quaternion b";
    EXPECT_DOUBLE_EQ(q_conj_b.get_c(), -q.get_c()) << "Incorrect conjugate quaternion c";
    EXPECT_DOUBLE_EQ(q_conj_b.get_d(), -q.get_d()) << "Incorrect conjugate quaternion d";
}

TEST(rotations_tests, test_q_inverse)
{
    Quaternion q(-2.17, 0.92, 3.02, RotationSequence::BODY_ZYZ);

    Quaternion q_inv_a = q.conjugate();
    EXPECT_DOUBLE_EQ(q_inv_a.get_a(), q.get_a()) << "Incorrect inverse quaternion a";
    EXPECT_DOUBLE_EQ(q_inv_a.get_b(), -q.get_b()) << "Incorrect inverse quaternion b";
    EXPECT_DOUBLE_EQ(q_inv_a.get_c(), -q.get_c()) << "Incorrect inverse quaternion c";
    EXPECT_DOUBLE_EQ(q_inv_a.get_d(), -q.get_d()) << "Incorrect inverse quaternion d";

    Quaternion q_inv_b;
    q.conjugate(q_inv_b);
    EXPECT_DOUBLE_EQ(q_inv_b.get_a(), q.get_a()) << "Incorrect inverse quaternion a";
    EXPECT_DOUBLE_EQ(q_inv_b.get_b(), -q.get_b()) << "Incorrect inverse quaternion b";
    EXPECT_DOUBLE_EQ(q_inv_b.get_c(), -q.get_c()) << "Incorrect inverse quaternion c";
    EXPECT_DOUBLE_EQ(q_inv_b.get_d(), -q.get_d()) << "Incorrect inverse quaternion d";
}

TEST(rotations_tests, test_q_norm)
{
    Quaternion q(-2.17, 0.92, 3.02, RotationSequence::BODY_XZX);
    EXPECT_DOUBLE_EQ(q.norm(), 1.0) << "Incorrect quaternion norm";
}

TEST(rotations_tests, test_q_operator_multiply)
{
    Quaternion q_a(1.06, 2.40, -1.46, RotationSequence::BODY_XYZ);
    Quaternion q_b(-2.64, 0.86, 1.82, RotationSequence::BODY_ZYX);

    Quaternion q_c = q_a * q_b;
    EXPECT_DOUBLE_EQ(q_c.get_a(), -0.616005951318418) << "Incorrect q_a * q_b quaternion a";
    EXPECT_DOUBLE_EQ(q_c.get_b(), 0.053517656679567968) << "Incorrect q_a * q_b quaternion b";
    EXPECT_DOUBLE_EQ(q_c.get_c(), 0.663725456977726) << "Incorrect q_a * q_b quaternion c";
    EXPECT_DOUBLE_EQ(q_c.get_d(), 0.420881273191772) << "Incorrect q_a * q_b quaternion d";

    q_c = q_b * q_a;
    EXPECT_DOUBLE_EQ(q_c.get_a(), -0.616005951318418) << "Incorrect q_b * q_a quaternion a";
    EXPECT_DOUBLE_EQ(q_c.get_b(), -0.66388126864859) << "Incorrect q_b * q_a quaternion b";
    EXPECT_DOUBLE_EQ(q_c.get_c(), 0.287821338724548) << "Incorrect q_b * q_a quaternion c";
    EXPECT_DOUBLE_EQ(q_c.get_d(), 0.311379520926857) << "Incorrect q_b * q_a quaternion d";
}

TEST(rotations_tests, test_q_operator_eq_and_ne)
{
    const double a1_rad(3.03), a2_rad(-2.16), a3_rad(2.23);

    Quaternion q(a1_rad, a2_rad, a3_rad, RotationSequence::BODY_YZY);
    Quaternion q_eq(a1_rad, a2_rad, a3_rad, RotationSequence::BODY_YZY);
    Quaternion q_ne(a1_rad, a2_rad, a3_rad, RotationSequence::BODY_XYZ);

    EXPECT_TRUE(q == q_eq) << "Equality operator did not return true for equal quaternions";
    EXPECT_FALSE(q == q_ne) << "Equality operator did not return false for not equal quaternions";

    EXPECT_TRUE(q != q_ne) << "Inequality operator did not return true for not equal quaternions";
    EXPECT_FALSE(q != q_eq) << "Inequality operator did not return false for equal quaternions";
}

TEST(rotations_tests, test_q_square)
{
    Quaternion q(-2.17, 0.92, 3.02, RotationSequence::BODY_ZXY);
    EXPECT_DOUBLE_EQ(q.square(), 1.0) << "Incorrect quaternion square";
}

TEST(rotations_tests, test_seq_xyx)
{
    const double           a1_rad(3.03), a2_rad(-2.16), a3_rad(2.23);
    const RotationSequence seq(RotationSequence::BODY_XYX);

    DCM dcm(a1_rad, a2_rad, a3_rad, seq);
    EXPECT_NEAR(dcm.get_e00(), -0.5556991462506127, constants::a_tol) << "Incorrect element 00";
    EXPECT_NEAR(dcm.get_e01(), -0.09258385044673502, constants::a_tol) << "Incorrect element 01";
    EXPECT_NEAR(dcm.get_e02(), -0.8262122545041296, constants::a_tol) << "Incorrect element 02";
    EXPECT_NEAR(dcm.get_e10(), -0.6571921829277988, constants::a_tol) << "Incorrect element 10";
    EXPECT_NEAR(dcm.get_e11(), 0.6575954307136367, constants::a_tol) << "Incorrect element 11";
    EXPECT_NEAR(dcm.get_e12(), 0.3683295863803792, constants::a_tol) << "Incorrect element 12";
    EXPECT_NEAR(dcm.get_e20(), 0.5092120320209791, constants::a_tol) << "Incorrect element 20";
    EXPECT_NEAR(dcm.get_e21(), 0.7476606717896851, constants::a_tol) << "Incorrect element 21";
    EXPECT_NEAR(dcm.get_e22(), -0.4262706022048224, constants::a_tol) << "Incorrect element 22";

    Quaternion q(a1_rad, a2_rad, a3_rad, seq);
    EXPECT_NEAR(q.get_a(), 0.4109822630778004, constants::a_tol) << "Incorrect quaternion a";
    EXPECT_NEAR(q.get_b(), -0.2307466279496697, constants::a_tol) << "Incorrect quaternion b";
    EXPECT_NEAR(q.get_c(), 0.8123369342780543, constants::a_tol) << "Incorrect quaternion c";
    EXPECT_NEAR(q.get_d(), 0.3434505471433091, constants::a_tol) << "Incorrect quaternion d";

    DCM dcm_from_q(q);
    check_dcm_near(dcm, dcm_from_q, constants::a_tol);

    Quaternion q_from_dcm(dcm);
    check_quaternion_near(q, q_from_dcm, constants::a_tol);

    KinematicVector kv(1.4, -1.2, -3.1);
    KinematicVector kv_transform_truth(1.894379804748026, -2.851005290734458, 1.137142905516698);
    check_dcm_transform(dcm, kv, kv_transform_truth);
    check_q_transform(dcm, kv, kv_transform_truth);

    KinematicVector kv_rotate_truth(-1.567905484502534, -3.236479990029817, -0.2772537931272867);
    check_dcm_rotate(dcm, kv, kv_rotate_truth);
    check_q_rotate(dcm, kv, kv_rotate_truth);
}

TEST(rotations_tests, test_seq_xyz)
{
    const double           a1_rad(-0.45), a2_rad(-0.11), a3_rad(-2.38);
    const RotationSequence seq(RotationSequence::BODY_XYZ);

    DCM dcm(a1_rad, a2_rad, a3_rad, seq);
    EXPECT_NEAR(dcm.get_e00(), -0.7193636778586622, constants::a_tol) << "Incorrect element 00";
    EXPECT_NEAR(dcm.get_e01(), -0.6559343418507525, constants::a_tol) << "Incorrect element 01";
    EXPECT_NEAR(dcm.get_e02(), 0.2286176680803103, constants::a_tol) << "Incorrect element 02";
    EXPECT_NEAR(dcm.get_e10(), 0.6859042379537842, constants::a_tol) << "Incorrect element 10";
    EXPECT_NEAR(dcm.get_e11(), -0.6187367489848085, constants::a_tol) << "Incorrect element 11";
    EXPECT_NEAR(dcm.get_e12(), 0.3830146365515926, constants::a_tol) << "Incorrect element 12";
    EXPECT_NEAR(dcm.get_e20(), -0.1097783008371748, constants::a_tol) << "Incorrect element 20";
    EXPECT_NEAR(dcm.get_e21(), 0.4323366450308489, constants::a_tol) << "Incorrect element 21";
    EXPECT_NEAR(dcm.get_e22(), 0.8950048882708811, constants::a_tol) << "Incorrect element 22";

    Quaternion q(a1_rad, a2_rad, a3_rad, seq);
    EXPECT_NEAR(q.get_a(), -0.37313015873399, constants::a_tol) << "Incorrect quaternion a";
    EXPECT_NEAR(q.get_b(), 0.03304611495798355, constants::a_tol) << "Incorrect quaternion b";
    EXPECT_NEAR(q.get_c(), 0.2267278327659468, constants::a_tol) << "Incorrect quaternion c";
    EXPECT_NEAR(q.get_d(), 0.8990418948962212, constants::a_tol) << "Incorrect quaternion d";

    DCM dcm_from_q(q);
    check_dcm_near(dcm, dcm_from_q, constants::a_tol);

    Quaternion q_from_dcm(dcm);
    check_quaternion_near(q, q_from_dcm, constants::a_tol);

    KinematicVector kv(0.9, -2.7, -1.2);
    KinematicVector kv_transform_truth(0.8492542112278636, 1.828285472555478, -2.340115278261806);
    check_dcm_transform(dcm, kv, kv_transform_truth);
    check_q_transform(dcm, kv, kv_transform_truth);

    KinematicVector kv_rotate_truth(-2.367634791543404, 0.5614443405562869, -1.902389483342078);
    check_dcm_rotate(dcm, kv, kv_rotate_truth);
    check_q_rotate(dcm, kv, kv_rotate_truth);
}

TEST(rotations_tests, test_seq_xzx)
{
    const double           a1_rad(0.52), a2_rad(-1.56), a3_rad(-1.32);
    const RotationSequence seq(RotationSequence::BODY_XZX);

    DCM dcm(a1_rad, a2_rad, a3_rad, seq);
    EXPECT_NEAR(dcm.get_e00(), 0.01079611705826739, constants::a_tol) << "Incorrect element 00";
    EXPECT_NEAR(dcm.get_e01(), -0.8677686033754274, constants::a_tol) << "Incorrect element 01";
    EXPECT_NEAR(dcm.get_e02(), -0.4968511797835689, constants::a_tol) << "Incorrect element 02";
    EXPECT_NEAR(dcm.get_e10(), 0.2481609880441226, constants::a_tol) << "Incorrect element 10";
    EXPECT_NEAR(dcm.get_e11(), 0.4836604675055965, constants::a_tol) << "Incorrect element 11";
    EXPECT_NEAR(dcm.get_e12(), -0.8393382370565711, constants::a_tol) << "Incorrect element 12";
    EXPECT_NEAR(dcm.get_e20(), 0.9686586436250022, constants::a_tol) << "Incorrect element 20";
    EXPECT_NEAR(dcm.get_e21(), -0.1142374858272359, constants::a_tol) << "Incorrect element 21";
    EXPECT_NEAR(dcm.get_e22(), 0.2205679690309612, constants::a_tol) << "Incorrect element 22";

    Quaternion q(a1_rad, a2_rad, a3_rad, seq);
    EXPECT_NEAR(q.get_a(), 0.654794729971696, constants::a_tol) << "Incorrect quaternion a";
    EXPECT_NEAR(q.get_b(), -0.2768427714975188, constants::a_tol) << "Incorrect quaternion b";
    EXPECT_NEAR(q.get_c(), 0.5595302452540809, constants::a_tol) << "Incorrect quaternion c";
    EXPECT_NEAR(q.get_d(), -0.426060847904116, constants::a_tol) << "Incorrect quaternion d";

    DCM dcm_from_q(q);
    check_dcm_near(dcm, dcm_from_q, constants::a_tol);

    Quaternion q_from_dcm(dcm);
    check_quaternion_near(q, q_from_dcm, constants::a_tol);

    KinematicVector kv(1.2, -2.3, 3.2);
    KinematicVector kv_transform_truth(0.4188993529259828, -3.500508248190952, 2.130954090651721);
    check_dcm_transform(dcm, kv, kv_transform_truth);
    check_q_transform(dcm, kv, kv_transform_truth);

    KinematicVector kv_rotate_truth(2.541892727568446, -2.51930135396054, 2.040074030388907);
    check_dcm_rotate(dcm, kv, kv_rotate_truth);
    check_q_rotate(dcm, kv, kv_rotate_truth);
}

TEST(rotations_tests, test_seq_xzy)
{
    const double           a1_rad(3.03), a2_rad(1.45), a3_rad(-0.98);
    const RotationSequence seq(RotationSequence::BODY_XZY);

    DCM dcm(a1_rad, a2_rad, a3_rad, seq);
    EXPECT_NEAR(dcm.get_e00(), 0.06712275948539267, constants::a_tol) << "Incorrect element 00";
    EXPECT_NEAR(dcm.get_e01(), -0.6420092590059827, constants::a_tol) << "Incorrect element 01";
    EXPECT_NEAR(dcm.get_e02(), -0.7637530009824218, constants::a_tol) << "Incorrect element 02";
    EXPECT_NEAR(dcm.get_e10(), -0.9927129910375885, constants::a_tol) << "Incorrect element 10";
    EXPECT_NEAR(dcm.get_e11(), -0.1197532419754337, constants::a_tol) << "Incorrect element 11";
    EXPECT_NEAR(dcm.get_e12(), 0.01341933163678318, constants::a_tol) << "Incorrect element 12";
    EXPECT_NEAR(dcm.get_e20(), -0.1000772330965983, constants::a_tol) << "Incorrect element 20";
    EXPECT_NEAR(dcm.get_e21(), 0.7572867834492837, constants::a_tol) << "Incorrect element 21";
    EXPECT_NEAR(dcm.get_e22(), -0.6453690998403679, constants::a_tol) << "Incorrect element 22";

    Quaternion q(a1_rad, a2_rad, a3_rad, seq);
    EXPECT_NEAR(q.get_a(), -0.2747728232875256, constants::a_tol) << "Incorrect quaternion a";
    EXPECT_NEAR(q.get_b(), 0.6768022424056369, constants::a_tol) << "Incorrect quaternion b";
    EXPECT_NEAR(q.get_c(), -0.6038404380255478, constants::a_tol) << "Incorrect quaternion c";
    EXPECT_NEAR(q.get_d(), -0.3190851699192839, constants::a_tol) << "Incorrect quaternion d";

    DCM dcm_from_q(q);
    check_dcm_near(dcm, dcm_from_q, constants::a_tol);

    Quaternion q_from_dcm(dcm);
    check_quaternion_near(q, q_from_dcm, constants::a_tol);

    KinematicVector kv(0.8, -3.9, 4.1);
    KinematicVector kv_transform_truth(-0.5738529863162825, -0.2721134894150683, -5.679493551274993);
    check_dcm_transform(dcm, kv, kv_transform_truth);
    check_q_transform(dcm, kv, kv_transform_truth);

    KinematicVector kv_rotate_truth(3.514962216938856, 3.058306048641468, -3.3093511035149);
    check_dcm_rotate(dcm, kv, kv_rotate_truth);
    check_q_rotate(dcm, kv, kv_rotate_truth);
}

TEST(rotations_tests, test_seq_yxy)
{
    const double           a1_rad(2.39), a2_rad(2.00), a3_rad(-1.50);
    const RotationSequence seq(RotationSequence::BODY_YXY);

    DCM dcm(a1_rad, a2_rad, a3_rad, seq);
    EXPECT_NEAR(dcm.get_e00(), -0.3351153743894538, constants::a_tol) << "Incorrect element 00";
    EXPECT_NEAR(dcm.get_e01(), -0.9070196245905846, constants::a_tol) << "Incorrect element 01";
    EXPECT_NEAR(dcm.get_e02(), 0.2549766390385037, constants::a_tol) << "Incorrect element 02";
    EXPECT_NEAR(dcm.get_e10(), 0.6208712127298156, constants::a_tol) << "Incorrect element 10";
    EXPECT_NEAR(dcm.get_e11(), -0.4161468365471424, constants::a_tol) << "Incorrect element 11";
    EXPECT_NEAR(dcm.get_e12(), -0.6643348159137937, constants::a_tol) << "Incorrect element 12";
    EXPECT_NEAR(dcm.get_e20(), 0.7086724370618802, constants::a_tol) << "Incorrect element 20";
    EXPECT_NEAR(dcm.get_e21(), -0.06432115545729157, constants::a_tol) << "Incorrect element 21";
    EXPECT_NEAR(dcm.get_e22(), 0.7025995772197806, constants::a_tol) << "Incorrect element 22";

    Quaternion q(a1_rad, a2_rad, a3_rad, seq);
    EXPECT_NEAR(q.get_a(), -0.4876826238147061, constants::a_tol) << "Incorrect quaternion a";
    EXPECT_NEAR(q.get_b(), 0.3075840880710135, constants::a_tol) << "Incorrect quaternion b";
    EXPECT_NEAR(q.get_c(), -0.2325773853056927, constants::a_tol) << "Incorrect quaternion c";
    EXPECT_NEAR(q.get_d(), 0.7832403507475174, constants::a_tol) << "Incorrect quaternion d";

    DCM dcm_from_q(q);
    check_dcm_near(dcm, dcm_from_q, constants::a_tol);

    Quaternion q_from_dcm(dcm);
    check_quaternion_near(q, q_from_dcm, constants::a_tol);

    KinematicVector kv(0.9, -4.8, -0.7);
    KinematicVector kv_transform_truth(3.873606713757346, 3.021323278022773, 0.4547270354968453);
    check_dcm_transform(dcm, kv, kv_transform_truth);
    check_q_transform(dcm, kv, kv_transform_truth);

    KinematicVector kv_rotate_truth(-3.77785636399694, 1.226211962114861, 2.926466387467017);
    check_dcm_rotate(dcm, kv, kv_rotate_truth);
    check_q_rotate(dcm, kv, kv_rotate_truth);
}

TEST(rotations_tests, test_seq_yxz)
{
    const double           a1_rad(-1.18), a2_rad(-2.13), a3_rad(-2.02);
    const RotationSequence seq(RotationSequence::BODY_YXZ);

    DCM dcm(a1_rad, a2_rad, a3_rad, seq);
    EXPECT_NEAR(dcm.get_e00(), -0.8714288779471885, constants::a_tol) << "Incorrect element 00";
    EXPECT_NEAR(dcm.get_e01(), 0.4778810009563105, constants::a_tol) << "Incorrect element 01";
    EXPECT_NEAR(dcm.get_e02(), -0.1106411298057783, constants::a_tol) << "Incorrect element 02";
    EXPECT_NEAR(dcm.get_e10(), 0.002784518557860272, constants::a_tol) << "Incorrect element 10";
    EXPECT_NEAR(dcm.get_e11(), 0.2303736709597711, constants::a_tol) << "Incorrect element 11";
    EXPECT_NEAR(dcm.get_e12(), 0.9730982572098874, constants::a_tol) << "Incorrect element 12";
    EXPECT_NEAR(dcm.get_e20(), 0.490513972416796, constants::a_tol) << "Incorrect element 20";
    EXPECT_NEAR(dcm.get_e21(), 0.8476778401335698, constants::a_tol) << "Incorrect element 21";
    EXPECT_NEAR(dcm.get_e22(), -0.2020849381086598, constants::a_tol) << "Incorrect element 22";

    Quaternion q(a1_rad, a2_rad, a3_rad, seq);
    EXPECT_NEAR(q.get_a(), 0.198027684241322, constants::a_tol) << "Incorrect quaternion a";
    EXPECT_NEAR(q.get_b(), 0.1583369738893134, constants::a_tol) << "Incorrect quaternion b";
    EXPECT_NEAR(q.get_c(), 0.7589281071049516, constants::a_tol) << "Incorrect quaternion c";
    EXPECT_NEAR(q.get_d(), 0.5997854343177145, constants::a_tol) << "Incorrect quaternion d";

    DCM dcm_from_q(q);
    check_dcm_near(dcm, dcm_from_q, constants::a_tol);

    Quaternion q_from_dcm(dcm);
    check_quaternion_near(q, q_from_dcm, constants::a_tol);

    KinematicVector kv(-0.8, -4.1, 1.0);
    KinematicVector kv_transform_truth(-1.3728101313689, 0.02633859142853778, -4.069975260589732);
    check_dcm_transform(dcm, kv, kv_transform_truth);
    check_q_transform(dcm, kv, kv_transform_truth);

    KinematicVector kv_rotate_truth(1.17624054868732, -0.47915901156654, -4.103274888824575);
    check_dcm_rotate(dcm, kv, kv_rotate_truth);
    check_q_rotate(dcm, kv, kv_rotate_truth);
}

TEST(rotations_tests, test_seq_yzx)
{
    const double           a1_rad(-0.18), a2_rad(1.23), a3_rad(1.26);
    const RotationSequence seq(RotationSequence::BODY_YZX);

    DCM dcm(a1_rad, a2_rad, a3_rad, seq);
    EXPECT_NEAR(dcm.get_e00(), 0.3288376797232791, constants::a_tol) << "Incorrect element 00";
    EXPECT_NEAR(dcm.get_e01(), 0.9424888019316975, constants::a_tol) << "Incorrect element 01";
    EXPECT_NEAR(dcm.get_e02(), 0.05983843770991672, constants::a_tol) << "Incorrect element 02";
    EXPECT_NEAR(dcm.get_e10(), -0.4540246228471165, constants::a_tol) << "Incorrect element 10";
    EXPECT_NEAR(dcm.get_e11(), 0.1022155483726017, constants::a_tol) << "Incorrect element 11";
    EXPECT_NEAR(dcm.get_e12(), 0.8851065605447865, constants::a_tol) << "Incorrect element 12";
    EXPECT_NEAR(dcm.get_e20(), 0.8280866031054624, constants::a_tol) << "Incorrect element 20";
    EXPECT_NEAR(dcm.get_e21(), -0.3182245117904053, constants::a_tol) << "Incorrect element 21";
    EXPECT_NEAR(dcm.get_e22(), 0.4615254465931586, constants::a_tol) << "Incorrect element 22";

    Quaternion q(a1_rad, a2_rad, a3_rad, seq);
    EXPECT_NEAR(q.get_a(), 0.6878551218623439, constants::a_tol) << "Incorrect quaternion a";
    EXPECT_NEAR(q.get_b(), 0.4373490267388047, constants::a_tol) << "Incorrect quaternion b";
    EXPECT_NEAR(q.get_c(), 0.2792187413373984, constants::a_tol) << "Incorrect quaternion c";
    EXPECT_NEAR(q.get_d(), 0.5075608876029747, constants::a_tol) << "Incorrect quaternion d";

    DCM dcm_from_q(q);
    check_dcm_near(dcm, dcm_from_q, constants::a_tol);

    Quaternion q_from_dcm(dcm);
    check_quaternion_near(q, q_from_dcm, constants::a_tol);

    KinematicVector kv(1.4, -4.7, -4.3);
    KinematicVector kv_transform_truth(-4.226629899619029, -4.922005759679773, 0.67041702941197);
    check_dcm_transform(dcm, kv, kv_transform_truth);
    check_q_transform(dcm, kv, kv_transform_truth);

    KinematicVector kv_rotate_truth(-0.9664839143594498, 2.207436646051891, -6.060786442117196);
    check_dcm_rotate(dcm, kv, kv_rotate_truth);
    check_q_rotate(dcm, kv, kv_rotate_truth);
}

TEST(rotations_tests, test_seq_yzy)
{
    const double           a1_rad(-1.13), a2_rad(0.19), a3_rad(0.97);
    const RotationSequence seq(RotationSequence::BODY_YZY);

    DCM dcm(a1_rad, a2_rad, a3_rad, seq);
    EXPECT_NEAR(dcm.get_e00(), 0.9828868742370757, constants::a_tol) << "Incorrect element 00";
    EXPECT_NEAR(dcm.get_e01(), 0.1067618447856784, constants::a_tol) << "Incorrect element 01";
    EXPECT_NEAR(dcm.get_e02(), 0.1501176237169703, constants::a_tol) << "Incorrect element 02";
    EXPECT_NEAR(dcm.get_e10(), -0.0805784998565755, constants::a_tol) << "Incorrect element 10";
    EXPECT_NEAR(dcm.get_e11(), 0.9820042351172703, constants::a_tol) << "Incorrect element 11";
    EXPECT_NEAR(dcm.get_e12(), -0.1708062866893626, constants::a_tol) << "Incorrect element 12";
    EXPECT_NEAR(dcm.get_e20(), -0.1656517365237534, constants::a_tol) << "Incorrect element 20";
    EXPECT_NEAR(dcm.get_e21(), 0.1557870043030021, constants::a_tol) << "Incorrect element 21";
    EXPECT_NEAR(dcm.get_e22(), 0.9738017824367347, constants::a_tol) << "Incorrect element 22";

    Quaternion q(a1_rad, a2_rad, a3_rad, seq);
    EXPECT_NEAR(q.get_a(), 0.9923070205071465, constants::a_tol) << "Incorrect quaternion a";
    EXPECT_NEAR(q.get_b(), -0.08228131118770329, constants::a_tol) << "Incorrect quaternion b";
    EXPECT_NEAR(q.get_c(), -0.0795543500436339, constants::a_tol) << "Incorrect quaternion c";
    EXPECT_NEAR(q.get_d(), 0.04719818079753894, constants::a_tol) << "Incorrect quaternion d";

    DCM dcm_from_q(q);
    check_dcm_near(dcm, dcm_from_q, constants::a_tol);

    Quaternion q_from_dcm(dcm);
    check_quaternion_near(q, q_from_dcm, constants::a_tol);

    KinematicVector kv(-0.9, 3.2, 2.2);
    KinematicVector kv_transform_truth(-0.2127015113218627, 2.839160371529585, 2.789968898001801);
    check_dcm_transform(dcm, kv, kv_transform_truth);
    check_q_transform(dcm, kv, kv_transform_truth);

    KinematicVector kv_rotate_truth(-1.506883206706667, 3.389059301534759, 1.460677942609583);
    check_dcm_rotate(dcm, kv, kv_rotate_truth);
    check_q_rotate(dcm, kv, kv_rotate_truth);
}

TEST(rotations_tests, test_seq_zxy)
{
    const double           a1_rad(2.94), a2_rad(0.20), a3_rad(-1.10);
    const RotationSequence seq(RotationSequence::BODY_ZXY);

    DCM dcm(a1_rad, a2_rad, a3_rad, seq);
    EXPECT_NEAR(dcm.get_e00(), -0.4089584776574394, constants::a_tol) << "Incorrect element 00";
    EXPECT_NEAR(dcm.get_e01(), 0.2642935484074433, constants::a_tol) << "Incorrect element 01";
    EXPECT_NEAR(dcm.get_e02(), 0.8734425475223383, constants::a_tol) << "Incorrect element 02";
    EXPECT_NEAR(dcm.get_e10(), -0.1962387159074697, constants::a_tol) << "Incorrect element 10";
    EXPECT_NEAR(dcm.get_e11(), -0.96021917465776, constants::a_tol) << "Incorrect element 11";
    EXPECT_NEAR(dcm.get_e12(), 0.1986693307950612, constants::a_tol) << "Incorrect element 12";
    EXPECT_NEAR(dcm.get_e20(), 0.8912033044884298, constants::a_tol) << "Incorrect element 20";
    EXPECT_NEAR(dcm.get_e21(), -0.09015573686556229, constants::a_tol) << "Incorrect element 21";
    EXPECT_NEAR(dcm.get_e22(), 0.4445543984476258, constants::a_tol) << "Incorrect element 22";

    Quaternion q(a1_rad, a2_rad, a3_rad, seq);
    EXPECT_NEAR(q.get_a(), 0.1372741291471433, constants::a_tol) << "Incorrect quaternion a";
    EXPECT_NEAR(q.get_b(), 0.5260005462337218, constants::a_tol) << "Incorrect quaternion b";
    EXPECT_NEAR(q.get_c(), 0.03234541911945765, constants::a_tol) << "Incorrect quaternion c";
    EXPECT_NEAR(q.get_d(), 0.8387091347366536, constants::a_tol) << "Incorrect quaternion d";

    DCM dcm_from_q(q);
    check_dcm_near(dcm, dcm_from_q, constants::a_tol);

    Quaternion q_from_dcm(dcm);
    check_quaternion_near(q, q_from_dcm, constants::a_tol);

    KinematicVector kv(-3.9, 1.1, 2.8);
    KinematicVector kv_transform_truth(4.331300099174748, 0.2653640261417672, -2.330111882403642);
    check_dcm_transform(dcm, kv, kv_transform_truth);
    check_q_transform(dcm, kv, kv_transform_truth);

    KinematicVector kv_rotate_truth(3.8744447279334, -2.339421994136139, -1.9431373558092);
    check_dcm_rotate(dcm, kv, kv_rotate_truth);
    check_q_rotate(dcm, kv, kv_rotate_truth);
}

TEST(rotations_tests, test_seq_zxz)
{
    const double           a1_rad(-0.48), a2_rad(-2.57), a3_rad(-1.47);
    const RotationSequence seq(RotationSequence::BODY_ZXZ);

    DCM dcm(a1_rad, a2_rad, a3_rad, seq);
    EXPECT_NEAR(dcm.get_e00(), 0.4756582297157966, constants::a_tol) << "Incorrect element 00";
    EXPECT_NEAR(dcm.get_e01(), 0.6957453223663198, constants::a_tol) << "Incorrect element 01";
    EXPECT_NEAR(dcm.get_e02(), 0.5382264346063096, constants::a_tol) << "Incorrect element 02";
    EXPECT_NEAR(dcm.get_e10(), 0.8434123305557789, constants::a_tol) << "Incorrect element 10";
    EXPECT_NEAR(dcm.get_e11(), -0.5345020040709673, constants::a_tol) << "Incorrect element 11";
    EXPECT_NEAR(dcm.get_e12(), -0.05443572641739133, constants::a_tol) << "Incorrect element 12";
    EXPECT_NEAR(dcm.get_e20(), 0.2498097059165311, constants::a_tol) << "Incorrect element 20";
    EXPECT_NEAR(dcm.get_e21(), 0.4798396128390249, constants::a_tol) << "Incorrect element 21";
    EXPECT_NEAR(dcm.get_e22(), -0.8410404608462014, constants::a_tol) << "Incorrect element 22";

    Quaternion q(a1_rad, a2_rad, a3_rad, seq);
    EXPECT_NEAR(q.get_a(), -0.1582053766458555, constants::a_tol) << "Incorrect quaternion a";
    EXPECT_NEAR(q.get_b(), 0.8442749396128262, constants::a_tol) << "Incorrect quaternion b";
    EXPECT_NEAR(q.get_c(), 0.4557631586305099, constants::a_tol) << "Incorrect quaternion c";
    EXPECT_NEAR(q.get_d(), 0.2333470127883412, constants::a_tol) << "Incorrect quaternion d";

    DCM dcm_from_q(q);
    check_dcm_near(dcm, dcm_from_q, constants::a_tol);

    Quaternion q_from_dcm(dcm);
    check_quaternion_near(q, q_from_dcm, constants::a_tol);

    KinematicVector kv(-3.5, -2.2, -0.6);
    KinematicVector kv_transform_truth(-3.518379373974978, -1.743377312138663, -1.425356842445993);
    check_dcm_transform(dcm, kv, kv_transform_truth);
    check_q_transform(dcm, kv, kv_transform_truth);

    KinematicVector kv_rotate_truth(-3.67019675477792, -1.547107987029406, -1.259409646496102);
    check_dcm_rotate(dcm, kv, kv_rotate_truth);
    check_q_rotate(dcm, kv, kv_rotate_truth);
}

TEST(rotations_tests, test_seq_zyx)
{
    const double           a1_rad(0.17), a2_rad(-0.27), a3_rad(2.36);
    const RotationSequence seq(RotationSequence::BODY_ZYX);

    DCM dcm(a1_rad, a2_rad, a3_rad, seq);
    EXPECT_NEAR(dcm.get_e00(), 0.9498779142489947, constants::a_tol) << "Incorrect element 00";
    EXPECT_NEAR(dcm.get_e01(), 0.1630530242095858, constants::a_tol) << "Incorrect element 01";
    EXPECT_NEAR(dcm.get_e02(), 0.2667314366888311, constants::a_tol) << "Incorrect element 02";
    EXPECT_NEAR(dcm.get_e10(), -0.06509566707865096, constants::a_tol) << "Incorrect element 10";
    EXPECT_NEAR(dcm.get_e11(), -0.7313481482599988, constants::a_tol) << "Incorrect element 11";
    EXPECT_NEAR(dcm.get_e12(), 0.6788905951361062, constants::a_tol) << "Incorrect element 12";
    EXPECT_NEAR(dcm.get_e20(), 0.3057687069494934, constants::a_tol) << "Incorrect element 20";
    EXPECT_NEAR(dcm.get_e21(), -0.6622262433132495, constants::a_tol) << "Incorrect element 21";
    EXPECT_NEAR(dcm.get_e22(), -0.6840774082789577, constants::a_tol) << "Incorrect element 22";

    Quaternion q(a1_rad, a2_rad, a3_rad, seq);
    EXPECT_NEAR(q.get_a(), 0.3655312427515732, constants::a_tol) << "Incorrect quaternion a";
    EXPECT_NEAR(q.get_b(), 0.917238173920486, constants::a_tol) << "Incorrect quaternion b";
    EXPECT_NEAR(q.get_c(), 0.02669899703155687, constants::a_tol) << "Incorrect quaternion c";
    EXPECT_NEAR(q.get_d(), 0.1560391182781152, constants::a_tol) << "Incorrect quaternion d";

    DCM dcm_from_q(q);
    check_dcm_near(dcm, dcm_from_q, constants::a_tol);

    Quaternion q_from_dcm(dcm);
    check_quaternion_near(q, q_from_dcm, constants::a_tol);

    KinematicVector kv(0.2, 4.4, 1.4);
    KinematicVector kv_transform_truth(1.28083290073634, -2.280504152569177, -3.81035010077894);
    check_dcm_transform(dcm, kv, kv_transform_truth);
    check_q_transform(dcm, kv, kv_transform_truth);

    KinematicVector kv_rotate_truth(0.3316308374330253, -4.112437988140627, 2.082756534346093);
    check_dcm_rotate(dcm, kv, kv_rotate_truth);
    check_q_rotate(dcm, kv, kv_rotate_truth);
}

TEST(rotations_tests, test_seq_zyz)
{
    const double           a1_rad(2.88), a2_rad(-1.63), a3_rad(1.11);
    const RotationSequence seq(RotationSequence::BODY_ZYZ);

    DCM dcm(a1_rad, a2_rad, a3_rad, seq);
    EXPECT_NEAR(dcm.get_e00(), -0.2062298843675349, constants::a_tol) << "Incorrect element 00";
    EXPECT_NEAR(dcm.get_e01(), -0.8720307322246085, constants::a_tol) << "Incorrect element 01";
    EXPECT_NEAR(dcm.get_e02(), 0.4438824583710945, constants::a_tol) << "Incorrect element 02";
    EXPECT_NEAR(dcm.get_e10(), -0.1661927342547344, constants::a_tol) << "Incorrect element 10";
    EXPECT_NEAR(dcm.get_e11(), -0.4158276007932318, constants::a_tol) << "Incorrect element 11";
    EXPECT_NEAR(dcm.get_e12(), -0.8941293986328153, constants::a_tol) << "Incorrect element 12";
    EXPECT_NEAR(dcm.get_e20(), 0.9642868918919765, constants::a_tol) << "Incorrect element 20";
    EXPECT_NEAR(dcm.get_e21(), -0.2581662419340645, constants::a_tol) << "Incorrect element 21";
    EXPECT_NEAR(dcm.get_e22(), -0.05916909371414814, constants::a_tol) << "Incorrect element 22";

    Quaternion q(a1_rad, a2_rad, a3_rad, seq);
    EXPECT_NEAR(q.get_a(), -0.2823001156239072, constants::a_tol) << "Incorrect quaternion a";
    EXPECT_NEAR(q.get_b(), 0.5631977472744021, constants::a_tol) << "Incorrect quaternion b";
    EXPECT_NEAR(q.get_c(), -0.460860981557468, constants::a_tol) << "Incorrect quaternion c";
    EXPECT_NEAR(q.get_d(), 0.6250776734627902, constants::a_tol) << "Incorrect quaternion d";

    DCM dcm_from_q(q);
    check_dcm_near(dcm, dcm_from_q, constants::a_tol);

    Quaternion q_from_dcm(dcm);
    check_quaternion_near(q, q_from_dcm, constants::a_tol);

    KinematicVector kv(-2.1, 1.7, 2.0);
    KinematicVector kv_transform_truth(-0.161604570867822, -2.146160976679182, -2.582223271689357);
    check_dcm_transform(dcm, kv, kv_transform_truth);
    check_q_transform(dcm, kv, kv_transform_truth);

    KinematicVector kv_rotate_truth(2.079128892722728, 0.608025132455055, -2.570511327683381);
    check_dcm_rotate(dcm, kv, kv_rotate_truth);
    check_q_rotate(dcm, kv, kv_rotate_truth);
}
