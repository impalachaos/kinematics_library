#include <gtest/gtest.h>
#include <kinematics-library/errors.h>
#include <kinematics-library/rotations.h>
#include <stdint.h>
#include <string>
#include <universal-constants/constants.h>

struct DCMInitTestParams
{
    std::string      name;
    bool             input1Null;
    dcm_t            input1;
    precision_type_t input1a1;
    precision_type_t input1a2;
    precision_type_t input1a3;
    int              input1Rotation;
    dcm_t            want;
    precision_type_t near;
    int              wantError;
};

class InitTest : public testing::TestWithParam<DCMInitTestParams>
{};

TEST_P(InitTest, Compute)
{
    DCMInitTestParams params = GetParam();

    dcm_t *input1 = &params.input1;

    int error;

    if (params.input1Null)
    {
        input1 = NULL;
    }

    error = dcm_init(input1, params.input1a1, params.input1a2, params.input1a3, params.input1Rotation);
    EXPECT_EQ(error, params.wantError);
    if (error == KL_NO_ERROR)
    {
        if (params.near == 0)
        {
            EXPECT_DOUBLE_EQ(params.want.e00, params.input1.e00);
            EXPECT_DOUBLE_EQ(params.want.e01, params.input1.e01);
            EXPECT_DOUBLE_EQ(params.want.e02, params.input1.e02);
            EXPECT_DOUBLE_EQ(params.want.e10, params.input1.e10);
            EXPECT_DOUBLE_EQ(params.want.e11, params.input1.e11);
            EXPECT_DOUBLE_EQ(params.want.e12, params.input1.e12);
            EXPECT_DOUBLE_EQ(params.want.e20, params.input1.e20);
            EXPECT_DOUBLE_EQ(params.want.e21, params.input1.e21);
            EXPECT_DOUBLE_EQ(params.want.e22, params.input1.e22);
        }
        else
        {
            EXPECT_NEAR(params.want.e00, params.input1.e00, params.near);
            EXPECT_NEAR(params.want.e01, params.input1.e01, params.near);
            EXPECT_NEAR(params.want.e02, params.input1.e02, params.near);
            EXPECT_NEAR(params.want.e10, params.input1.e10, params.near);
            EXPECT_NEAR(params.want.e11, params.input1.e11, params.near);
            EXPECT_NEAR(params.want.e12, params.input1.e12, params.near);
            EXPECT_NEAR(params.want.e20, params.input1.e20, params.near);
            EXPECT_NEAR(params.want.e21, params.input1.e21, params.near);
            EXPECT_NEAR(params.want.e22, params.input1.e22, params.near);
        }
    }
}

INSTANTIATE_TEST_SUITE_P(
    DCM,
    InitTest,
    testing::Values(
        DCMInitTestParams{
            "NULL input",                                  // name
            true,                                          // input1Null
            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, // input1
            0.0,                                           // input1a1
            0.0,                                           // input1a2
            0.0,                                           // input1a3
            0,                                             // input1Rotation
            {1.0, 2.0, 3.0, 0.0, 1.0, 2.0, 0.0, 0.0, 1.0}, // want
            0.0,                                           // near
            KL_ERROR_NULL_ARGUMENT                         // wantError
        },
        DCMInitTestParams{
            "Sequence body xyx",                           // name
            false,                                         // input1Null
            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, // input1
            3.03,                                          // input1a1
            -2.16,                                         // input1a2
            2.23,                                          // input1a3
            ROTATION_SEQ_BODY_XYX,                         // input1Rotation
            {-0.5556991462506127,
             -0.09258385044673502,
             -0.8262122545041296,
             -0.6571921829277988,
             0.6575954307136367,
             0.3683295863803792,
             0.5092120320209791,
             0.7476606717896851,
             -0.4262706022048224}, // want
            1.0E-14,               // near
            KL_NO_ERROR            // wantError
        },
        DCMInitTestParams{
            "Sequence body xyz",                           // name
            false,                                         // input1Null
            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, // input1
            -0.45,                                         // input1a1
            -0.11,                                         // input1a2
            -2.38,                                         // input1a3
            ROTATION_SEQ_BODY_XYZ,                         // input1Rotation
            {-0.7193636778586622,
             -0.6559343418507525,
             0.2286176680803103,
             0.6859042379537842,
             -0.6187367489848085,
             0.3830146365515926,
             -0.1097783008371748,
             0.4323366450308489,
             0.8950048882708811}, // want
            1.0E-14,              // near
            KL_NO_ERROR           // wantError
        },
        DCMInitTestParams{
            "Sequence body xzx",                           // name
            false,                                         // input1Null
            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, // input1
            0.52,                                          // input1a1
            -1.56,                                         // input1a2
            -1.32,                                         // input1a3
            ROTATION_SEQ_BODY_XZX,                         // input1Rotation
            {0.01079611705826739,
             -0.8677686033754274,
             -0.4968511797835689,
             0.2481609880441226,
             0.4836604675055965,
             -0.8393382370565711,
             0.9686586436250022,
             -0.1142374858272359,
             0.2205679690309612}, // want
            1.0E-14,              // near
            KL_NO_ERROR           // wantError
        },
        DCMInitTestParams{
            "Sequence body xzy",                           // name
            false,                                         // input1Null
            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, // input1
            3.03,                                          // input1a1
            1.45,                                          // input1a2
            -0.98,                                         // input1a3
            ROTATION_SEQ_BODY_XZY,                         // input1Rotation
            {0.06712275948539267,
             -0.6420092590059827,
             -0.7637530009824218,
             -0.9927129910375885,
             -0.1197532419754337,
             0.01341933163678318,
             -0.1000772330965983,
             0.7572867834492837,
             -0.6453690998403679}, // want
            1.0E-14,               // near
            KL_NO_ERROR            // wantError
        },
        DCMInitTestParams{
            "Sequence body yxy",                           // name
            false,                                         // input1Null
            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, // input1
            2.39,                                          // input1a1
            2.00,                                          // input1a2
            -1.50,                                         // input1a3
            ROTATION_SEQ_BODY_YXY,                         // input1Rotation
            {-0.3351153743894538,
             -0.9070196245905846,
             0.2549766390385037,
             0.6208712127298156,
             -0.4161468365471424,
             -0.6643348159137937,
             0.7086724370618802,
             -0.06432115545729157,
             0.7025995772197806}, // want
            1.0E-14,              // near
            KL_NO_ERROR           // wantError
        },
        DCMInitTestParams{
            "Sequence body yxz",                           // name
            false,                                         // input1Null
            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, // input1
            -1.18,                                         // input1a1
            -2.13,                                         // input1a2
            -2.02,                                         // input1a3
            ROTATION_SEQ_BODY_YXZ,                         // input1Rotation
            {-0.8714288779471885,
             0.4778810009563105,
             -0.1106411298057783,
             0.002784518557860272,
             0.2303736709597711,
             0.9730982572098874,
             0.490513972416796,
             0.8476778401335698,
             -0.2020849381086598}, // want
            1.0E-14,               // near
            KL_NO_ERROR            // wantError
        },
        DCMInitTestParams{
            "Sequence body yzx",                           // name
            false,                                         // input1Null
            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, // input1
            -0.18,                                         // input1a1
            1.23,                                          // input1a2
            1.26,                                          // input1a3
            ROTATION_SEQ_BODY_YZX,                         // input1Rotation
            {0.3288376797232791,
             0.9424888019316975,
             0.05983843770991672,
             -0.4540246228471165,
             0.1022155483726017,
             0.8851065605447865,
             0.8280866031054624,
             -0.3182245117904053,
             0.4615254465931586}, // want
            1.0E-14,              // near
            KL_NO_ERROR           // wantError
        },
        DCMInitTestParams{
            "Sequence body yzy",                           // name
            false,                                         // input1Null
            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, // input1
            -1.13,                                         // input1a1
            0.19,                                          // input1a2
            0.97,                                          // input1a3
            ROTATION_SEQ_BODY_YZY,                         // input1Rotation
            {0.9828868742370757,
             0.1067618447856784,
             0.1501176237169703,
             -0.0805784998565755,
             0.9820042351172703,
             -0.1708062866893626,
             -0.1656517365237534,
             0.1557870043030021,
             0.9738017824367347}, // want
            1.0E-14,              // near
            KL_NO_ERROR           // wantError
        },
        DCMInitTestParams{
            "Sequence body zxy",                           // name
            false,                                         // input1Null
            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, // input1
            2.94,                                          // input1a1
            0.20,                                          // input1a2
            -1.10,                                         // input1a3
            ROTATION_SEQ_BODY_ZXY,                         // input1Rotation
            {-0.4089584776574394,
             0.2642935484074433,
             0.8734425475223383,
             -0.1962387159074697,
             -0.96021917465776,
             0.1986693307950612,
             0.8912033044884298,
             -0.09015573686556229,
             0.4445543984476258}, // want
            1.0E-14,              // near
            KL_NO_ERROR           // wantError
        },
        DCMInitTestParams{
            "Sequence body zxz",                           // name
            false,                                         // input1Null
            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, // input1
            -0.48,                                         // input1a1
            -2.57,                                         // input1a2
            -1.47,                                         // input1a3
            ROTATION_SEQ_BODY_ZXZ,                         // input1Rotation
            {0.4756582297157966,
             0.6957453223663198,
             0.5382264346063096,
             0.8434123305557789,
             -0.5345020040709673,
             -0.05443572641739133,
             0.2498097059165311,
             0.4798396128390249,
             -0.8410404608462014}, // want
            1.0E-14,               // near
            KL_NO_ERROR            // wantError
        },
        DCMInitTestParams{
            "Sequence body zyx",                           // name
            false,                                         // input1Null
            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, // input1
            0.17,                                          // input1a1
            -0.27,                                         // input1a2
            2.36,                                          // input1a3
            ROTATION_SEQ_BODY_ZYX,                         // input1Rotation
            {0.9498779142489947,
             0.1630530242095858,
             0.2667314366888311,
             -0.06509566707865096,
             -0.7313481482599988,
             0.6788905951361062,
             0.3057687069494934,
             -0.6622262433132495,
             -0.6840774082789577}, // want
            1.0E-14,               // near
            KL_NO_ERROR            // wantError
        },
        DCMInitTestParams{
            "Sequence body zyz",                           // name
            false,                                         // input1Null
            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, // input1
            2.88,                                          // input1a1
            -1.63,                                         // input1a2
            1.11,                                          // input1a3
            ROTATION_SEQ_BODY_ZYZ,                         // input1Rotation
            {-0.2062298843675349,
             -0.8720307322246085,
             0.4438824583710945,
             -0.1661927342547344,
             -0.4158276007932318,
             -0.8941293986328153,
             0.9642868918919765,
             -0.2581662419340645,
             -0.05916909371414814}, // want
            1.0E-14,                // near
            KL_NO_ERROR             // wantError
        }),
    [](const ::testing::TestParamInfo<DCMInitTestParams> &info) -> std::string {
        std::string name = info.param.name;
        std::replace(name.begin(), name.end(), ' ', '_');
        return name;
    });
