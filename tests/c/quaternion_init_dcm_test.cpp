#include <gtest/gtest.h>
#include <kinematics-library/errors.h>
#include <kinematics-library/rotations.h>
#include <stdint.h>
#include <string>
#include <universal-constants/constants.h>

struct QInitDCMTestParams
{
    std::string      name;
    bool             input1Null;
    bool             input2Null;
    quaternion_t     input1;
    dcm_t            input2;
    quaternion_t     want;
    precision_type_t near;
    int              wantError;
};

class InitDCMTest : public testing::TestWithParam<QInitDCMTestParams>
{};

TEST_P(InitDCMTest, Compute)
{
    QInitDCMTestParams params = GetParam();

    quaternion_t *input1 = &params.input1;
    dcm_t        *input2 = &params.input2;

    int error;

    if (params.input1Null)
    {
        input1 = NULL;
    }

    if (params.input2Null)
    {
        input2 = NULL;
    }

    error = quaternion_init_dcm(input1, input2);
    EXPECT_EQ(error, params.wantError);
    if (error == KL_NO_ERROR)
    {
        if (params.near == 0)
        {
            EXPECT_DOUBLE_EQ(params.want.a, params.input1.a);
            EXPECT_DOUBLE_EQ(params.want.b, params.input1.b);
            EXPECT_DOUBLE_EQ(params.want.c, params.input1.c);
            EXPECT_DOUBLE_EQ(params.want.d, params.input1.d);
        }
        else
        {
            EXPECT_NEAR(params.want.a, params.input1.a, params.near);
            EXPECT_NEAR(params.want.b, params.input1.b, params.near);
            EXPECT_NEAR(params.want.c, params.input1.c, params.near);
            EXPECT_NEAR(params.want.d, params.input1.d, params.near);
        }
    }
}

INSTANTIATE_TEST_SUITE_P(
    Quaternion,
    InitDCMTest,
    testing::Values(
        QInitDCMTestParams{
            "NULL input 1",                                // name
            true,                                          // input1Null
            false,                                         // input2Null
            {0.0, 0.0, 0.0, 0.0},                          // input1
            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, // input2
            {1.0, 2.0, 3.0, 0.0},                          // want
            0.0,                                           // near
            KL_ERROR_NULL_ARGUMENT                         // wantError
        },
        QInitDCMTestParams{
            "NULL input 2",                                // name
            false,                                         // input1Null
            true,                                          // input2Null
            {0.0, 0.0, 0.0, 0.0},                          // input1
            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, // input2
            {1.0, 2.0, 3.0, 0.0},                          // want
            0.0,                                           // near
            KL_ERROR_NULL_ARGUMENT                         // wantError
        },
        QInitDCMTestParams{
            "Sequence body xyx",  // name
            false,                // input1Null
            false,                // input2Null
            {0.0, 0.0, 0.0, 0.0}, // input1
            {-0.5556991462506127,
             -0.09258385044673502,
             -0.8262122545041296,
             -0.6571921829277988,
             0.6575954307136367,
             0.3683295863803792,
             0.5092120320209791,
             0.7476606717896851,
             -0.4262706022048224},                                                             // input2
            {0.4109822630778004, -0.2307466279496697, 0.8123369342780543, 0.3434505471433091}, // want
            1.0E-14,                                                                           // near
            KL_NO_ERROR                                                                        // wantError
        },
        QInitDCMTestParams{
            "Sequence body xyz",  // name
            false,                // input1Null
            false,                // input2Null
            {0.0, 0.0, 0.0, 0.0}, // input1
            {-0.7193636778586622,
             -0.6559343418507525,
             0.2286176680803103,
             0.6859042379537842,
             -0.6187367489848085,
             0.3830146365515926,
             -0.1097783008371748,
             0.4323366450308489,
             0.8950048882708811},                                                             // input2
            {-0.37313015873399, 0.03304611495798355, 0.2267278327659468, 0.8990418948962212}, // want
            1.0E-14,                                                                          // near
            KL_NO_ERROR                                                                       // wantError
        },
        QInitDCMTestParams{
            "Sequence body xzx",  // name
            false,                // input1Null
            false,                // input2Null
            {0.0, 0.0, 0.0, 0.0}, // input1
            {0.01079611705826739,
             -0.8677686033754274,
             -0.4968511797835689,
             0.2481609880441226,
             0.4836604675055965,
             -0.8393382370565711,
             0.9686586436250022,
             -0.1142374858272359,
             0.2205679690309612},                                                             // input2
            {0.654794729971696, -0.2768427714975188, 0.5595302452540809, -0.426060847904116}, // want
            1.0E-14,                                                                          // near
            KL_NO_ERROR                                                                       // wantError
        },
        QInitDCMTestParams{
            "Sequence body xzy",  // name
            false,                // input1Null
            false,                // input2Null
            {0.0, 0.0, 0.0, 0.0}, // input1
            {0.06712275948539267,
             -0.6420092590059827,
             -0.7637530009824218,
             -0.9927129910375885,
             -0.1197532419754337,
             0.01341933163678318,
             -0.1000772330965983,
             0.7572867834492837,
             -0.6453690998403679},                                                               // input2
            {-0.2747728232875256, 0.6768022424056369, -0.6038404380255478, -0.3190851699192839}, // want
            1.0E-14,                                                                             // near
            KL_NO_ERROR                                                                          // wantError
        },
        QInitDCMTestParams{
            "Sequence body yxy",  // name
            false,                // input1Null
            false,                // input2Null
            {0.0, 0.0, 0.0, 0.0}, // input1
            {-0.3351153743894538,
             -0.9070196245905846,
             0.2549766390385037,
             0.6208712127298156,
             -0.4161468365471424,
             -0.6643348159137937,
             0.7086724370618802,
             -0.06432115545729157,
             0.7025995772197806},                                                               // input2
            {-0.4876826238147061, 0.3075840880710135, -0.2325773853056927, 0.7832403507475174}, // want
            1.0E-14,                                                                            // near
            KL_NO_ERROR                                                                         // wantError
        },
        QInitDCMTestParams{
            "Sequence body yxz",  // name
            false,                // input1Null
            false,                // input2Null
            {0.0, 0.0, 0.0, 0.0}, // input1
            {-0.8714288779471885,
             0.4778810009563105,
             -0.1106411298057783,
             0.002784518557860272,
             0.2303736709597711,
             0.9730982572098874,
             0.490513972416796,
             0.8476778401335698,
             -0.2020849381086598},                                                           // input2
            {0.198027684241322, 0.1583369738893134, 0.7589281071049516, 0.5997854343177145}, // want
            1.0E-14,                                                                         // near
            KL_NO_ERROR                                                                      // wantError
        },
        QInitDCMTestParams{
            "Sequence body yzx",  // name
            false,                // input1Null
            false,                // input2Null
            {0.0, 0.0, 0.0, 0.0}, // input1
            {0.3288376797232791,
             0.9424888019316975,
             0.05983843770991672,
             -0.4540246228471165,
             0.1022155483726017,
             0.8851065605447865,
             0.8280866031054624,
             -0.3182245117904053,
             0.4615254465931586},                                                             // input2
            {0.6878551218623439, 0.4373490267388047, 0.2792187413373984, 0.5075608876029747}, // want
            1.0E-14,                                                                          // near
            KL_NO_ERROR                                                                       // wantError
        },
        QInitDCMTestParams{
            "Sequence body yzy",  // name
            false,                // input1Null
            false,                // input2Null
            {0.0, 0.0, 0.0, 0.0}, // input1
            {0.9828868742370757,
             0.1067618447856784,
             0.1501176237169703,
             -0.0805784998565755,
             0.9820042351172703,
             -0.1708062866893626,
             -0.1656517365237534,
             0.1557870043030021,
             0.9738017824367347},                                                                 // input2
            {0.9923070205071465, -0.08228131118770329, -0.0795543500436339, 0.04719818079753894}, // want
            1.0E-14,                                                                              // near
            KL_NO_ERROR                                                                           // wantError
        },
        QInitDCMTestParams{
            "Sequence body zxy",  // name
            false,                // input1Null
            false,                // input2Null
            {0.0, 0.0, 0.0, 0.0}, // input1
            {-0.4089584776574394,
             0.2642935484074433,
             0.8734425475223383,
             -0.1962387159074697,
             -0.96021917465776,
             0.1986693307950612,
             0.8912033044884298,
             -0.09015573686556229,
             0.4445543984476258},                                                              // input2
            {0.1372741291471433, 0.5260005462337218, 0.03234541911945765, 0.8387091347366536}, // want
            1.0E-14,                                                                           // near
            KL_NO_ERROR                                                                        // wantError
        },
        QInitDCMTestParams{
            "Sequence body zxz",  // name
            false,                // input1Null
            false,                // input2Null
            {0.0, 0.0, 0.0, 0.0}, // input1
            {0.4756582297157966,
             0.6957453223663198,
             0.5382264346063096,
             0.8434123305557789,
             -0.5345020040709673,
             -0.05443572641739133,
             0.2498097059165311,
             0.4798396128390249,
             -0.8410404608462014},                                                             // input2
            {-0.1582053766458555, 0.8442749396128262, 0.4557631586305099, 0.2333470127883412}, // want
            1.0E-14,                                                                           // near
            KL_NO_ERROR                                                                        // wantError
        },
        QInitDCMTestParams{
            "Sequence body zyx",  // name
            false,                // input1Null
            false,                // input2Null
            {0.0, 0.0, 0.0, 0.0}, // input1
            {0.9498779142489947,
             0.1630530242095858,
             0.2667314366888311,
             -0.06509566707865096,
             -0.7313481482599988,
             0.6788905951361062,
             0.3057687069494934,
             -0.6622262433132495,
             -0.6840774082789577},                                                            // input2
            {0.3655312427515732, 0.917238173920486, 0.02669899703155687, 0.1560391182781152}, // want
            1.0E-14,                                                                          // near
            KL_NO_ERROR                                                                       // wantError
        },
        QInitDCMTestParams{
            "Sequence body zyz",  // name
            false,                // input1Null
            false,                // input2Null
            {0.0, 0.0, 0.0, 0.0}, // input1
            {-0.2062298843675349,
             -0.8720307322246085,
             0.4438824583710945,
             -0.1661927342547344,
             -0.4158276007932318,
             -0.8941293986328153,
             0.9642868918919765,
             -0.2581662419340645,
             -0.05916909371414814},                                                            // input2
            {-0.2823001156239072, 0.5631977472744021, -0.460860981557468, 0.6250776734627902}, // want
            1.0E-14,                                                                           // near
            KL_NO_ERROR                                                                        // wantError
        }),
    [](const ::testing::TestParamInfo<QInitDCMTestParams> &info) -> std::string {
        std::string name = info.param.name;
        std::replace(name.begin(), name.end(), ' ', '_');
        return name;
    });
