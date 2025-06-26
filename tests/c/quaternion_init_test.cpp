#include <gtest/gtest.h>
#include <kinematics-library/errors.h>
#include <kinematics-library/rotations.h>
#include <stdint.h>
#include <string>
#include <universal-constants/constants.h>

struct QInitTestParams
{
    std::string      name;
    bool             input1Null;
    quaternion_t     input1;
    precision_type_t input1a1;
    precision_type_t input1a2;
    precision_type_t input1a3;
    int              input1Rotation;
    quaternion_t     want;
    precision_type_t near;
    int              wantError;
};

class InitTest : public testing::TestWithParam<QInitTestParams>
{};

TEST_P(InitTest, Compute)
{
    QInitTestParams params = GetParam();

    quaternion_t *input1 = &params.input1;

    int error;

    if (params.input1Null)
    {
        input1 = NULL;
    }

    error = quaternion_init(input1, params.input1a1, params.input1a2, params.input1a3, params.input1Rotation);
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
    InitTest,
    testing::Values(
        QInitTestParams{
            "NULL input",          // name
            true,                  // input1Null
            {0.0, 0.0, 0.0, 0.0},  // input1
            0.0,                   // input1a1
            0.0,                   // input1a2
            0.0,                   // input1a3
            0,                     // input1Rotation
            {1.0, 2.0, 3.0, 0.0},  // want
            0.0,                   // near
            KL_ERROR_NULL_ARGUMENT // wantError
        },
        QInitTestParams{
            "Sequence body xyx",                                                               // name
            false,                                                                             // input1Null
            {0.0, 0.0, 0.0, 0.0},                                                              // input1
            3.03,                                                                              // input1a1
            -2.16,                                                                             // input1a2
            2.23,                                                                              // input1a3
            ROTATION_SEQ_BODY_XYX,                                                             // input1Rotation
            {0.4109822630778004, -0.2307466279496697, 0.8123369342780543, 0.3434505471433091}, // want
            1.0E-14,                                                                           // near
            KL_NO_ERROR                                                                        // wantError
        },
        QInitTestParams{
            "Sequence body xyz",                                                              // name
            false,                                                                            // input1Null
            {0.0, 0.0, 0.0, 0.0},                                                             // input1
            -0.45,                                                                            // input1a1
            -0.11,                                                                            // input1a2
            -2.38,                                                                            // input1a3
            ROTATION_SEQ_BODY_XYZ,                                                            // input1Rotation
            {-0.37313015873399, 0.03304611495798355, 0.2267278327659468, 0.8990418948962212}, // want
            1.0E-14,                                                                          // near
            KL_NO_ERROR                                                                       // wantError
        },
        QInitTestParams{
            "Sequence body xzx",                                                              // name
            false,                                                                            // input1Null
            {0.0, 0.0, 0.0, 0.0},                                                             // input1
            0.52,                                                                             // input1a1
            -1.56,                                                                            // input1a2
            -1.32,                                                                            // input1a3
            ROTATION_SEQ_BODY_XZX,                                                            // input1Rotation
            {0.654794729971696, -0.2768427714975188, 0.5595302452540809, -0.426060847904116}, // want
            1.0E-14,                                                                          // near
            KL_NO_ERROR                                                                       // wantError
        },
        QInitTestParams{
            "Sequence body xzy",                                                                 // name
            false,                                                                               // input1Null
            {0.0, 0.0, 0.0, 0.0},                                                                // input1
            3.03,                                                                                // input1a1
            1.45,                                                                                // input1a2
            -0.98,                                                                               // input1a3
            ROTATION_SEQ_BODY_XZY,                                                               // input1Rotation
            {-0.2747728232875256, 0.6768022424056369, -0.6038404380255478, -0.3190851699192839}, // want
            1.0E-14,                                                                             // near
            KL_NO_ERROR                                                                          // wantError
        },
        QInitTestParams{
            "Sequence body yxy",                                                                // name
            false,                                                                              // input1Null
            {0.0, 0.0, 0.0, 0.0},                                                               // input1
            2.39,                                                                               // input1a1
            2.00,                                                                               // input1a2
            -1.50,                                                                              // input1a3
            ROTATION_SEQ_BODY_YXY,                                                              // input1Rotation
            {-0.4876826238147061, 0.3075840880710135, -0.2325773853056927, 0.7832403507475174}, // want
            1.0E-14,                                                                            // near
            KL_NO_ERROR                                                                         // wantError
        },
        QInitTestParams{
            "Sequence body yxz",                                                             // name
            false,                                                                           // input1Null
            {0.0, 0.0, 0.0, 0.0},                                                            // input1
            -1.18,                                                                           // input1a1
            -2.13,                                                                           // input1a2
            -2.02,                                                                           // input1a3
            ROTATION_SEQ_BODY_YXZ,                                                           // input1Rotation
            {0.198027684241322, 0.1583369738893134, 0.7589281071049516, 0.5997854343177145}, // want
            1.0E-14,                                                                         // near
            KL_NO_ERROR                                                                      // wantError
        },
        QInitTestParams{
            "Sequence body yzx",                                                              // name
            false,                                                                            // input1Null
            {0.0, 0.0, 0.0, 0.0},                                                             // input1
            -0.18,                                                                            // input1a1
            1.23,                                                                             // input1a2
            1.26,                                                                             // input1a3
            ROTATION_SEQ_BODY_YZX,                                                            // input1Rotation
            {0.6878551218623439, 0.4373490267388047, 0.2792187413373984, 0.5075608876029747}, // want
            1.0E-14,                                                                          // near
            KL_NO_ERROR                                                                       // wantError
        },
        QInitTestParams{
            "Sequence body yzy",                                                                  // name
            false,                                                                                // input1Null
            {0.0, 0.0, 0.0, 0.0},                                                                 // input1
            -1.13,                                                                                // input1a1
            0.19,                                                                                 // input1a2
            0.97,                                                                                 // input1a3
            ROTATION_SEQ_BODY_YZY,                                                                // input1Rotation
            {0.9923070205071465, -0.08228131118770329, -0.0795543500436339, 0.04719818079753894}, // want
            1.0E-14,                                                                              // near
            KL_NO_ERROR                                                                           // wantError
        },
        QInitTestParams{
            "Sequence body zxy",                                                               // name
            false,                                                                             // input1Null
            {0.0, 0.0, 0.0, 0.0},                                                              // input1
            2.94,                                                                              // input1a1
            0.20,                                                                              // input1a2
            -1.10,                                                                             // input1a3
            ROTATION_SEQ_BODY_ZXY,                                                             // input1Rotation
            {0.1372741291471433, 0.5260005462337218, 0.03234541911945765, 0.8387091347366536}, // want
            1.0E-14,                                                                           // near
            KL_NO_ERROR                                                                        // wantError
        },
        QInitTestParams{
            "Sequence body zxz",                                                               // name
            false,                                                                             // input1Null
            {0.0, 0.0, 0.0, 0.0},                                                              // input1
            -0.48,                                                                             // input1a1
            -2.57,                                                                             // input1a2
            -1.47,                                                                             // input1a3
            ROTATION_SEQ_BODY_ZXZ,                                                             // input1Rotation
            {-0.1582053766458555, 0.8442749396128262, 0.4557631586305099, 0.2333470127883412}, // want
            1.0E-14,                                                                           // near
            KL_NO_ERROR                                                                        // wantError
        },
        QInitTestParams{
            "Sequence body zyx",                                                              // name
            false,                                                                            // input1Null
            {0.0, 0.0, 0.0, 0.0},                                                             // input1
            0.17,                                                                             // input1a1
            -0.27,                                                                            // input1a2
            2.36,                                                                             // input1a3
            ROTATION_SEQ_BODY_ZYX,                                                            // input1Rotation
            {0.3655312427515732, 0.917238173920486, 0.02669899703155687, 0.1560391182781152}, // want
            1.0E-14,                                                                          // near
            KL_NO_ERROR                                                                       // wantError
        },
        QInitTestParams{
            "Sequence body zyz",                                                               // name
            false,                                                                             // input1Null
            {0.0, 0.0, 0.0, 0.0},                                                              // input1
            2.88,                                                                              // input1a1
            -1.63,                                                                             // input1a2
            1.11,                                                                              // input1a3
            ROTATION_SEQ_BODY_ZYZ,                                                             // input1Rotation
            {-0.2823001156239072, 0.5631977472744021, -0.460860981557468, 0.6250776734627902}, // want
            1.0E-14,                                                                           // near
            KL_NO_ERROR                                                                        // wantError
        }),
    [](const ::testing::TestParamInfo<QInitTestParams> &info) -> std::string {
        std::string name = info.param.name;
        std::replace(name.begin(), name.end(), ' ', '_');
        return name;
    });
