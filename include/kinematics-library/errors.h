#ifndef KINEMATICS_LIBRARY_ERRORS_H_
#define KINEMATICS_LIBRARY_ERRORS_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define KL_NO_ERROR               (int)0
#define KL_ERROR_NULL_ARGUMENT    (int)1
#define KL_ERROR_INVALID_ARGUMENT (int)2
#define KL_ERROR_NOT_IMPLEMENTED  (int)3
#define KL_ERROR_UNSUPPORTED      (int)4

#ifdef __cplusplus
}
#endif

#endif // KINEMATICS_LIBRARY_ERRORS_H_
