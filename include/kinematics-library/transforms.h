#ifndef KINEMATICS_LIBRARY_TRANSFORMS_H_
#define KINEMATICS_LIBRARY_TRANSFORMS_H_

#include <kinematics-library/kinematic_vector.h>

#ifdef __cplusplus
extern "C" {
#endif

/***********************************************************************************************************************
 * @brief Converts local Azimuth-Elevation-Range (AER) coordinates to local North-East-Down (NED) coordinate.
 *
 * @param aer Vector represented in AER coordinates [radians-radians-meters]
 * @param ned Vector represented in NED coordinates [meters-meters-meters]
 * @return Function execution status
 **********************************************************************************************************************/
int transform_aer_to_ned(const kvector_t *aer, kvector_t *ned);

/***********************************************************************************************************************
 * @brief Converts an ECEF vector to LLA using an elliptical earth model. The altitude corresponds to height above the
 * ellipsoid.
 *
 * @param ecef Vector represented in ECEF coordinates [meters]
 * @param lla Vector represented in LLA coordinates [radians-radians-meters]
 * @return Function execution status
 **********************************************************************************************************************/
int transform_ecef_to_lla(const kvector_t *ecef, kvector_t *lla);

/***********************************************************************************************************************
 * @brief Converts an ECEF position vector to NED using an elliptical earth model. The reference altitude is expected as
 * height above the ellipsoid.
 *
 * @param ecef Vector represented in ECEF coordinates [meters]
 * @param lla_ref Reference latitude-longitude-altitude [radians-radians-meters]
 * @param ned Vector represented in NED coordinates [meters]
 * @return Function execution status
 **********************************************************************************************************************/
int transform_ecef_to_ned(const kvector_t *ecef, const kvector_t *lla_ref, kvector_t *ned);

/***********************************************************************************************************************
 * @brief Converts a local ECEF vector to NED using an elliptical earth model. The units of the returned NED vector
 * match the units of the given ECEF vector. The reference altitude is expected as height above the ellipsoid.
 *
 * @param ecef Vector represented in ECEF coordinates
 * @param lla_ref Reference latitude-longitude-altitude [radians-radians-meters]
 * @param ned Vector represented in NED coordinates
 * @return Function execution status
 **********************************************************************************************************************/
int transform_ecef_to_ned_local(const kvector_t *ecef, const kvector_t *lla_ref, kvector_t *ned);

/***********************************************************************************************************************
 * @brief Converts a latitude-longitude-altitude vector to ECEF using an elliptical earth model. The altitude is
 * expected as height above the ellipsoid.
 *
 * @param lla Latitude-longitude-altitude point [radians-radians-meters]
 * @param ecef Vector represented in ECEF coordinates [meters]
 * @return Function execution status
 **********************************************************************************************************************/
int transform_lla_to_ecef(const kvector_t *lla, kvector_t *ecef);

/***********************************************************************************************************************
 * @brief Converts a latitude-longitude-altitude vector to NED using an elliptical earth model. The altitude is
 * expected as height above the ellipsoid.
 *
 * @param lla Latitude-longitude-altitude point [radians-radians-meters]
 * @param lla_ref Reference latitude-longitude-altitude [radians-radians-meters]
 * @param ned Vector represented in NED coordinates [meters]
 * @return Function execution status
 **********************************************************************************************************************/
int transform_lla_to_ned(const kvector_t *lla, const kvector_t *lla_ref, kvector_t *ned);

/***********************************************************************************************************************
 * @brief Converts local North-East-Down (NED) coordinates to local Azimuth-Elevation-Range (AER) coordinate.
 *
 * @param ned Vector represented in NED coordinates [meters-meters-meters]
 * @param aer Vector represented in AER coordinates [radians-radians-meters]
 * @return Function execution status
 **********************************************************************************************************************/
int transform_ned_to_aer(const kvector_t *ned, kvector_t *aer);

/***********************************************************************************************************************
 * @brief Converts a NED position vector to ECEF using an elliptical earth model. The reference altitude is expected as
 * height above the ellipsoid.
 *
 * @param ned Vector represented in NED coordinates [meters]
 * @param lla_ref Reference latitude-longitude-altitude [radians-radians-meters]
 * @param ecef Vector represented in ECEF coordinates [meters]
 * @return Function execution status
 **********************************************************************************************************************/
int transform_ned_to_ecef(const kvector_t *ned, const kvector_t *lla_ref, kvector_t *ecef);

/***********************************************************************************************************************
 * @brief Converts a NED vector to local ECEF using an elliptical earth model. The units of the returned ECEF vector
 * match the units of the given NED vector. The reference altitude is expected as height above the ellipsoid.
 *
 * @param ned Vector represented in NED coordinates
 * @param lla_ref Reference latitude-longitude-altitude [radians-radians-meters]
 * @param ecef Vector represented in local ECEF coordinates
 * @return Function execution status
 **********************************************************************************************************************/
int transform_ned_to_ecef_local(const kvector_t *ned, const kvector_t *lla_ref, kvector_t *ecef);

/***********************************************************************************************************************
 * @brief Converts a NED position vector to LLA using an elliptical earth model. The altitude corresponds to height
 * above the ellipsoid.
 *
 * @param ned Vector represented in NED coordinates [meters]
 * @param lla_ref Reference latitude-longitude-altitude [radians-radians-meters]
 * @param lla Vector represented in LLA coordinates [radians-radians-meters]
 * @return Function execution status
 **********************************************************************************************************************/
int transform_ned_to_lla(const kvector_t *ned, const kvector_t *lla_ref, kvector_t *lla);

#ifdef __cplusplus
}
#endif

#endif // KINEMATICS_LIBRARY_TRANSFORMS_H_