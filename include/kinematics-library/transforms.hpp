/***********************************************************************************************************************
 * Coordinate transforms library.
 **********************************************************************************************************************/

#pragma once

#include <kinematics-library/kinematic_vector.hpp>

/***********************************************************************************************************************
 * @brief Converts local Azimuth-Elevation-Range (AER) coordinates to local North-East-Down (NED) coordinate.
 *
 * @param aer Vector represented in AER coordinates [radians-radians-meters]
 * @return Vector represented in NED coordinates [meters-meters-meters]
 **********************************************************************************************************************/
KinematicVector aer_to_ned(const KinematicVector &aer);

/***********************************************************************************************************************
 * @brief Converts an ECEF vector to LLA using an elliptical earth model. The altitude corresponds to height above the
 * ellipsoid.
 *
 * @param ecef Vector represented in ECEF coordinates [meters]
 * @return Vector represented in LLA coordinates [radians-radians-meters]
 **********************************************************************************************************************/
KinematicVector ecef_to_lla(const KinematicVector &ecef);

/***********************************************************************************************************************
 * @brief Converts an ECEF position vector to NED using an elliptical earth model. The reference altitude is expected as
 * height above the ellipsoid.
 *
 * @param ecef Vector represented in ECEF coordinates [meters]
 * @param lla_ref Reference latitude-longitude-altitude [radians-radians-meters]
 * @return Vector represented in NED coordinates [meters]
 **********************************************************************************************************************/
KinematicVector ecef_to_ned(const KinematicVector &ecef, const KinematicVector &lla_ref);

/***********************************************************************************************************************
 * @brief Converts a local ECEF vector to NED using an elliptical earth model. The units of the returned NED vector
 * match the units of the given ECEF vector. The reference altitude is expected as height above the ellipsoid.
 *
 * @param ecef Vector represented in ECEF coordinates
 * @param lla_ref Reference latitude-longitude-altitude [radians-radians-meters]
 * @return Vector represented in NED coordinates
 **********************************************************************************************************************/
KinematicVector ecef_to_ned_local(const KinematicVector &ecef, const KinematicVector &lla_ref);

/***********************************************************************************************************************
 * @brief Converts a latitude-longitude-altitude vector to ECEF using an elliptical earth model. The altitude is
 * expected as height above the ellipsoid.
 *
 * @param lla Latitude-longitude-altitude point [radians-radians-meters]
 * @return Vector represented in ECEF coordinates [meters]
 **********************************************************************************************************************/
KinematicVector lla_to_ecef(const KinematicVector &lla);

/***********************************************************************************************************************
 * @brief Converts a latitude-longitude-altitude vector to NED using an elliptical earth model. The altitude is
 * expected as height above the ellipsoid.
 *
 * @param lla Latitude-longitude-altitude point [radians-radians-meters]
 * @param lla_ref Reference latitude-longitude-altitude [radians-radians-meters]
 * @return Vector represented in NED coordinates [meters]
 **********************************************************************************************************************/
KinematicVector lla_to_ned(const KinematicVector &lla, const KinematicVector &lla_ref);

/***********************************************************************************************************************
 * @brief Converts local North-East-Down (NED) coordinates to local Azimuth-Elevation-Range (AER) coordinate.
 *
 * @param ned Vector represented in NED coordinates [meters-meters-meters]
 * @return Vector represented in AER coordinates [radians-radians-meters]
 **********************************************************************************************************************/
KinematicVector ned_to_aer(const KinematicVector &ned);

/***********************************************************************************************************************
 * @brief Converts a NED position vector to ECEF using an elliptical earth model. The reference altitude is expected as
 * height above the ellipsoid.
 *
 * @param ned Vector represented in NED coordinates [meters]
 * @param lla_ref Reference latitude-longitude-altitude [radians-radians-meters]
 * @return Vector represented in ECEF coordinates [meters]
 **********************************************************************************************************************/
KinematicVector ned_to_ecef(const KinematicVector &ned, const KinematicVector &lla_ref);

/***********************************************************************************************************************
 * @brief Converts a NED vector to local ECEF using an elliptical earth model. The units of the returned ECEF vector
 * match the units of the given NED vector. The reference altitude is expected as height above the ellipsoid.
 *
 * @param ned Vector represented in NED coordinates
 * @param lla_ref Reference latitude-longitude-altitude [radians-radians-meters]
 * @return Vector represented in local ECEF coordinates
 **********************************************************************************************************************/
KinematicVector ned_to_ecef_local(const KinematicVector &ned, const KinematicVector &lla_ref);

/***********************************************************************************************************************
 * @brief Converts a NED position vector to LLA using an elliptical earth model. The altitude corresponds to height
 * above the ellipsoid.
 *
 * @param ned Vector represented in NED coordinates [meters]
 * @param lla_ref Reference latitude-longitude-altitude [radians-radians-meters]
 * @return Vector represented in LLA coordinates [radians-radians-meters]
 **********************************************************************************************************************/
KinematicVector ned_to_lla(const KinematicVector &ned, const KinematicVector &lla_ref);
