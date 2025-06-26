/***********************************************************************************************************************
 * Coordinate transforms library.
 **********************************************************************************************************************/

#include <kinematics-library/transforms.h>
#include <kinematics-library/transforms.hpp>

KinematicVector aer_to_ned(const KinematicVector &aer)
{
    kvector_t tmp, out;
    kvector_init(&tmp, aer.get_x(), aer.get_y(), aer.get_z());
    transform_aer_to_ned(&tmp, &out);

    return KinematicVector(out.x, out.y, out.z);
}

KinematicVector ecef_to_lla(const KinematicVector &ecef)
{
    kvector_t tmp, out;
    kvector_init(&tmp, ecef.get_x(), ecef.get_y(), ecef.get_z());
    transform_ecef_to_lla(&tmp, &out);

    return KinematicVector(out.x, out.y, out.z);
}

KinematicVector ecef_to_ned(const KinematicVector &ecef, const KinematicVector &lla_ref)
{
    kvector_t tmp1, tmp2, out;
    kvector_init(&tmp1, ecef.get_x(), ecef.get_y(), ecef.get_z());
    kvector_init(&tmp2, lla_ref.get_x(), lla_ref.get_y(), lla_ref.get_z());
    transform_ecef_to_ned(&tmp1, &tmp2, &out);

    return KinematicVector(out.x, out.y, out.z);
}

KinematicVector ecef_to_ned_local(const KinematicVector &ecef, const KinematicVector &lla_ref)
{
    kvector_t tmp1, tmp2, out;
    kvector_init(&tmp1, ecef.get_x(), ecef.get_y(), ecef.get_z());
    kvector_init(&tmp2, lla_ref.get_x(), lla_ref.get_y(), lla_ref.get_z());
    transform_ecef_to_ned_local(&tmp1, &tmp2, &out);

    return KinematicVector(out.x, out.y, out.z);
}

KinematicVector lla_to_ecef(const KinematicVector &lla)
{
    kvector_t tmp, out;
    kvector_init(&tmp, lla.get_x(), lla.get_y(), lla.get_z());
    transform_lla_to_ecef(&tmp, &out);

    return KinematicVector(out.x, out.y, out.z);
}

KinematicVector lla_to_ned(const KinematicVector &lla, const KinematicVector &lla_ref)
{
    kvector_t tmp1, tmp2, out;
    kvector_init(&tmp1, lla.get_x(), lla.get_y(), lla.get_z());
    kvector_init(&tmp2, lla_ref.get_x(), lla_ref.get_y(), lla_ref.get_z());
    transform_lla_to_ned(&tmp1, &tmp2, &out);

    return KinematicVector(out.x, out.y, out.z);
}

KinematicVector ned_to_aer(const KinematicVector &ned)
{
    kvector_t tmp, out;
    kvector_init(&tmp, ned.get_x(), ned.get_y(), ned.get_z());
    transform_ned_to_aer(&tmp, &out);

    return KinematicVector(out.x, out.y, out.z);
}

KinematicVector ned_to_ecef(const KinematicVector &ned, const KinematicVector &lla_ref)
{
    kvector_t tmp1, tmp2, out;
    kvector_init(&tmp1, ned.get_x(), ned.get_y(), ned.get_z());
    kvector_init(&tmp2, lla_ref.get_x(), lla_ref.get_y(), lla_ref.get_z());
    transform_ned_to_ecef(&tmp1, &tmp2, &out);

    return KinematicVector(out.x, out.y, out.z);
}

KinematicVector ned_to_ecef_local(const KinematicVector &ned, const KinematicVector &lla_ref)
{
    kvector_t tmp1, tmp2, out;
    kvector_init(&tmp1, ned.get_x(), ned.get_y(), ned.get_z());
    kvector_init(&tmp2, lla_ref.get_x(), lla_ref.get_y(), lla_ref.get_z());
    transform_ned_to_ecef_local(&tmp1, &tmp2, &out);

    return KinematicVector(out.x, out.y, out.z);
}

KinematicVector ned_to_lla(const KinematicVector &ned, const KinematicVector &lla_ref)
{
    kvector_t tmp1, tmp2, out;
    kvector_init(&tmp1, ned.get_x(), ned.get_y(), ned.get_z());
    kvector_init(&tmp2, lla_ref.get_x(), lla_ref.get_y(), lla_ref.get_z());
    transform_ned_to_lla(&tmp1, &tmp2, &out);

    return KinematicVector(out.x, out.y, out.z);
}
