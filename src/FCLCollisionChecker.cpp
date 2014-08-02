#include <boost/range/adaptor/map.hpp>
#include <boost/format.hpp>
#include <boost/make_shared.hpp>
#include <boost/unordered_map.hpp>
#include <fcl/shape/geometric_shapes.h>
#include "FCLCollisionChecker.h"

using std::make_pair;
using boost::adaptors::map_values;
using boost::dynamic_pointer_cast;
using boost::format;
using boost::make_shared;
using boost::unordered_map;
using boost::str;
using OpenRAVE::KinBodyPtr;
using OpenRAVE::TriMesh;
using OpenRAVE::UserData;
using OpenRAVE::UserDataPtr;
using OpenRAVE::Vector;

typedef boost::shared_ptr<fcl::CollisionObject> CollisionObjectPtr;
typedef OpenRAVE::KinBody::LinkPtr LinkPtr;
typedef OpenRAVE::KinBody::Link::Geometry Geometry;
typedef OpenRAVE::KinBody::Link::GeometryPtr GeometryPtr;

namespace or_fcl {

/*
 * FCLUserData
 */
FCLUserData::~FCLUserData()
{
}


/*
 * FCLCollisionChecker
 */
FCLCollisionChecker::FCLCollisionChecker(OpenRAVE::EnvironmentBasePtr env) : OpenRAVE::CollisionCheckerBase(env)
    // Create a unique UserData key for this collision checker.
    , user_data_(str(format("or_fcl[%p]") % this))
{
}

FCLCollisionChecker::~FCLCollisionChecker()
{
}

bool FCLCollisionChecker::InitEnvironment()
{
    return true;
}

void FCLCollisionChecker::DestroyEnvironment()
{
}

bool FCLCollisionChecker::InitKinBody(KinBodyPtr body)
{
    //body->SetUserData(user_data_, NULL);
    //
    //

#if 0
    for (GeometryPtr const &geom : body->GetGeometries()) {
        CollisionGeometryPtr const fcl_geom = ConvertGeometryToFCL(geom);
    }
#endif

    return true;
}

void FCLCollisionChecker::RemoveKinBody(KinBodyPtr body)
{
    body->RemoveUserData(user_data_);
}

void FCLCollisionChecker::Synchronize()
{
    broad_phase_->clear();

    std::vector<KinBodyPtr> bodies;
    GetEnv()->GetBodies(bodies);
    for (KinBodyPtr const &body : bodies) {
        // Skip objects that are disabled.
        if (!body->IsEnabled()) {
            continue;
        }

        // Synchronize this object with the FCL environment.
        FCLUserDataPtr const collision_data = Synchronize(body);

        // Register the object's geometry with the broad-phase checker.
        for (CollisionObjectPtr const &collision_object
                : collision_data->geometries | map_values) {
            // TODO: Check if this geometry is enabled.
            broad_phase_->registerObject(collision_object.get());
        }
    }

    broad_phase_->setup();
}

FCLUserDataPtr FCLCollisionChecker::Synchronize(KinBodyConstPtr const &body)
{
    UserDataPtr const user_data = body->GetUserData(user_data_);
    auto const collision_data = dynamic_pointer_cast<FCLUserData>(user_data);
    BOOST_ASSERT(collision_data);

    for (LinkPtr const &link : body->GetLinks()) {
        OpenRAVE::Transform const link_pose = link->GetTransform();

        for (GeometryPtr const &geom : link->GetGeometries()) {
            auto const result = collision_data->geometries.insert(
                make_pair(geom.get(), CollisionObjectPtr())
            );
            CollisionObjectPtr &fcl_object = result.first->second;

            // TODO: Check if the geometry is enabled.

            // Convert the OpenRAVE geometry into FCL geometry. This is
            // necessary if: (1) there is no existing FCL geometry for this
            // shape or (2) the geometric is dynamic and could be modified at
            // any time.
            if (result.second || geom->IsModifiable()) {
                CollisionGeometryPtr const fcl_geom = ConvertGeometryToFCL(geom);
                fcl_object = make_shared<fcl::CollisionObject>(fcl_geom);
                RAVELOG_VERBOSE(
                    "Created FCL geometry for KinBody[%s].Link[%s].Geometry[%p].\n",
                    body->GetName().c_str(), link->GetName().c_str(), geom.get()
                );
            }

            // Update the pose of the FCL geometry to match the environment. We
            // need to check for NULL here because some OpenRAVE geometry may
            // not map to no FCL geometry (e.g. GT_None). We retain these NULLs
            // to simplify bookkeeping when geometry is modified.
            if (fcl_object) {
                OpenRAVE::Transform const &pose = link_pose * geom->GetTransform();
                fcl_object->setTranslation(ConvertVectorToFCL(pose.trans));
                fcl_object->setQuatRotation(ConvertQuaternionToFCL(pose.rot));
            }
        }

        // One or more geometries were dynamically removed from the OpenRAVE
        // environment. Delete the associated FCL geometries.
        size_t const num_or_geometries = link->GetGeometries().size();
        size_t const num_fcl_geometries = collision_data->geometries.size();
        BOOST_ASSERT(num_fcl_geometries >= num_or_geometries);

        if (num_fcl_geometries > num_or_geometries) {
            RAVELOG_WARN("Removing geometries from the environment may leak"
                          " memory inside the FCL environment. Garbage"
                          " collection is not currently implemented.");
        }
    }
    return collision_data;
}

fcl::Vec3f FCLCollisionChecker::ConvertVectorToFCL(Vector const &v) const
{
    return fcl::Vec3f(v.x, v.y, v.z);
}

fcl::Quaternion3f FCLCollisionChecker::ConvertQuaternionToFCL(Vector const &v) const
{
    // OpenRAVE and FCL both use the (scalar, vector) convention. Remember to
    // never use the named attributes on OpenRAVE::Vector to access quaternion
    // elements: they're wrong!
    return fcl::Quaternion3f(v[0], v[1], v[2], v[3]);
}

auto FCLCollisionChecker::ConvertGeometryToFCL(
        GeometryConstPtr const &geom) const -> CollisionGeometryPtr
{
    switch (geom->GetType()) {
    case OpenRAVE::GT_None:
        return CollisionGeometryPtr();

    case OpenRAVE::GT_Box: {
        // OpenRAVE's extents are actually half-extents.
        OpenRAVE::Vector const extents = geom->GetBoxExtents();
        return make_shared<fcl::Box>(
            2 * extents[0], 2 * extents[1], 2 * extents[2]
        );
    }

    case OpenRAVE::GT_Cylinder:
        return make_shared<fcl::Cylinder>(
            geom->GetCylinderRadius(), geom->GetCylinderHeight()
        );

    case OpenRAVE::GT_Sphere:
        return make_shared<fcl::Sphere>(geom->GetSphereRadius());

    case OpenRAVE::GT_TriMesh: {
        TriMesh const &mesh = geom->GetCollisionMesh();
        if (mesh.vertices.empty() || mesh.indices.empty()) {
            return CollisionGeometryPtr();
        }

        BOOST_ASSERT(mesh.indices.size() % 3 == 0);
        size_t const num_points = mesh.vertices.size();
        size_t const num_triangles = mesh.indices.size() / 3;

        // Convert the OpenRAVE mesh to FCL data types.
        // TODO: Is it more efficient to use addSubModel or manually call
        // addVertex and addTriangle? My guess is addSubModel since it doesn't
        // require duplicating verticies, but I should verify.
        std::vector<fcl::Vec3f> fcl_points(num_points);
        for (size_t ipoint = 0; ipoint < num_points; ++ipoint) {
            fcl_points[ipoint] = ConvertVectorToFCL(mesh.vertices[ipoint]);
        }

        std::vector<fcl::Triangle> fcl_triangles(num_triangles);
        for (size_t itri = 0; itri < num_triangles; ++itri) {
            int const *const tri_indices = &mesh.indices[3 * itri];
            fcl_triangles[itri] = fcl::Triangle(
                tri_indices[0], tri_indices[2], tri_indices[2]
            );
        }

        // Create the FCL mesh.
        auto const model = boost::make_shared<BVHModel>();
        model->beginModel(mesh.indices.size(), mesh.vertices.size());
        model->addSubModel(fcl_points, fcl_triangles);
        model->endModel();
        return model;
    }

    default:
        throw OpenRAVE::openrave_exception(
            str(format("Unknown geometry type %d.") % geom->GetType()),
            OpenRAVE::ORE_InvalidArguments
        );
    }
}

}
