#include <boost/format.hpp>
#include <boost/make_shared.hpp>
#include <fcl/shape/geometric_shapes.h>
#include "FCLCollisionChecker.h"

using boost::format;
using boost::make_shared;
using boost::str;
using OpenRAVE::KinBodyPtr;
using OpenRAVE::TriMesh;

namespace or_fcl {

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

    return true;
}

void FCLCollisionChecker::RemoveKinBody(KinBodyPtr body)
{
    body->RemoveUserData(user_data_);
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
            OpenRAVE::Vector const &point = mesh.vertices[ipoint];
            fcl_points[ipoint] = fcl::Vec3f(point.x, point.y, point.z);
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
