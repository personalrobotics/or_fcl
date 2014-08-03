#include <boost/range/adaptor/map.hpp>
#include <boost/format.hpp>
#include <boost/make_shared.hpp>
#include <boost/unordered_map.hpp>
#include <fcl/collision.h>
#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#include <fcl/shape/geometric_shapes.h>
#include "FCLCollisionChecker.h"

using std::make_pair;
using boost::adaptors::map_values;
using boost::dynamic_pointer_cast;
using boost::format;
using boost::make_shared;
using boost::unordered_map;
using boost::str;
using OpenRAVE::CollisionAction;
using OpenRAVE::CollisionReport;
using OpenRAVE::CollisionReportPtr;
using OpenRAVE::EnvironmentBasePtr;
using OpenRAVE::KinBodyPtr;
using OpenRAVE::TriMesh;
using OpenRAVE::UserData;
using OpenRAVE::UserDataPtr;
using OpenRAVE::Vector;

typedef boost::shared_ptr<fcl::CollisionObject> CollisionObjectPtr;
typedef OpenRAVE::KinBody::Link Link;
typedef OpenRAVE::KinBody::LinkPtr LinkPtr;
typedef OpenRAVE::KinBody::Link::Geometry Geometry;
typedef OpenRAVE::KinBody::Link::GeometryPtr GeometryPtr;
typedef OpenRAVE::EnvironmentBase::CollisionCallbackFn CollisionCallbackFn;

namespace {

/*
 * CollisionQuery
 */
struct CollisionQuery {
    EnvironmentBasePtr env;
    fcl::CollisionRequest request;
    fcl::CollisionResult result;
    CollisionReportPtr report;
    std::list<CollisionCallbackFn> callbacks;
    bool is_collision;
};

}

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
FCLCollisionChecker::FCLCollisionChecker(OpenRAVE::EnvironmentBasePtr env)
    : OpenRAVE::CollisionCheckerBase(env)
    // Create a unique UserData key for this collision checker.
    , user_data_(str(format("or_fcl[%p]") % this))
    , num_contacts_(100)
    , options_(0)
    , manager1_(make_shared<fcl::DynamicAABBTreeCollisionManager>())
    , manager2_(make_shared<fcl::DynamicAABBTreeCollisionManager>())
{
}

FCLCollisionChecker::~FCLCollisionChecker()
{
}

bool FCLCollisionChecker::InitEnvironment()
{
    std::vector<KinBodyPtr> bodies;
    GetEnv()->GetBodies(bodies);

    for (KinBodyPtr const &body : bodies) {
        InitKinBody(body);
    }
    return true;
}

void FCLCollisionChecker::DestroyEnvironment()
{
    manager1_->clear();
    manager2_->clear();

    std::vector<KinBodyPtr> bodies;
    GetEnv()->GetBodies(bodies);

    for (KinBodyPtr const &body : bodies) {
        RemoveKinBody(body);
    }
}

bool FCLCollisionChecker::CheckCollision(
        KinBodyConstPtr body1, CollisionReportPtr report)
{
    CollisionGroup group1, group2;

    // TODO: Implement CO_ActiveDOFs

    // Group 1: Argument.
    manager1_->clear();
    Synchronize(body1, &group1);
    manager1_->registerObjects(group1);
    manager1_->setup();

    // Group 2: Everything else in the environment.
    manager2_->clear();

    std::vector<KinBodyPtr> bodies;
    GetEnv()->GetBodies(bodies);

    for (KinBodyPtr const &body2 : bodies) {
        if (body2 != body1 && body2->IsEnabled()) {
            Synchronize(body2, &group2);
        }
    }

    manager2_->registerObjects(group2);
    manager2_->setup();

    return RunCheck(report);
} 

bool FCLCollisionChecker::CheckCollision(
        LinkConstPtr link1, LinkConstPtr link2, CollisionReportPtr report)
{
    CollisionGroup group1, group2;

    // Group 1: link1.
    manager1_->clear();
    Synchronize(link1, &group1);
    manager1_->registerObjects(group1);
    manager1_->setup();

    // Group 2: link2.
    manager2_->clear();
    Synchronize(link2, &group2);
    manager2_->registerObjects(group2);
    manager2_->setup();

    return RunCheck(report);
}

bool FCLCollisionChecker::InitKinBody(KinBodyPtr body)
{
    body->SetUserData(user_data_, make_shared<FCLUserData>());
    return true;
}

void FCLCollisionChecker::RemoveKinBody(KinBodyPtr body)
{
    body->RemoveUserData(user_data_);
}

auto FCLCollisionChecker::GetCollisionLink(
        fcl::CollisionObject const &o) -> LinkConstPtr
{
    void *user_data = o.getUserData();
    Link const *link_raw = static_cast<Link const *>(user_data);

    // Reconstruct a shared_ptr.
    size_t const index = link_raw->GetIndex();
    return link_raw->GetParent()->GetLinks()[index];
}

FCLUserDataPtr FCLCollisionChecker::GetCollisionData(
        KinBodyConstPtr const &body) const
{
    UserDataPtr const user_data = body->GetUserData(user_data_);
    return dynamic_pointer_cast<FCLUserData>(user_data);
}

bool FCLCollisionChecker::RunCheck(CollisionReportPtr report)
{
    CollisionQuery query;
    query.env = GetEnv();
    query.report = report;
    query.env->GetRegisteredCollisionCallbacks(query.callbacks);

    if (options_ & OpenRAVE::CO_Distance) {
        throw OpenRAVE::openrave_exception(
            "or_fcl does not currently support CO_Distance.",
            OpenRAVE::ORE_NotImplemented
        );
    }

    if (options_ | OpenRAVE::CO_Contacts) {
        query.request.enable_contact = true;
        query.request.num_max_contacts = num_contacts_;
    }

    manager1_->collide(manager2_.get(), &query,
                       &FCLCollisionChecker::NarrowPhaseCheckCollision);

    return query.result.isCollision();
    
}

void FCLCollisionChecker::Synchronize(KinBodyConstPtr const &body,
                                      CollisionGroup *group)
{
    return Synchronize(GetCollisionData(body), body, group);
}

void FCLCollisionChecker::Synchronize(FCLUserDataPtr const &collision_data,
                                      KinBodyConstPtr const &body,
                                      CollisionGroup *group)
{
    for (LinkPtr const &link : body->GetLinks()) {
        if (!link->IsEnabled()) {
            continue;
        }

        Synchronize(collision_data, link, group);
    }
}

void FCLCollisionChecker::Synchronize(LinkConstPtr const &link,
                                      CollisionGroup *group)
{
    return Synchronize(GetCollisionData(link->GetParent()), link, group);
}

void FCLCollisionChecker::Synchronize(FCLUserDataPtr const &collision_data,
                                      LinkConstPtr const &link,
                                      CollisionGroup *group)
{
    OpenRAVE::Transform const link_pose = link->GetTransform();

    for (GeometryPtr const &geom : link->GetGeometries()) {
        auto const result = collision_data->geometries.insert(
            make_pair(geom.get(), CollisionObjectPtr())
        );
        CollisionObjectPtr &fcl_object = result.first->second;

        // Convert the OpenRAVE geometry into FCL geometry. This is
        // necessary if: (1) there is no existing FCL geometry for this
        // shape or (2) the geometric is dynamic and could be modified at
        // any time.
        if (result.second || geom->IsModifiable()) {
            CollisionGeometryPtr const fcl_geom = ConvertGeometryToFCL(geom);
            fcl_object = make_shared<fcl::CollisionObject>(fcl_geom);
            if (fcl_object) {
                fcl_object->setUserData(const_cast<Link *>(link.get()));
            }
        }

        // Update the pose of the FCL geometry to match the environment. We
        // need to check for NULL here because some OpenRAVE geometry may
        // not map to no FCL geometry (e.g. GT_None). We retain these NULLs
        // to simplify bookkeeping when geometry is modified.
        if (fcl_object) {
            OpenRAVE::Transform const &pose = link_pose * geom->GetTransform();
            fcl_object->setTranslation(ConvertVectorToFCL(pose.trans));
            fcl_object->setQuatRotation(ConvertQuaternionToFCL(pose.rot));

            if (group) {
                group->push_back(fcl_object.get());
            }
        }
    }

    // One or more geometries were dynamically removed from the OpenRAVE
    // environment. Delete the associated FCL geometries.
    // TODO: This logic only works if we maintain a separate list of geometries
    // for each link. Otherwise we can only GC at the KinBody level.
#if 0
    size_t const num_or_geometries = link->GetGeometries().size();
    size_t const num_fcl_geometries = collision_data->geometries.size();
    BOOST_ASSERT(num_fcl_geometries >= num_or_geometries);

    if (num_fcl_geometries > num_or_geometries) {
        // TODO: Cleanup geometries.
    }
#endif
}

bool FCLCollisionChecker::NarrowPhaseCheckCollision(
        fcl::CollisionObject *o1, fcl::CollisionObject *o2, void *data)
{
    auto const query = static_cast<CollisionQuery *>(data);

    size_t const num_contacts = fcl::collide(o1, o2, query->request,
                                                     query->result);

    if (num_contacts > 0) {
        // If an output report was specified, fill it in.
        if (query->report) {
            query->report->plink1 = GetCollisionLink(*o1);
            query->report->plink2 = GetCollisionLink(*o2);
            query->report->vLinkColliding.push_back(query->report->plink2);
            // TODO: Update numCols.
        }

        // Call any collision callbacks. Ignore this collision if any of the
        // callbacks return CA_Ignore.
        CollisionAction action = OpenRAVE::CA_DefaultAction;
        for (CollisionCallbackFn const &callback : query->callbacks) {
            action = callback(query->report, false);
            if (action == OpenRAVE::CA_Ignore) { return false; // Keep going.
            }
        }

        // Store contact information if CO_Contacts is enabled.
        if (query->report && query->request.enable_contact) {
            query->report->contacts.resize(num_contacts);
            for (size_t icontact = 0; icontact < num_contacts; ++icontact) {
                fcl::Contact const &contact =  query->result.getContact(icontact);
                query->report->contacts[icontact] = ConvertContactToOR(contact);
            }
        }

        // Otherwise, note that a collision occurred and short-circuit.
        query->is_collision = true;
        return true; // Stop checking.
    }
    return false; // Keep going.
}

Vector FCLCollisionChecker::ConvertVectorToOR(fcl::Vec3f const &v)
{
    return Vector(v[0], v[1], v[2]);
}

CollisionReport::CONTACT FCLCollisionChecker::ConvertContactToOR(
        fcl::Contact const &contact)
{
    CollisionReport::CONTACT or_contact;
    or_contact.pos = ConvertVectorToOR(contact.pos);
    or_contact.norm = ConvertVectorToOR(contact.normal);
    or_contact.depth = contact.penetration_depth;
    return or_contact;
}

fcl::Vec3f FCLCollisionChecker::ConvertVectorToFCL(Vector const &v)
{
    return fcl::Vec3f(v.x, v.y, v.z);
}

fcl::Quaternion3f FCLCollisionChecker::ConvertQuaternionToFCL(Vector const &v)
{
    // OpenRAVE and FCL both use the (scalar, vector) convention. Remember to
    // never use the named attributes on OpenRAVE::Vector to access quaternion
    // elements: they're wrong!
    return fcl::Quaternion3f(v[0], v[1], v[2], v[3]);
}

auto FCLCollisionChecker::ConvertGeometryToFCL(
        GeometryConstPtr const &geom) -> CollisionGeometryPtr
{
    switch (geom->GetType()) {
    case OpenRAVE::GT_None:
        return CollisionGeometryPtr();

    case OpenRAVE::GT_Box: {
        OpenRAVE::Vector const extents = geom->GetBoxExtents();

        if (extents[0] != 0 || extents[1] != 0 || extents[2] != 0) {
            // OpenRAVE's extents are actually half-extents.
            return make_shared<fcl::Box>(
                2 * extents[0], 2 * extents[1], 2 * extents[2]
            );
        } else {
            return CollisionGeometryPtr();
        }
    }

    case OpenRAVE::GT_Cylinder:
        // TODO: I'm not sure what the coordinate frame convention is for
        // cylinders in FCL.
        if (geom->GetCylinderRadius() != 0 || geom->GetCylinderHeight() != 0) {
            return make_shared<fcl::Cylinder>(
                geom->GetCylinderRadius(), geom->GetCylinderHeight()
            );
        } else {
            return CollisionGeometryPtr();
        }

    case OpenRAVE::GT_Sphere:
        if (geom->GetSphereRadius() != 0) {
            return make_shared<fcl::Sphere>(geom->GetSphereRadius());
        } else {
            return CollisionGeometryPtr();
        }

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
