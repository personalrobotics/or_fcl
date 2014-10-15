#include <boost/range/adaptor/map.hpp>
#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <boost/make_shared.hpp>
#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>
#include <fcl/collision.h>
#include <fcl/shape/geometric_shapes.h>
#include "FCLCollisionChecker.h"

// Broad-phase collision checkers.
#include <fcl/broadphase/broadphase_SaP.h>
#include <fcl/broadphase/broadphase_SSaP.h>
#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#include <fcl/broadphase/broadphase_dynamic_AABB_tree_array.h>
#include <fcl/broadphase/broadphase_bruteforce.h>
#include <fcl/broadphase/broadphase_interval_tree.h>
#include <fcl/broadphase/broadphase_spatialhash.h>

using std::make_pair;
using boost::adaptors::map_values;
using boost::dynamic_pointer_cast;
using boost::format;
using boost::make_shared;
using boost::unordered_map;
using boost::unordered_set;
using boost::str;
using OpenRAVE::CollisionAction;
using OpenRAVE::CollisionReport;
using OpenRAVE::CollisionReportPtr;
using OpenRAVE::EnvironmentBasePtr;
using OpenRAVE::KinBody;
using OpenRAVE::KinBodyPtr;
using OpenRAVE::TriMesh;
using OpenRAVE::RobotBase;
using OpenRAVE::RobotBaseConstPtr;
using OpenRAVE::UserData;
using OpenRAVE::UserDataPtr;
using OpenRAVE::Vector;

typedef boost::shared_ptr<fcl::CollisionObject> CollisionObjectPtr;
typedef OpenRAVE::KinBody::Link Link;
typedef OpenRAVE::KinBody::LinkPtr LinkPtr;
typedef OpenRAVE::KinBody::Link::Geometry Geometry;
typedef OpenRAVE::KinBody::Link::GeometryPtr GeometryPtr;
typedef OpenRAVE::KinBody::JointPtr JointPtr;
typedef OpenRAVE::RobotBase::GrabbedInfoPtr GrabbedInfoPtr;
typedef OpenRAVE::EnvironmentBase::CollisionCallbackFn CollisionCallbackFn;

namespace {

/*
 * CollisionQuery
 */
struct CollisionQuery {
    CollisionQuery()
        : is_collision(false)
        , num_narrow(0)
    {
    }

    EnvironmentBasePtr env;
    fcl::CollisionRequest request;
    fcl::CollisionResult result;
    CollisionReportPtr report;
    std::list<CollisionCallbackFn> callbacks;
    unordered_set<std::pair<Link const *, Link const *> > disabled_pairs;

    bool is_collision;
    int num_narrow;

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
{
    SetBroadphaseAlgorithm("SSaP");
    SetBVHRepresentation("OBB");
}

FCLCollisionChecker::~FCLCollisionChecker()
{
    DestroyEnvironment();
}

bool FCLCollisionChecker::SetCollisionOptions(int collision_options)
{
    options_ = collision_options;

    bool is_supported = true;
    if (options_ & OpenRAVE::CO_UseTolerance) {
        RAVELOG_WARN("or_fcl does not support CO_UseTolerance\n");
        is_supported = false;
    }
    if (options_ & OpenRAVE::CO_RayAnyHit) {
        RAVELOG_WARN("or_fcl does not support CO_RayAnyHit\n");
        is_supported = false;
    }
    return is_supported;
}

int FCLCollisionChecker::GetCollisionOptions() const
{
    return options_;
}

void FCLCollisionChecker::SetBroadphaseAlgorithm(
        std::string const &algorithm)
{
    if (algorithm == "Naive") {
        manager1_ = make_shared<fcl::NaiveCollisionManager>();
        manager2_ = make_shared<fcl::NaiveCollisionManager>();
    } else if (algorithm == "SaP") {
        manager1_ = make_shared<fcl::SaPCollisionManager>();
        manager2_ = make_shared<fcl::SaPCollisionManager>();
    } else if (algorithm == "SSaP") {
        manager1_ = make_shared<fcl::SSaPCollisionManager>();
        manager2_ = make_shared<fcl::SSaPCollisionManager>();
    } else if (algorithm == "IntervalTree") {
        manager1_ = make_shared<fcl::IntervalTreeCollisionManager>();
        manager2_ = make_shared<fcl::IntervalTreeCollisionManager>();
    } else if (algorithm == "DynamicAABBTree")  {
        manager1_ = make_shared<fcl::DynamicAABBTreeCollisionManager>();
        manager2_ = make_shared<fcl::DynamicAABBTreeCollisionManager>();
    } else if (algorithm == "DynamicAABBTree_Array") {
        manager1_ = make_shared<fcl::DynamicAABBTreeCollisionManager_Array>();
        manager2_ = make_shared<fcl::DynamicAABBTreeCollisionManager_Array>();
    } else {
        throw OpenRAVE::openrave_exception(
            str(format("Unknown broad-phase algorithm '%s'.") % algorithm),
            OpenRAVE::ORE_InvalidArguments
        );
    }
}

void FCLCollisionChecker::SetBVHRepresentation(std::string const &type)
{
    if (type == "AABB") {
        mesh_factory_ = &FCLCollisionChecker::ConvertMeshToFCL<fcl::AABB>;
    } else if (type == "OBB") {
        mesh_factory_ = &FCLCollisionChecker::ConvertMeshToFCL<fcl::OBB>;
    } else if (type == "RSS") {
        mesh_factory_ = &FCLCollisionChecker::ConvertMeshToFCL<fcl::RSS>;
    } else if (type == "OBBRSS") {
        mesh_factory_ = &FCLCollisionChecker::ConvertMeshToFCL<fcl::OBBRSS>;
    } else if (type == "kIDS") {
        mesh_factory_ = &FCLCollisionChecker::ConvertMeshToFCL<fcl::AABB>;
    } else {
        throw OpenRAVE::openrave_exception(
            str(format("Unknown BVH representation '%s'.") % type),
            OpenRAVE::ORE_InvalidArguments
        );
    }
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

    // Group 1: body1 + attached, active only
    manager1_->clear();
    Synchronize(body1.get(), true, true, &group1);
    manager1_->registerObjects(group1);
    manager1_->setup();

    // Group 2: everything else
    manager2_->clear();

    std::vector<KinBodyPtr> bodies;
    GetEnv()->GetBodies(bodies);

    for (KinBodyPtr const &body2 : bodies) {
        if (body2 != body1 && body2->IsEnabled()) {
            Synchronize(body2.get(), false, false, &group2);
        }
    }

    manager2_->registerObjects(group2);
    manager2_->setup();

    return RunCheck(report);
} 

bool FCLCollisionChecker::CheckCollision(
        KinBodyConstPtr pbody1, KinBodyConstPtr pbody2,
        CollisionReportPtr report)
{
    CollisionGroup group1, group2;

    // TODO: Implement CO_ActiveDOFs
    // TODO: Attached bodies.

    // Group 1: body1 + attached, active only
    manager1_->clear();
    Synchronize(pbody1.get(), true, true, &group1);
    manager1_->registerObjects(group1);
    manager1_->setup();

    // Group 2: body2 + attached
    manager2_->clear();
    Synchronize(pbody2.get(), true, false, &group2);
    manager2_->registerObjects(group2);
    manager2_->setup();

    return RunCheck(report);
}

bool FCLCollisionChecker::CheckCollision(
        LinkConstPtr plink, CollisionReportPtr report)
{
    CollisionGroup group1, group2;

    // Group 1: link
    manager1_->clear();
    Synchronize(plink.get(), &group1);
    manager1_->registerObjects(group1);
    manager1_->setup();

    // Group 2: bodies - link's parent
    manager2_->clear();

    std::vector<KinBodyPtr> bodies;
    GetEnv()->GetBodies(bodies);

    // TODO: Should we check against plink->GetParent()?
    for (KinBodyPtr const &body2 : bodies) {
        if (body2 == plink->GetParent() || !body2->IsEnabled()) {
            continue;
        }
        Synchronize(body2.get(), false, false, &group2);
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
    Synchronize(link1.get(), &group1);
    manager1_->registerObjects(group1);
    manager1_->setup();

    // Group 2: link2.
    manager2_->clear();
    Synchronize(link2.get(), &group2);
    manager2_->registerObjects(group2);
    manager2_->setup();

    return RunCheck(report);
}

bool FCLCollisionChecker::CheckCollision(
    LinkConstPtr plink, KinBodyConstPtr pbody, CollisionReportPtr report)
{
    CollisionGroup group1, group2;

    // TODO: handle attached bodies.

    // Group 1: link.
    manager1_->clear();
    Synchronize(plink.get(), &group1);
    manager1_->registerObjects(group1);
    manager1_->setup();

    // Group 2: link2.
    manager2_->clear();
    Synchronize(pbody.get(), true, false, &group2);
    manager2_->registerObjects(group2);
    manager2_->setup();

    return RunCheck(report);
}

bool FCLCollisionChecker::CheckStandaloneSelfCollision(
        KinBodyConstPtr pbody, CollisionReportPtr report)
{
    unordered_set<std::pair<Link const *, Link const *> > disabled_pairs;
    CollisionGroup group1, group2;
    
    // Generate the minimal set of possible link-link collisions. This
    // implicitly handles the CO_ActiveDOFs option by delegating that
    // responsibility to GetNonAdjacentLinks. We intentionally do not pass
    // AO_Enabled so the output can be used to determine which grabbed
    // objects should be considered.
    int ao = 0;
    if (options_ & OpenRAVE::CO_ActiveDOFs) {
        ao |= OpenRAVE::KinBody::AO_ActiveDOFs;
    }

    std::set<int> const &nonadjacent_pairs_raw = pbody->GetNonAdjacentLinks(ao);
    std::vector<std::pair<Link const *, Link const *> > nonadjacent_pairs;
    UnpackLinkPairs(pbody, nonadjacent_pairs_raw, &nonadjacent_pairs);

    unordered_set<Link const *> group1_links, group2_links;
    for (std::pair<Link const *, Link const *> const &link_pair : nonadjacent_pairs) {
        Link const *link1 = link_pair.first;
        if (link1->IsEnabled() && !group1_links.count(link1)) {
            Synchronize(link1, &group1);
            group1_links.insert(link1);
        }

        Link const *link2 = link_pair.second;
        if (link2->IsEnabled() && !group2_links.count(link2)) {
            Synchronize(link2, &group2);
            group2_links.insert(link2);
        }
    }

    // Disable collisions between adjacent links.
    std::set<int> const &adjacent_pairs_raw = pbody->GetAdjacentLinks();
    std::vector<std::pair<Link const *, Link const *> > adjacent_pairs;
    UnpackLinkPairs(pbody, adjacent_pairs_raw, &adjacent_pairs);
    disabled_pairs.insert(adjacent_pairs.begin(),
                          adjacent_pairs.end());

    // Include grabbed objects that are attached to one or more of
    // the links that we're considered.
    if (pbody->IsRobot()) {
        auto const robot = dynamic_pointer_cast<RobotBase const>(pbody);
        std::vector<GrabbedInfoPtr> grabbed_infos;
        robot->GetGrabbedInfo(grabbed_infos);

        for (GrabbedInfoPtr const &grabbed_info : grabbed_infos) {
            std::string const &link_name = grabbed_info->_robotlinkname;
            std::string const &grabbed_name = grabbed_info->_grabbedname;
            KinBodyPtr const grabbed_body = GetEnv()->GetKinBody(grabbed_name);
            LinkPtr const link = robot->GetLink(link_name);
            int const link_index = link->GetIndex();

            // Body is grabbed by an active link.
            if (group1_links.count(link.get()) || group2_links.count(link.get())) {
                if (grabbed_body != robot && grabbed_body->IsEnabled()) {
                    CollisionGroup temp_group;
                    Synchronize(grabbed_body.get(), true, false, &temp_group);
                    group1.insert(group1.end(), temp_group.begin(), temp_group.end());
                    group2.insert(group2.end(), temp_group.begin(), temp_group.end());
                }
            }

            // Disable collisions between the grabbed body and select robot links.
            std::list<LinkConstPtr> ignored_robot_links;
            robot->GetIgnoredLinksOfGrabbed(grabbed_body, ignored_robot_links);
            std::vector<LinkPtr> const &grabbed_links = grabbed_body->GetLinks();

            for (LinkConstPtr const &grabbed_link : grabbed_links) {
                for (LinkConstPtr const &robot_link : ignored_robot_links) {
                    disabled_pairs.insert(
                        MakeLinkPair(grabbed_link.get(), robot_link.get())
                    );
                }
            }
        }
    }

    manager1_->clear();
    manager1_->registerObjects(group1);
    manager1_->setup();

    manager2_->clear();
    manager2_->registerObjects(group2);
    manager2_->setup();

    return RunCheck(report, disabled_pairs);
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
        KinBody const *body) const
{
    UserDataPtr const user_data = body->GetUserData(user_data_);
    return dynamic_pointer_cast<FCLUserData>(user_data);
}

bool FCLCollisionChecker::RunCheck(
        CollisionReportPtr report,
        boost::unordered_set<LinkPair> const &disabled_pairs)
{
    CollisionQuery query;
    query.env = GetEnv();
    query.report = report;
    query.env->GetRegisteredCollisionCallbacks(query.callbacks);

    // TODO: We don't need to copy here.
    query.disabled_pairs = disabled_pairs;

    if (options_ & OpenRAVE::CO_Distance) {
        throw OpenRAVE::openrave_exception(
            "or_fcl does not currently support CO_Distance.",
            OpenRAVE::ORE_NotImplemented
        );
    }

    if (options_ & OpenRAVE::CO_Contacts) {
        query.request.enable_contact = true;
        query.request.num_max_contacts = num_contacts_;
    }

    // Initialize the report.
    if (report) {
        report->options = options_;
        report->plink1.reset(); 
        report->plink2.reset(); 
        report->numCols = 0;
        report->vLinkColliding.clear();
        report->minDistance = std::numeric_limits<OpenRAVE::dReal>::max();
        report->numWithinTol = 0;
        report->contacts.clear();
    }

    manager1_->collide(manager2_.get(), &query,
                       &FCLCollisionChecker::NarrowPhaseCheckCollision);
    return query.result.isCollision();
    
}

void FCLCollisionChecker::UnpackLinkPairs(
        std::set<int> const &packed,
        std::vector<std::pair<int, int> > *unpacked) const
{
    BOOST_ASSERT(unpacked);

    unpacked->reserve(unpacked->size() + packed.size());

    for (int const &link_pair : packed) {
        int const index1 = ((link_pair >>  0) & 0xFFFF);
        int const index2 = ((link_pair >> 16) & 0xFFFF);
        if (index1 < index2) {
            unpacked->push_back(make_pair(index1, index2));
        } else {
            unpacked->push_back(make_pair(index2, index1));
        }
    }
}

void FCLCollisionChecker::UnpackLinkPairs(
        KinBodyConstPtr const &body, std::set<int> const &packed,
        std::vector<std::pair<Link const *, Link const *> > *unpacked) const
{
    BOOST_ASSERT(body);
    BOOST_ASSERT(unpacked);

    std::vector<std::pair<int, int> > index_pairs;
    UnpackLinkPairs(packed, &index_pairs);

    std::vector<LinkPtr> const &links = body->GetLinks();
    unpacked->reserve(unpacked->size() + packed.size());

    for (std::pair<int, int> const &index_pair : index_pairs) {
        LinkPtr const &link1 = links[index_pair.first];
        LinkPtr const &link2 = links[index_pair.second];
        unpacked->push_back(MakeLinkPair(link1.get(), link2.get()));
    }
}

std::pair<Link const *, Link const *> FCLCollisionChecker::MakeLinkPair(
        Link const *link1, Link const *link2)
{
    if (link1 < link2) {
        return std::make_pair(link1, link2);
    } else {
        return std::make_pair(link2, link1);
    }
}

void FCLCollisionChecker::Synchronize(KinBody const *body,
                                      bool attached, bool active_only,
                                      CollisionGroup *group)
{
    return Synchronize(GetCollisionData(body), body,
                       attached, active_only, group);
}

void FCLCollisionChecker::Synchronize(FCLUserDataPtr const &collision_data,
                                      KinBody const *body,
                                      bool attached, bool active_only,
                                      CollisionGroup *group)
{
    // If this is a robot and we're honoring CO_ActiveDOFs, then only
    // synchronize the links that are affected by one or more active DOFs.
    std::vector<LinkConstPtr> body_links;
    bool const co_activedofs = options_ & OpenRAVE::CO_ActiveDOFs;

    if (body->IsRobot() && co_activedofs && active_only) {
        auto const robot = dynamic_cast<RobotBase const *>(body);

        // Pre-compute the bodies attached to each link.
        boost::unordered_map<int, std::vector<KinBodyPtr> > grabbed_map;
        std::vector<GrabbedInfoPtr> grabbed_infos;
        robot->GetGrabbedInfo(grabbed_infos);

        for (GrabbedInfoPtr const &grabbed_info : grabbed_infos) {
            std::string const link_name = grabbed_info->_robotlinkname;
            LinkPtr const link = robot->GetLink(link_name);

            std::string const grabbed_name = grabbed_info->_grabbedname;
            KinBodyPtr const grabbed_body = GetEnv()->GetKinBody(grabbed_name);

            if (grabbed_body.get() != robot) {
                grabbed_map[link->GetIndex()].push_back(grabbed_body);
            }
        }

        // Joints may cover more than one DOF (e.g. spherical joints). First,
        // compute the set of joints that covers the active DOFs.
        std::vector<int> const &dof_indices = robot->GetActiveDOFIndices();
        boost::unordered_set<int> active_joint_indices; 

        for (int const &dof_index : dof_indices) {
            JointPtr const joint = robot->GetJointFromDOFIndex(dof_index);
            active_joint_indices.insert(joint->GetJointIndex());
        }

        for (LinkPtr const &link : robot->GetLinks()) {
            int const link_index = link->GetIndex();

            // Check if the link is affected by the active DOFs.
            bool is_affected = false;
            for (int const &joint_index : active_joint_indices) {
                if (robot->DoesAffect(joint_index, link_index)) {
                    is_affected = true;
                    break;
                }
            }

            if (is_affected) {
                // Synchronize the link itself.
                if (link->IsEnabled()) {
                    Synchronize(collision_data, link.get(), group);
                }

                // Synchronize bodies attached to this link.
                if (attached) {
                    for (KinBodyPtr const &grabbed_body
                            : grabbed_map[link_index]) {
                        Synchronize(collision_data, grabbed_body.get(),
                                    true, false, group);
                    }
                }
            }
        }
    }
    // Otherwise, synchronize all enabled links and attached bodies.
    else {
        for (LinkPtr const &link : body->GetLinks()) {
            if (link->IsEnabled()) {
                Synchronize(collision_data, link.get(), group);
            }
        }

        if (attached) {
            std::set<KinBodyPtr> attached_bodies;
            body->GetAttached(attached_bodies);

            for (KinBodyPtr const &attached_body : attached_bodies) {
                if (attached_body.get() != body && attached_body->IsEnabled()) {
                    Synchronize(collision_data, attached_body.get(),
                                true, false, group);
                }
            }
        }
    }
}

void FCLCollisionChecker::Synchronize(Link const *link,
                                      CollisionGroup *group)
{
    return Synchronize(GetCollisionData(link->GetParent().get()), link, group);
}

void FCLCollisionChecker::Synchronize(FCLUserDataPtr const &collision_data,
                                      Link const *link,
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
        if (result.second) { // || geom->IsModifiable()) {
            CollisionGeometryPtr const fcl_geom = ConvertGeometryToFCL(
                mesh_factory_, geom
            );
            if (fcl_geom) {
                fcl_object = make_shared<fcl::CollisionObject>(fcl_geom);
                fcl_object->setUserData(const_cast<Link *>(link));
            }
        }

        // Update the pose of the FCL geometry to match the environment. We
        // need to check for NULL here because some OpenRAVE geometry may
        // not map to no FCL geometry (e.g. GT_None). We retain these NULLs
        // to simplify bookkeeping when geometry is modified.
        if (fcl_object) {
            // TODO: Only update the object's pose if it's changed.
            OpenRAVE::Transform const &pose = link_pose * geom->GetTransform();
            fcl::Vec3f const new_position = ConvertVectorToFCL(pose.trans);
            fcl::Quaternion3f const new_orientation = ConvertQuaternionToFCL(pose.rot);

            // Only recompute the AABB if the pose changed.
            if (!AreEqual(new_position, fcl_object->getTranslation())
             || !AreEqual(new_orientation, fcl_object->getQuatRotation())) {
                fcl_object->setTranslation(new_position);
                fcl_object->setQuatRotation(new_orientation);
                fcl_object->computeAABB();
            }

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
    LinkConstPtr const plink1 = GetCollisionLink(*o1);
    LinkConstPtr const plink2 = GetCollisionLink(*o2);

    // Ignore collision checks with the same object. This might happen if we
    // call a CheckCollision with two parameters that overlap.
    if (plink1 == plink2) {
        return false; // Keep going.
    }
    // Ignore disabled pairs of links. MakeLinkPair enforces the invariant that
    // plink1 <= plink2, so we don't need to check (plink2, plink1).
    else if (query->disabled_pairs.count(MakeLinkPair(plink1.get(), plink2.get()))) {
        return false; // Keep going.
    }

    size_t const num_contacts = fcl::collide(o1, o2, query->request,
                                                     query->result);
    query->num_narrow++;

    if (num_contacts > 0) {
        // If an output report was specified, fill it in.
        if (query->report) {
            query->report->plink1 = GetCollisionLink(*o1);
            query->report->plink2 = GetCollisionLink(*o2);
            query->report->vLinkColliding.push_back(query->report->plink2);
            // TODO: Update numCols.

            if (query->request.enable_contact) {
                query->report->contacts.resize(num_contacts);
                for (size_t icontact = 0; icontact < num_contacts; ++icontact) {
                    fcl::Contact const &contact =  query->result.getContact(icontact);
                    query->report->contacts[icontact] = ConvertContactToOR(contact);
                }
            }
        }

        // Call any collision callbacks. Ignore this collision if any of the
        // callbacks return CA_Ignore.
        CollisionAction action = OpenRAVE::CA_DefaultAction;
        for (CollisionCallbackFn const &callback : query->callbacks) {
            action = callback(query->report, false);
            if (action == OpenRAVE::CA_Ignore) {
                return false; // Keep going.
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
        MeshFactory const &mesh_factory,
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
                tri_indices[0], tri_indices[1], tri_indices[2]
            );
        }

        return mesh_factory(fcl_points, fcl_triangles);
    }

    default:
        throw OpenRAVE::openrave_exception(
            str(format("Unknown geometry type %d.") % geom->GetType()),
            OpenRAVE::ORE_InvalidArguments
        );
    }
}

template <class T>
auto FCLCollisionChecker::ConvertMeshToFCL(
    std::vector<fcl::Vec3f> const &points,
    std::vector<fcl::Triangle> const &triangles) -> CollisionGeometryPtr
{
    auto const model = make_shared<fcl::BVHModel<T> >();
    model->beginModel(triangles.size(), points.size());
    model->addSubModel(points, triangles);
    model->endModel();
    return model;
}

bool FCLCollisionChecker::AreEqual(fcl::Vec3f const &x,
                                   fcl::Vec3f const &y) const
{
    return x[0] == y[0] && x[1] == y[1] && x[2] == y[2];
}

bool FCLCollisionChecker::AreEqual(fcl::Quaternion3f const &x,
                                   fcl::Quaternion3f const &y) const
{
    // We only need to check three components since a quaternion only has three
    // degrees of freedom (it has unit length.
    return x.getX() == y.getX() && x.getY() == y.getY() && x.getZ() == y.getZ();
}

}
