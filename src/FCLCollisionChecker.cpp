/***********************************************************************

Copyright (c) 2016, Carnegie Mellon University
All rights reserved.

Author: Michael Koval <mkoval@cs.cmu.edu>

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:

  Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.

  Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*************************************************************************/
#include <boost/range/adaptor/map.hpp>
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
    unordered_set<std::pair<Link const *, Link const *> > self_enabled_pairs;

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

/* checks collision of a body and a scene.
 * Attached bodies are respected.
 * If CO_ActiveDOFs is set, will only check affected links of the body.
 */
bool FCLCollisionChecker::CheckCollision(
        KinBodyConstPtr body, CollisionReportPtr report)
{
    static std::vector<KinBodyConstPtr> const vbodyexcluded_empty;
    static std::vector<LinkConstPtr> const vlinkexcluded_empty;

    return CheckCollision(body, vbodyexcluded_empty, vlinkexcluded_empty, report);
} 

/* checks collision between two bodies.
 * Attached bodies are respected.
 * If CO_ActiveDOFs is set, will only check affected links of the pbody1.
 */
bool FCLCollisionChecker::CheckCollision(
        KinBodyConstPtr pbody1, KinBodyConstPtr pbody2,
        CollisionReportPtr report)
{
    CollisionGroup group1, group2;

    // Check to see if either of the bodies is grabbing the other
    if( pbody1->IsAttached(pbody2) ){
        return false;
    }
    
    // is pbody1 a robot whose activedofs we should respect?
    OpenRAVE::RobotBaseConstPtr robot1;
    if (pbody1->IsRobot() && (options_ & OpenRAVE::CO_ActiveDOFs)) {
        robot1 = dynamic_pointer_cast<RobotBase const>(pbody1);
    }
    
    // get kinbodies attached to each kinbody
    std::set<KinBodyPtr> attached_kinbodies1;
    std::set<KinBodyPtr> attached_kinbodies2;
    pbody1->GetAttached(attached_kinbodies1);
    pbody2->GetAttached(attached_kinbodies2);
    
    static unordered_set<KinBodyConstPtr> const empty_body_set;
    static unordered_set<LinkConstPtr> const empty_link_set;
    
    // Group 1: body1 + attached, only check active dofs
    manager1_->clear();
    SynchronizeKinbodies(attached_kinbodies1, empty_body_set, empty_link_set, &group1, robot1);
    manager1_->registerObjects(group1);
    manager1_->setup();

    // Group 2: body2 + attached, check all links
    manager2_->clear();
    SynchronizeKinbodies(attached_kinbodies2, empty_body_set, empty_link_set, &group2);
    manager2_->registerObjects(group2);
    manager2_->setup();

    return RunCheck(report);
}

/* checks collision of a link and a scene.
 * Attached bodies are ignored.
 * CO_ActiveDOFs option is ignored.
 */
bool FCLCollisionChecker::CheckCollision(
        LinkConstPtr plink, CollisionReportPtr report)
{
    static std::vector<KinBodyConstPtr> const vbodyexcluded_empty;
    static std::vector<LinkConstPtr> const vlinkexcluded_empty;

    return CheckCollision(plink, vbodyexcluded_empty, vlinkexcluded_empty, report);
}

/* checks collision of two links.
 * Attached bodies are ignored.
 * CO_ActiveDOFs option is ignored.
 */
bool FCLCollisionChecker::CheckCollision(
        LinkConstPtr link1, LinkConstPtr link2, CollisionReportPtr report)
{
    CollisionGroup group1, group2;

    if (!link1->IsEnabled() || !link2->IsEnabled())
        return false;

    boost::unordered_set<LinkPair> disabled_pairs;
    boost::unordered_set<LinkPair> self_enabled_pairs;
    if (link1->GetParent() == link2->GetParent()) {
        self_enabled_pairs.insert(MakeLinkPair(link1.get(), link2.get()));
    }

    // Group 1: link1.
    manager1_->clear();
    SynchronizeLink(GetCollisionData(link1->GetParent()), link1.get(), &group1);
    manager1_->registerObjects(group1);
    manager1_->setup();

    // Group 2: link2.
    manager2_->clear();
    SynchronizeLink(GetCollisionData(link2->GetParent()), link2.get(), &group2);
    manager2_->registerObjects(group2);
    manager2_->setup();

    return RunCheck(report, disabled_pairs, self_enabled_pairs);
}

/* checks collision of a link and a body.
 * Attached bodies for pbody are respected.
 * CO_ActiveDOFs option is ignored.
 */
bool FCLCollisionChecker::CheckCollision(
    LinkConstPtr plink, KinBodyConstPtr pbody, CollisionReportPtr report)
{
    CollisionGroup group1, group2;
    
    // Check to see if either of the bodies is grabbing the other
    // (this would be a self-collision instead!)
    if (!plink->IsEnabled() || plink->GetParent()->IsAttached(pbody)) {
        return false;
    }
    
    static unordered_set<KinBodyConstPtr> const empty_body_set;
    static unordered_set<LinkConstPtr> const empty_link_set;

    // Group 1: link.
    manager1_->clear();
    SynchronizeLink(GetCollisionData(plink->GetParent()), plink.get(), &group1);
    manager1_->registerObjects(group1);
    manager1_->setup();

    // Group 2: body + attached, check all links
    std::set<KinBodyPtr> attached_kinbodies;
    pbody->GetAttached(attached_kinbodies);
    manager2_->clear();
    SynchronizeKinbodies(attached_kinbodies, empty_body_set, empty_link_set, &group2);
    manager2_->registerObjects(group2);
    manager2_->setup();

    return RunCheck(report);
}

/* checks collision of a link and a scene.
 * Attached bodies are ignored.
 * CO_ActiveDOFs option is ignored.
 */
bool FCLCollisionChecker::CheckCollision(
    LinkConstPtr plink,
    std::vector<KinBodyConstPtr> const &vbodyexcluded,
    std::vector<LinkConstPtr> const &vlinkexcluded,
    CollisionReportPtr report)
{
    CollisionGroup group1, group2;

    unordered_set<KinBodyConstPtr> excluded_body_set(
        vbodyexcluded.begin(), vbodyexcluded.end());
    unordered_set<LinkConstPtr> excluded_link_set(
        vlinkexcluded.begin(), vlinkexcluded.end());
    
    if (!plink->IsEnabled() || excluded_link_set.count(plink))
        return false;
    
    // Group 1: link.
    KinBodyPtr pbody = plink->GetParent();
    manager1_->clear();
    SynchronizeLink(GetCollisionData(pbody), plink.get(), &group1);
    manager1_->registerObjects(group1);
    manager1_->setup();
    
    // exclude kinbodies attached to the link's body (would be self)
    std::set<KinBodyPtr> attached_kinbodies;
    pbody->GetAttached(attached_kinbodies);
    for (auto it=attached_kinbodies.begin(); it!=attached_kinbodies.end(); it++) {
        excluded_body_set.insert(*it);
    }

    // Group 2: all enabled links that are not excluded or attached to pbody
    manager2_->clear();
    std::vector<KinBodyPtr> kinbodies;
    GetEnv()->GetBodies(kinbodies);
    for (KinBodyPtr const &kinbody : kinbodies) {
        if (kinbody->IsEnabled() && !excluded_body_set.count(kinbody)) {
            SynchronizeKinbody(kinbody, excluded_link_set, &group2);
        }
    }

    manager2_->registerObjects(group2);
    manager2_->setup();

    return RunCheck(report);
}

/* checks collision of a body and a scene.
 * Attached bodies are respected.
 * If CO_ActiveDOFs is set, will only check affected links of pbody.
 */
bool FCLCollisionChecker::CheckCollision(
    KinBodyConstPtr pbody, 
    std::vector<KinBodyConstPtr> const &vbodyexcluded,
    std::vector<LinkConstPtr> const &vlinkexcluded,
    CollisionReportPtr report)
{

    CollisionGroup group1, group2;

    // convert excluded bodies/links
    unordered_set<KinBodyConstPtr> excluded_body_set(
        vbodyexcluded.begin(), vbodyexcluded.end());
    unordered_set<LinkConstPtr> const excluded_link_set(
        vlinkexcluded.begin(), vlinkexcluded.end());
    
    // is there a robot whose activedofs we should respect?
    OpenRAVE::RobotBaseConstPtr robot;
    if (pbody->IsRobot() && (options_ & OpenRAVE::CO_ActiveDOFs)) {
        robot = dynamic_pointer_cast<RobotBase const>(pbody);
    }
    
    // get kinbodies attached to pbody
    std::set<KinBodyPtr> attached_kinbodies;
    pbody->GetAttached(attached_kinbodies);

    // Group 1: body + attached, respect active dofs.
    manager1_->clear();
    SynchronizeKinbodies(attached_kinbodies, excluded_body_set, excluded_link_set, &group1, robot);
    manager1_->registerObjects(group1);
    manager1_->setup();
    
    for (auto it=attached_kinbodies.begin(); it!=attached_kinbodies.end(); it++) {
        excluded_body_set.insert(*it);
    }

    // Group 2: all other enabled kinbodies in environment
    manager2_->clear();
    
    std::vector<KinBodyPtr> kinbodies;
    GetEnv()->GetBodies(kinbodies);
    for (KinBodyPtr const &kinbody : kinbodies) {
        if (kinbody->IsEnabled() && !excluded_body_set.count(kinbody)) {
            SynchronizeKinbody(kinbody, excluded_link_set, &group2);
        }
    }

    manager2_->registerObjects(group2);
    manager2_->setup();

    return RunCheck(report);
}

bool FCLCollisionChecker::CheckStandaloneSelfCollision(
        KinBodyConstPtr pbody, CollisionReportPtr report)
{
    unordered_set<std::pair<Link const *, Link const *> > disabled_pairs;
    unordered_set<std::pair<Link const *, Link const *> > self_enabled_pairs;
    CollisionGroup group1, group2;
    
    // Generate the minimal set of possible link-link collisions. This
    // implicitly handles the CO_ActiveDOFs option by delegating that
    // responsibility to GetNonAdjacentLinks. We intentionally do not pass
    // AO_Enabled so the output can be used to determine which grabbed
    // objects should be considered.
    int ao = 0;
    if (pbody->IsRobot() && (options_ & OpenRAVE::CO_ActiveDOFs)) {
        ao |= OpenRAVE::KinBody::AO_ActiveDOFs;
    }

    std::set<int> const &nonadjacent_pairs_raw = pbody->GetNonAdjacentLinks(ao);
    std::vector<std::pair<Link const *, Link const *> > nonadjacent_pairs;
    UnpackLinkPairs(pbody, nonadjacent_pairs_raw, &nonadjacent_pairs);
    self_enabled_pairs.insert(nonadjacent_pairs.begin(), nonadjacent_pairs.end());

    unordered_set<Link const *> group1_links, group2_links;
    for (std::pair<Link const *, Link const *> const &link_pair : nonadjacent_pairs) {
        Link const *link1 = link_pair.first;
        if (link1->IsEnabled() && !group1_links.count(link1)) {
            SynchronizeLink(GetCollisionData(link1->GetParent()), link1, &group1);
            group1_links.insert(link1);
        }

        Link const *link2 = link_pair.second;
        if (link2->IsEnabled() && !group2_links.count(link2)) {
            SynchronizeLink(GetCollisionData(link1->GetParent()), link2, &group2);
            group2_links.insert(link2);
        }
    }

    // Disable collisions between adjacent links.
    std::set<int> const &adjacent_pairs_raw = pbody->GetAdjacentLinks();
    std::vector<std::pair<Link const *, Link const *> > adjacent_pairs;
    UnpackLinkPairs(pbody, adjacent_pairs_raw, &adjacent_pairs);
    disabled_pairs.insert(adjacent_pairs.begin(),
                          adjacent_pairs.end());

    manager1_->clear();
    manager1_->registerObjects(group1);
    manager1_->setup();

    manager2_->clear();
    manager2_->registerObjects(group2);
    manager2_->setup();

    return RunCheck(report, disabled_pairs, self_enabled_pairs);
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

bool FCLCollisionChecker::RunCheck(
        CollisionReportPtr report,
        boost::unordered_set<LinkPair> const &disabled_pairs,
        boost::unordered_set<LinkPair> const &self_enabled_pairs)
{
    CollisionQuery query;
    query.env = GetEnv();
    query.report = report;
    query.env->GetRegisteredCollisionCallbacks(query.callbacks);

    // We must have a CollisionReport if callbacks are registered, since they
    // get passed the CollisionReport as an argument.
    if (!query.callbacks.empty() && !query.report) {
        query.report = make_shared<CollisionReport>();
    }

    // TODO: We don't need to copy here.
    query.disabled_pairs = disabled_pairs;
    query.self_enabled_pairs = self_enabled_pairs;

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
    if (report)
        report->Reset(options_);

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

void FCLCollisionChecker::SynchronizeKinbodies(
        std::set<KinBodyPtr> const &attached_kinbodies,
        boost::unordered_set<KinBodyConstPtr> const &excluded_kinbody_set,
        boost::unordered_set<LinkConstPtr> const &excluded_link_set,
        CollisionGroup *group,
        OpenRAVE::RobotBaseConstPtr robot)
{    
    if (robot) {
        
        // Joints may cover more than one DOF (e.g. spherical joints). First,
        // compute the set of joints that covers the active DOFs.
        std::vector<int> const &active_dofs = robot->GetActiveDOFIndices();
        
        std::set<int> active_joint_indices;
        for (int const &active_dof : active_dofs) {
            JointPtr const active_joint = robot->GetJointFromDOFIndex(active_dof);
            active_joint_indices.insert(active_joint->GetJointIndex());
        }
        
        // compute inactive links
        boost::unordered_set<LinkConstPtr> inactive_link_set;
        for (LinkPtr link : robot->GetLinks()) {
            bool link_is_active = false;
            for (int active_joint_index : active_joint_indices) {
                if (robot->DoesAffect(active_joint_index, link->GetIndex())) {
                    link_is_active = true;
                    break;
                }
            }
            if (!link_is_active) {
                inactive_link_set.insert(link);
            }
        }
        
        // iterate over attached kinbodies
        for (KinBodyPtr const &kinbody : attached_kinbodies) {
            if (!kinbody->IsEnabled() || excluded_kinbody_set.count(kinbody)) {
                continue;
            }
            
            // if this is the robot itself,
            // include the inactive links we calculated into its exclude set
            if (kinbody == robot) {
                boost::unordered_set<LinkConstPtr> robot_excluded_link_set = excluded_link_set;
                robot_excluded_link_set.insert(inactive_link_set.begin(), inactive_link_set.end());
                SynchronizeKinbody(kinbody, robot_excluded_link_set, group);
                continue;
            }
            
            // if the kinbody is grabbed by an inactive robot link, skip it
            LinkPtr const robot_link = robot->IsGrabbing(kinbody);
            if (robot_link)
            {
                if (inactive_link_set.count(robot_link)) {
                    continue;
                }
            }
            
            // it's just a kinbody (maybe a parent robot)?
            // or maybe it's grabbed by an active robot link
            // so check everything
            SynchronizeKinbody(kinbody, excluded_link_set, group);
        }
            
    } else {
        for (KinBodyPtr const &kinbody : attached_kinbodies) {
            if (!kinbody->IsEnabled() || excluded_kinbody_set.count(kinbody)) {
                continue;
            }
            SynchronizeKinbody(kinbody, excluded_link_set, group);
        }
    }
}

void FCLCollisionChecker::SynchronizeKinbody(
        OpenRAVE::KinBodyConstPtr const &kinbody,
        boost::unordered_set<LinkConstPtr> const &excluded_link_set,
        CollisionGroup *group)
{
    // get user data pointer from kinbody
    FCLUserDataPtr const &collision_data = GetCollisionData(kinbody);
    
    // synchronize all non-excluded links
    for (LinkPtr const &link : kinbody->GetLinks()) {
        if (!link->IsEnabled() || excluded_link_set.count(link)) {
            continue;
        }
        SynchronizeLink(collision_data, link.get(), group);
    }
}

void FCLCollisionChecker::SynchronizeLink(
        FCLUserDataPtr const &collision_data,
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
    auto const link_pair = MakeLinkPair(plink1.get(), plink2.get());

    // Ignore collision checks with the same object. This might happen if we
    // call a CheckCollision with two parameters that overlap.
    if (plink1 == plink2) {
        return false; // Keep going.
    }
    // Ignore disabled pairs of links. MakeLinkPair enforces the invariant that
    // plink1 <= plink2, so we don't need to check (plink2, plink1).
    else if (query->disabled_pairs.count(link_pair)) {
        return false; // Keep going.
    }
    // Only check for self collision if links are non-adjacent. Due to a bug
    // in OpenRAVE, links may be neither adjacent nor non-adjacent.
    else if (plink1->GetParent() == plink2->GetParent() 
             && !query->self_enabled_pairs.count(link_pair)) {
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
#ifdef HAVE_REPORT_VLINKCOLLIDING_PAIR
            query->report->vLinkColliding.push_back(std::make_pair(
                query->report->plink1,query->report->plink2));
#else
            query->report->vLinkColliding.push_back(query->report->plink2);
#endif
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

} // namespace or_fcl
