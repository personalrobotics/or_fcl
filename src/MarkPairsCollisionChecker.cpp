/***********************************************************************

Copyright (c) 2016, Carnegie Mellon University
All rights reserved.

Author: Michael Koval <mkoval@cs.cmu.edu>
        Chris Dellin <cdellin@gmail.com>

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

#include "MarkPairsCollisionChecker.h"

or_fcl::MarkPairsCollisionChecker::MarkPairsCollisionChecker(OpenRAVE::EnvironmentBasePtr env)
    : OpenRAVE::CollisionCheckerBase(env)
{
}

or_fcl::MarkPairsCollisionChecker::~MarkPairsCollisionChecker()
{
}

bool or_fcl::MarkPairsCollisionChecker::SetCollisionOptions(int collision_options)
{
    _collision_options = collision_options;
    bool is_supported = true;
    if (_collision_options & OpenRAVE::CO_UseTolerance) {
        RAVELOG_WARN("or_fcl does not support CO_UseTolerance\n");
        is_supported = false;
    }
    if (_collision_options & OpenRAVE::CO_RayAnyHit) {
        RAVELOG_WARN("or_fcl does not support CO_RayAnyHit\n");
        is_supported = false;
    }
    return is_supported;
}

int or_fcl::MarkPairsCollisionChecker::GetCollisionOptions() const
{
    return _collision_options;
}

bool or_fcl::MarkPairsCollisionChecker::InitEnvironment()
{
    return true;
}

void or_fcl::MarkPairsCollisionChecker::DestroyEnvironment()
{
}

bool or_fcl::MarkPairsCollisionChecker::InitKinBody(OpenRAVE::KinBodyPtr body)
{
    return true;
}

void or_fcl::MarkPairsCollisionChecker::RemoveKinBody(OpenRAVE::KinBodyPtr body)
{
}

/* checks collision of a body and a scene.
 * Attached bodies are respected.
 * If CO_ActiveDOFs is set, will only check affected links of the body.
 */
bool or_fcl::MarkPairsCollisionChecker::CheckCollision(
    OpenRAVE::KinBodyConstPtr body, OpenRAVE::CollisionReportPtr report)
{
    static std::vector<OpenRAVE::KinBodyConstPtr> const vbodyexcluded_empty;
    static std::vector<OpenRAVE::KinBody::LinkConstPtr> const vlinkexcluded_empty;

    return CheckCollision(body, vbodyexcluded_empty, vlinkexcluded_empty, report);
} 

/* checks collision between two bodies.
 * Attached bodies are respected.
 * If CO_ActiveDOFs is set, will only check affected links of the pbody1.
 */
bool or_fcl::MarkPairsCollisionChecker::CheckCollision(
    OpenRAVE::KinBodyConstPtr pbody1, OpenRAVE::KinBodyConstPtr pbody2,
    OpenRAVE::CollisionReportPtr report)
{
    // Check to see if either of the bodies is grabbing the other
    // (this would be a self-collision instead!)
    if( pbody1->IsAttached(pbody2) ){
        return false;
    }
    
    // is pbody1 a robot whose activedofs we should respect?
    OpenRAVE::RobotBaseConstPtr robot1;
    if (pbody1->IsRobot() && (_collision_options & OpenRAVE::CO_ActiveDOFs)) {
        robot1 = boost::dynamic_pointer_cast<OpenRAVE::RobotBase const>(pbody1);
    }
    
    // get kinbodies attached to each kinbody
    std::set<OpenRAVE::KinBodyPtr> attached_kinbodies1;
    std::set<OpenRAVE::KinBodyPtr> attached_kinbodies2;
    pbody1->GetAttached(attached_kinbodies1);
    pbody2->GetAttached(attached_kinbodies2);
    
    static boost::unordered_set<OpenRAVE::KinBodyConstPtr> const empty_body_set;
    static boost::unordered_set<OpenRAVE::KinBody::LinkConstPtr> const empty_link_set;
    
    // Group 1: body1 + attached, only check active dofs
    std::set<OpenRAVE::KinBody::LinkConstPtr> group1;
    SynchronizeKinbodies(attached_kinbodies1, empty_body_set, empty_link_set, &group1, robot1);

    // Group 2: body2 + attached, check all links
    std::set<OpenRAVE::KinBody::LinkConstPtr> group2;
    SynchronizeKinbodies(attached_kinbodies2, empty_body_set, empty_link_set, &group2);
    
    // test all pairwise
    for (std::set<OpenRAVE::KinBody::LinkConstPtr>::iterator
        it1=group1.begin(); it1!=group1.end(); it1++)
    {
        for (std::set<OpenRAVE::KinBody::LinkConstPtr>::iterator
            it2=group2.begin(); it2!=group2.end(); it2++)
        {
            _link_pairs.insert(MakeLinkPair(*it1,*it2));
        }
    }
    
    return false;
}

/* checks collision of a link and a scene.
 * Attached bodies are ignored.
 * CO_ActiveDOFs option is ignored.
 */
bool or_fcl::MarkPairsCollisionChecker::CheckCollision(
    OpenRAVE::KinBody::LinkConstPtr plink, OpenRAVE::CollisionReportPtr report)
{
    static std::vector<OpenRAVE::KinBodyConstPtr> const vbodyexcluded_empty;
    static std::vector<OpenRAVE::KinBody::LinkConstPtr> const vlinkexcluded_empty;

    return CheckCollision(plink, vbodyexcluded_empty, vlinkexcluded_empty, report);
}

/* checks collision of two links.
 * Attached bodies are ignored.
 * CO_ActiveDOFs option is ignored.
 */
bool or_fcl::MarkPairsCollisionChecker::CheckCollision(
    OpenRAVE::KinBody::LinkConstPtr link1, OpenRAVE::KinBody::LinkConstPtr link2, OpenRAVE::CollisionReportPtr report)
{
    if (!link1->IsEnabled() || !link2->IsEnabled())
        return false;
    
    _link_pairs.insert(MakeLinkPair(link1, link2));
    
    return false;
}

/* checks collision of a link and a body.
 * Attached bodies for pbody are respected.
 * CO_ActiveDOFs option is ignored.
 */
bool or_fcl::MarkPairsCollisionChecker::CheckCollision(
    OpenRAVE::KinBody::LinkConstPtr plink, OpenRAVE::KinBodyConstPtr pbody, OpenRAVE::CollisionReportPtr report)
{
    // Check to see if either of the bodies is grabbing the other
    // (this would be a self-collision instead!)
    if (!plink->IsEnabled() || plink->GetParent()->IsAttached(pbody)) {
        return false;
    }
    
    static boost::unordered_set<OpenRAVE::KinBodyConstPtr> const empty_body_set;
    static boost::unordered_set<OpenRAVE::KinBody::LinkConstPtr> const empty_link_set;

    // Group 2: body2 + attached, check all links
    std::set<OpenRAVE::KinBody::LinkConstPtr> group2;
    std::set<OpenRAVE::KinBodyPtr> attached_kinbodies2;
    pbody->GetAttached(attached_kinbodies2);
    SynchronizeKinbodies(attached_kinbodies2, empty_body_set, empty_link_set, &group2);
    
    // test all pairwise (group1 = plink)
    for (std::set<OpenRAVE::KinBody::LinkConstPtr>::iterator
        it2=group2.begin(); it2!=group2.end(); it2++)
    {
        _link_pairs.insert(MakeLinkPair(plink,*it2));
    }

    return false;
}

/* checks collision of a link and a scene.
 * Attached bodies are ignored.
 * CO_ActiveDOFs option is ignored.
 */
bool or_fcl::MarkPairsCollisionChecker::CheckCollision(
    OpenRAVE::KinBody::LinkConstPtr plink,
    std::vector<OpenRAVE::KinBodyConstPtr> const &vbodyexcluded,
    std::vector<OpenRAVE::KinBody::LinkConstPtr> const &vlinkexcluded,
    OpenRAVE::CollisionReportPtr report)
{
    boost::unordered_set<OpenRAVE::KinBodyConstPtr> excluded_body_set(
        vbodyexcluded.begin(), vbodyexcluded.end());
    boost::unordered_set<OpenRAVE::KinBody::LinkConstPtr> excluded_link_set(
        vlinkexcluded.begin(), vlinkexcluded.end());
    
    if (!plink->IsEnabled() || excluded_link_set.count(plink))
        return false;
    
    OpenRAVE::KinBodyPtr pbody1 = plink->GetParent();
    
    // exclude kinbodies attached to the link's body (would be self)
    std::set<OpenRAVE::KinBodyPtr> attached_kinbodies1;
    pbody1->GetAttached(attached_kinbodies1);
    for (auto it=attached_kinbodies1.begin(); it!=attached_kinbodies1.end(); it++) {
        excluded_body_set.insert(*it);
    }

    // Group 2: all enabled links that are not excluded or attached to pbody
    std::set<OpenRAVE::KinBody::LinkConstPtr> group2;
    std::vector<OpenRAVE::KinBodyPtr> kinbodies2;
    GetEnv()->GetBodies(kinbodies2);
    for (OpenRAVE::KinBodyPtr const &kinbody2 : kinbodies2) {
        if (kinbody2->IsEnabled() && !excluded_body_set.count(kinbody2)) {
            SynchronizeKinbody(kinbody2, excluded_link_set, &group2);
        }
    }
    
    // test all pairwise (group1 = plink)
    for (std::set<OpenRAVE::KinBody::LinkConstPtr>::iterator
        it2=group2.begin(); it2!=group2.end(); it2++)
    {
        _link_pairs.insert(MakeLinkPair(plink,*it2));
    }

    return false;
}

/* checks collision of a body and a scene.
 * Attached bodies are respected.
 * If CO_ActiveDOFs is set, will only check affected links of pbody.
 */
bool or_fcl::MarkPairsCollisionChecker::CheckCollision(
    OpenRAVE::KinBodyConstPtr pbody, 
    std::vector<OpenRAVE::KinBodyConstPtr> const &vbodyexcluded,
    std::vector<OpenRAVE::KinBody::LinkConstPtr> const &vlinkexcluded,
    OpenRAVE::CollisionReportPtr report)
{
    // convert excluded bodies/links
    boost::unordered_set<OpenRAVE::KinBodyConstPtr> excluded_body_set(
        vbodyexcluded.begin(), vbodyexcluded.end());
    boost::unordered_set<OpenRAVE::KinBody::LinkConstPtr> const excluded_link_set(
        vlinkexcluded.begin(), vlinkexcluded.end());
    
    // is there a robot whose activedofs we should respect?
    OpenRAVE::RobotBaseConstPtr robot;
    if (pbody->IsRobot() && (_collision_options & OpenRAVE::CO_ActiveDOFs)) {
        robot = boost::dynamic_pointer_cast<OpenRAVE::RobotBase const>(pbody);
    }
    
    // get kinbodies attached to pbody
    std::set<OpenRAVE::KinBodyPtr> attached_kinbodies;
    pbody->GetAttached(attached_kinbodies);

    // Group 1: body + attached, respect active dofs.
    std::set<OpenRAVE::KinBody::LinkConstPtr> group1;
    SynchronizeKinbodies(attached_kinbodies, excluded_body_set, excluded_link_set, &group1, robot);
    
    for (auto it=attached_kinbodies.begin(); it!=attached_kinbodies.end(); it++) {
        excluded_body_set.insert(*it);
    }

    // Group 2: all other enabled kinbodies in environment
    std::set<OpenRAVE::KinBody::LinkConstPtr> group2;    
    std::vector<OpenRAVE::KinBodyPtr> kinbodies;
    GetEnv()->GetBodies(kinbodies);
    for (OpenRAVE::KinBodyPtr const &kinbody : kinbodies) {
        if (kinbody->IsEnabled() && !excluded_body_set.count(kinbody)) {
            SynchronizeKinbody(kinbody, excluded_link_set, &group2);
        }
    }

    // test all pairwise
    for (std::set<OpenRAVE::KinBody::LinkConstPtr>::iterator
        it1=group1.begin(); it1!=group1.end(); it1++)
    {
        for (std::set<OpenRAVE::KinBody::LinkConstPtr>::iterator
            it2=group2.begin(); it2!=group2.end(); it2++)
        {
            _link_pairs.insert(MakeLinkPair(*it1,*it2));
        }
    }

    return false;
}

bool or_fcl::MarkPairsCollisionChecker::CheckStandaloneSelfCollision(
    OpenRAVE::KinBodyConstPtr pbody, OpenRAVE::CollisionReportPtr report)
{
    int adjacentoptions = OpenRAVE::KinBody::AO_Enabled;
    if( (_collision_options & OpenRAVE::CO_ActiveDOFs) && pbody->IsRobot() ) {
        adjacentoptions |= OpenRAVE::KinBody::AO_ActiveDOFs;
    }

    const std::set<int> & nonadjacent = pbody->GetNonAdjacentLinks(adjacentoptions);
    
    for (std::set<int>::iterator itset=nonadjacent.begin(); itset!=nonadjacent.end(); itset++)
    {
        OpenRAVE::KinBody::LinkConstPtr plink1(pbody->GetLinks().at(*itset&0xffff));
        OpenRAVE::KinBody::LinkConstPtr plink2(pbody->GetLinks().at(*itset>>16));
        if (!plink1->IsEnabled() || !plink2->IsEnabled())
        {
            continue;
        }
        _link_pairs.insert(MakeLinkPair(plink1, plink2));
    }
    
    return false;
}

bool or_fcl::MarkPairsCollisionChecker::CmdReset(std::ostream & soutput, std::istream & sinput)
{
    Reset();
    return true;
}

bool or_fcl::MarkPairsCollisionChecker::CmdGetMarkedPairs(std::ostream & soutput, std::istream & sinput)
{
    const std::set<LinkPair> & link_pairs = GetMarkedPairs();
    
    std::set< std::pair<std::string,std::string> > str_pairs;
    for (std::set<LinkPair>::const_iterator
        it=link_pairs.begin(); it!=link_pairs.end(); it++)
    {
        std::string s1 = (*it).first->GetName();
        std::string s2 = (*it).second->GetName();
        if (s1 < s2)
            str_pairs.insert(std::make_pair(s1,s2));
        else
            str_pairs.insert(std::make_pair(s2,s1));
    }
    
    for (std::set< std::pair<std::string,std::string> >::const_iterator
        it=str_pairs.begin(); it!=str_pairs.end(); it++)
    {
        soutput << it->first << " " << it->second << std::endl;
    }
    
    return true;
}

std::pair<OpenRAVE::KinBody::LinkConstPtr, OpenRAVE::KinBody::LinkConstPtr>
or_fcl::MarkPairsCollisionChecker::MakeLinkPair(
    OpenRAVE::KinBody::LinkConstPtr link1, OpenRAVE::KinBody::LinkConstPtr link2)
{
    if (link1.get() < link2.get()) {
        return std::make_pair(link1, link2);
    } else {
        return std::make_pair(link2, link1);
    }
}

void or_fcl::MarkPairsCollisionChecker::SynchronizeKinbodies(
    std::set<OpenRAVE::KinBodyPtr> const &attached_kinbodies,
    boost::unordered_set<OpenRAVE::KinBodyConstPtr> const &excluded_kinbody_set,
    boost::unordered_set<OpenRAVE::KinBody::LinkConstPtr> const &excluded_link_set,
    std::set<OpenRAVE::KinBody::LinkConstPtr> * group,
    OpenRAVE::RobotBaseConstPtr robot)
{    
    if (robot) {
        
        // Joints may cover more than one DOF (e.g. spherical joints). First,
        // compute the set of joints that covers the active DOFs.
        std::vector<int> const &active_dofs = robot->GetActiveDOFIndices();
        
        std::set<int> active_joint_indices;
        for (int const &active_dof : active_dofs) {
            OpenRAVE::KinBody::JointPtr const active_joint = robot->GetJointFromDOFIndex(active_dof);
            active_joint_indices.insert(active_joint->GetJointIndex());
        }
        
        // compute inactive links
        boost::unordered_set<OpenRAVE::KinBody::LinkConstPtr> inactive_link_set;
        for (OpenRAVE::KinBody::LinkPtr link : robot->GetLinks()) {
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
        for (OpenRAVE::KinBodyPtr const &kinbody : attached_kinbodies) {
            if (!kinbody->IsEnabled() || excluded_kinbody_set.count(kinbody)) {
                continue;
            }
            
            // if this is the robot itself,
            // include the inactive links we calculated into its exclude set
            if (kinbody == robot) {
                boost::unordered_set<OpenRAVE::KinBody::LinkConstPtr> robot_excluded_link_set = excluded_link_set;
                robot_excluded_link_set.insert(inactive_link_set.begin(), inactive_link_set.end());
                SynchronizeKinbody(kinbody, robot_excluded_link_set, group);
                continue;
            }
            
            // if the kinbody is grabbed by an inactive robot link, skip it
            OpenRAVE::KinBody::LinkPtr const robot_link = robot->IsGrabbing(kinbody);
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
        for (OpenRAVE::KinBodyPtr const &kinbody : attached_kinbodies) {
            if (!kinbody->IsEnabled() || excluded_kinbody_set.count(kinbody)) {
                continue;
            }
            SynchronizeKinbody(kinbody, excluded_link_set, group);
        }
    }
}

void or_fcl::MarkPairsCollisionChecker::SynchronizeKinbody(
    OpenRAVE::KinBodyConstPtr const &kinbody,
    boost::unordered_set<OpenRAVE::KinBody::LinkConstPtr> const &excluded_link_set,
    std::set<OpenRAVE::KinBody::LinkConstPtr> * group)
{

    // synchronize all non-excluded links
    for (OpenRAVE::KinBody::LinkPtr const &link : kinbody->GetLinks()) {
        if (!link->IsEnabled() || excluded_link_set.count(link)) {
            continue;
        }
        group->insert(link);
    }
}
