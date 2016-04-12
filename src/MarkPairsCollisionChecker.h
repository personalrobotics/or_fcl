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

#ifndef MARKPAIRSCOLLISIONCHECKER_H_
#define MARKPAIRSCOLLISIONCHECKER_H_

#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>
#include <openrave/openrave.h>
#include <fcl/collision_data.h>
#include <fcl/BVH/BVH_model.h>
#include <fcl/broadphase/broadphase.h>
#include <fcl/collision_object.h>

#define OR_FCL_MARK_DUMMY_IMPLEMENTATION { throw OpenRAVE::openrave_exception("not implemented", OpenRAVE::ORE_NotImplemented); }

namespace or_fcl {

/*
 * MarkPairsCollisionChecker
 * this checker maintains no resources in the environment;
 * therefore, the following are gauranteed to be no-ops:
 *    InitEnvironment()
 *    DestroyEnvironment()
 *    InitKinBody(kb)
 *    RemoveKinBody(kb)
 */
class MarkPairsCollisionChecker : public OpenRAVE::CollisionCheckerBase {
public:
    MarkPairsCollisionChecker(OpenRAVE::EnvironmentBasePtr env);
    virtual ~MarkPairsCollisionChecker();

    virtual bool SetCollisionOptions(int collision_options);
    virtual int GetCollisionOptions() const;
    virtual void SetTolerance(OpenRAVE::dReal tolerance) OR_FCL_MARK_DUMMY_IMPLEMENTATION;

    virtual bool InitEnvironment();
    virtual void DestroyEnvironment();

    virtual bool InitKinBody(OpenRAVE::KinBodyPtr pbody);
    virtual void RemoveKinBody(OpenRAVE::KinBodyPtr pbody);

    virtual bool CheckCollision(
        OpenRAVE::KinBodyConstPtr pbody1,
        OpenRAVE::CollisionReportPtr report = OpenRAVE::CollisionReportPtr()
    );
    virtual bool CheckCollision(
        OpenRAVE::KinBodyConstPtr pbody1, OpenRAVE::KinBodyConstPtr pbody2,
        OpenRAVE::CollisionReportPtr report = OpenRAVE::CollisionReportPtr()
    );
    virtual bool CheckCollision(
        OpenRAVE::KinBody::LinkConstPtr plink,
        OpenRAVE::CollisionReportPtr report = OpenRAVE::CollisionReportPtr()
    );
    virtual bool CheckCollision(
        OpenRAVE::KinBody::LinkConstPtr plink1, OpenRAVE::KinBody::LinkConstPtr plink2,
        OpenRAVE::CollisionReportPtr report = OpenRAVE::CollisionReportPtr()
    );
    virtual bool CheckCollision(
        OpenRAVE::KinBody::LinkConstPtr plink, OpenRAVE::KinBodyConstPtr pbody,
        OpenRAVE::CollisionReportPtr report = OpenRAVE::CollisionReportPtr()
    );
    virtual bool CheckCollision(
        OpenRAVE::KinBody::LinkConstPtr plink,
        std::vector<OpenRAVE::KinBodyConstPtr> const &vbodyexcluded,
        std::vector<OpenRAVE::KinBody::LinkConstPtr> const &vlinkexcluded,
        OpenRAVE::CollisionReportPtr report = OpenRAVE::CollisionReportPtr()
    );
    virtual bool CheckCollision(
        OpenRAVE::KinBodyConstPtr pbody,
        std::vector<OpenRAVE::KinBodyConstPtr> const &vbodyexcluded,
        std::vector<OpenRAVE::KinBody::LinkConstPtr> const &vlinkexcluded,
        OpenRAVE::CollisionReportPtr report = OpenRAVE::CollisionReportPtr()
    );
    virtual bool CheckCollision(
        OpenRAVE::RAY const &ray, OpenRAVE::KinBody::LinkConstPtr plink,
        OpenRAVE::CollisionReportPtr report = OpenRAVE::CollisionReportPtr()
    ) OR_FCL_MARK_DUMMY_IMPLEMENTATION;
    virtual bool CheckCollision(
        OpenRAVE::RAY const &ray, OpenRAVE::KinBodyConstPtr pbody,
        OpenRAVE::CollisionReportPtr report = OpenRAVE::CollisionReportPtr()
    ) OR_FCL_MARK_DUMMY_IMPLEMENTATION;
    virtual bool CheckCollision(
        OpenRAVE::RAY const &ray,
        OpenRAVE::CollisionReportPtr report = OpenRAVE::CollisionReportPtr()
    ) OR_FCL_MARK_DUMMY_IMPLEMENTATION;

    virtual bool CheckStandaloneSelfCollision(
        OpenRAVE::KinBodyConstPtr pbody,
        OpenRAVE::CollisionReportPtr report = OpenRAVE::CollisionReportPtr()
    );
    virtual bool CheckStandaloneSelfCollision(
        OpenRAVE::KinBody::LinkConstPtr plink,
        OpenRAVE::CollisionReportPtr report = OpenRAVE::CollisionReportPtr()
    ) OR_FCL_MARK_DUMMY_IMPLEMENTATION;

    // guaranteeed that pair.first.get() < pair.second.get()
    typedef std::pair<OpenRAVE::KinBody::LinkConstPtr,
                      OpenRAVE::KinBody::LinkConstPtr> LinkPair;

    virtual void Reset()
    {
        _link_pairs.clear();
    }
    
    virtual const std::set<LinkPair> & GetMarkedPairs() const
    {
        return _link_pairs;
    }

    virtual bool CmdReset(std::ostream & soutput, std::istream & sinput);
    virtual bool CmdGetMarkedPairs(std::ostream & soutput, std::istream & sinput);

private:
    int _collision_options;

    std::set<LinkPair> _link_pairs;

    static std::pair<OpenRAVE::KinBody::LinkConstPtr, OpenRAVE::KinBody::LinkConstPtr>
    MakeLinkPair(
        OpenRAVE::KinBody::LinkConstPtr,
        OpenRAVE::KinBody::LinkConstPtr);

    /* If robot is passed, then any of its inactive links,
     * or grabbed kinbody attached to one of its inactive links,
     * will not be synchronized into the group.
     * This method is usually used for pbody1 as a left argument to
     * CheckCollision().
     * This will check for the enabled status of all children before
     * synchronizing them.
     */
    void SynchronizeKinbodies(
        std::set<OpenRAVE::KinBodyPtr> const &attached_kinbodies,
        boost::unordered_set<OpenRAVE::KinBodyConstPtr> const &excluded_kinbody_set,
        boost::unordered_set<OpenRAVE::KinBody::LinkConstPtr> const &excluded_link_set,
        std::set<OpenRAVE::KinBody::LinkConstPtr> * group,
        OpenRAVE::RobotBaseConstPtr robot = OpenRAVE::RobotBaseConstPtr()
    );

    /* Add all links in this kinbody, except if they're in the
     * excluded_link_set.
     * (Caller will add inactive links there, e.g. in case CO_ActiveDOFS is set)
     * Will not checked if the kinbody is enabled (do that yourself!) 
     */
    void SynchronizeKinbody(
        OpenRAVE::KinBodyConstPtr const &kinbody,
        boost::unordered_set<OpenRAVE::KinBody::LinkConstPtr> const &excluded_link_set,
        std::set<OpenRAVE::KinBody::LinkConstPtr> * group
    );
};

} // namespace or_fcl

#endif // MARKPAIRSCOLLISIONCHECKER_H_
