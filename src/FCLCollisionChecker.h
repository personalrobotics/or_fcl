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
#ifndef FCLCOLLISIONCHECKER_H_
#define FCLCOLLISIONCHECKER_H_
#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>
#include <openrave/openrave.h>
#include <fcl/collision_data.h>
#include <fcl/BVH/BVH_model.h>
#include <fcl/broadphase/broadphase.h>
#include <fcl/collision_object.h>

#define OR_FCL_DUMMY_IMPLEMENTATION { throw OpenRAVE::openrave_exception("not implemented", OpenRAVE::ORE_NotImplemented); }

namespace or_fcl {

/*
 * FCLUserData
 */
struct FCLUserData : public OpenRAVE::UserData {
    // TODO: Switch to a real weak_ptr. This requires defining a hash function
    // for boost::weak_ptr<T>.
    //typedef boost::weak_ptr<OpenRAVE::KinBody::Link::Geometry const>
    //            GeometryConstWeakPtr;
    typedef OpenRAVE::KinBody::Link::Geometry const *GeometryConstWeakPtr;
    typedef boost::shared_ptr<fcl::CollisionObject> CollisionObjectPtr;

    virtual ~FCLUserData();

    boost::unordered_map<GeometryConstWeakPtr, CollisionObjectPtr> geometries;
};

typedef boost::shared_ptr<FCLUserData> FCLUserDataPtr;


/*
 * FCLCollisionChecker
 */
class FCLCollisionChecker : public OpenRAVE::CollisionCheckerBase {
public:
    typedef OpenRAVE::KinBodyConstPtr KinBodyConstPtr;
    typedef OpenRAVE::KinBody::LinkConstPtr LinkConstPtr;
    typedef OpenRAVE::KinBody::Link::GeometryConstPtr GeometryConstPtr;
    typedef OpenRAVE::CollisionReportPtr CollisionReportPtr;
    typedef OpenRAVE::RAY RAY;

    FCLCollisionChecker(OpenRAVE::EnvironmentBasePtr env);
    virtual ~FCLCollisionChecker();

    virtual bool SetCollisionOptions(int collision_options);
    virtual int GetCollisionOptions() const;
    virtual void SetTolerance(OpenRAVE::dReal tolerance) OR_FCL_DUMMY_IMPLEMENTATION;

    void SetBroadphaseAlgorithm(std::string const &algorithm);
    void SetBVHRepresentation(std::string const &type);

    virtual bool InitEnvironment();
    virtual void DestroyEnvironment();

    virtual bool InitKinBody(OpenRAVE::KinBodyPtr pbody);
    virtual void RemoveKinBody(OpenRAVE::KinBodyPtr pbody);

    virtual bool CheckCollision(
        KinBodyConstPtr pbody1,
        CollisionReportPtr report = CollisionReportPtr()
    );
    virtual bool CheckCollision(
        KinBodyConstPtr pbody1, KinBodyConstPtr pbody2,
        CollisionReportPtr report = CollisionReportPtr()
    );
    virtual bool CheckCollision(
        LinkConstPtr plink,
        CollisionReportPtr report = CollisionReportPtr()
    );
    virtual bool CheckCollision(
        LinkConstPtr plink1, LinkConstPtr plink2,
        CollisionReportPtr report = CollisionReportPtr()
    );
    virtual bool CheckCollision(
        LinkConstPtr plink, KinBodyConstPtr pbody,
        CollisionReportPtr report = CollisionReportPtr()
    );
    virtual bool CheckCollision(
        LinkConstPtr plink,
        std::vector<KinBodyConstPtr> const &vbodyexcluded,
        std::vector<LinkConstPtr> const &vlinkexcluded,
        CollisionReportPtr report = CollisionReportPtr()
    );
    virtual bool CheckCollision(
        KinBodyConstPtr pbody,
        std::vector<KinBodyConstPtr> const &vbodyexcluded,
        std::vector<LinkConstPtr> const &vlinkexcluded,
        CollisionReportPtr report = CollisionReportPtr()
    );
    virtual bool CheckCollision(
        RAY const &ray, LinkConstPtr plink,
        CollisionReportPtr report = CollisionReportPtr()
    ) OR_FCL_DUMMY_IMPLEMENTATION;
    virtual bool CheckCollision(
        RAY const &ray, KinBodyConstPtr pbody,
        CollisionReportPtr report = CollisionReportPtr()
    ) OR_FCL_DUMMY_IMPLEMENTATION;
    virtual bool CheckCollision(
        RAY const &ray,
        CollisionReportPtr report = CollisionReportPtr()
    ) OR_FCL_DUMMY_IMPLEMENTATION;

    virtual bool CheckStandaloneSelfCollision(
        KinBodyConstPtr pbody,
        CollisionReportPtr report = CollisionReportPtr()
    );
    virtual bool CheckStandaloneSelfCollision(
        LinkConstPtr plink,
        CollisionReportPtr report = CollisionReportPtr()
    ) OR_FCL_DUMMY_IMPLEMENTATION;

private:
    typedef boost::shared_ptr<fcl::BroadPhaseCollisionManager> BroadPhaseCollisionManagerPtr;
    typedef boost::shared_ptr<fcl::CollisionGeometry> CollisionGeometryPtr;
    typedef std::vector<fcl::CollisionObject *> CollisionGroup;
    typedef boost::function<
        CollisionGeometryPtr (std::vector<fcl::Vec3f> const &points,
                              std::vector<fcl::Triangle> const &triangles)
        > MeshFactory;
    typedef std::pair<OpenRAVE::KinBody::Link const *,
                      OpenRAVE::KinBody::Link const *> LinkPair;

    std::string user_data_;
    int num_contacts_;
    int options_;
    MeshFactory mesh_factory_;
    BroadPhaseCollisionManagerPtr manager1_, manager2_;

    FCLUserDataPtr GetCollisionData(OpenRAVE::KinBodyConstPtr const &body) const;

    bool RunCheck(
        CollisionReportPtr report,
        boost::unordered_set<LinkPair> const &disabled_pairs
            = boost::unordered_set<LinkPair>(),
        boost::unordered_set<LinkPair> const &self_enabled_pairs
            = boost::unordered_set<LinkPair>()
    );

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
        boost::unordered_set<KinBodyConstPtr> const &excluded_kinbody_set,
        boost::unordered_set<LinkConstPtr> const &excluded_link_set,
        CollisionGroup *group,
        OpenRAVE::RobotBaseConstPtr robot = OpenRAVE::RobotBaseConstPtr()
    );

    /* Add all links in this kinbody, except if they're in the
     * excluded_link_set.
     * (Caller will add inactive links there, e.g. in case CO_ActiveDOFS is set)
     * Will not checked if the kinbody is enabled (do that yourself!) 
     */
    void SynchronizeKinbody(
        OpenRAVE::KinBodyConstPtr const &kinbody,
        boost::unordered_set<LinkConstPtr> const &excluded_link_set,
        CollisionGroup *group
    );

    /* Synchronize a single link.
     * Will not checked if the link is enabled (do that yourself!) */
    void SynchronizeLink(
        FCLUserDataPtr const &user_data,
        OpenRAVE::KinBody::Link const *link,
        CollisionGroup *group
    );

    void UnpackLinkPairs(
        std::set<int> const &packed,
        std::vector<std::pair<int, int> > *unpacked) const;
    void UnpackLinkPairs(
        OpenRAVE::KinBodyConstPtr const &body, std::set<int> const &packed,
        std::vector<std::pair<OpenRAVE::KinBody::Link const *,
                              OpenRAVE::KinBody::Link const *> > *unpacked) const;

    static std::pair<OpenRAVE::KinBody::Link const *,
              OpenRAVE::KinBody::Link const *> MakeLinkPair(
        OpenRAVE::KinBody::Link const *link1,
        OpenRAVE::KinBody::Link const *link2);

    static LinkConstPtr GetCollisionLink(fcl::CollisionObject const &o);
    static bool NarrowPhaseCheckCollision(
        fcl::CollisionObject *o1, fcl::CollisionObject *o2, void *data
    );

    static fcl::Vec3f ConvertVectorToFCL(OpenRAVE::Vector const &v);
    static fcl::Quaternion3f ConvertQuaternionToFCL(OpenRAVE::Vector const &v);
    static CollisionGeometryPtr ConvertGeometryToFCL(
        MeshFactory const &mesh_factory, GeometryConstPtr const &geom
    );

    template <class T>
    static CollisionGeometryPtr ConvertMeshToFCL(
        std::vector<fcl::Vec3f> const &points,
        std::vector<fcl::Triangle> const &triangles
    );

    static OpenRAVE::Vector ConvertVectorToOR(fcl::Vec3f const &v);
    static OpenRAVE::CollisionReport::CONTACT ConvertContactToOR(
            fcl::Contact const &contact);

    bool AreEqual(fcl::Vec3f const &x, fcl::Vec3f const &y) const;
    bool AreEqual(fcl::Quaternion3f const &x, fcl::Quaternion3f const &y) const;
};

}

#endif
