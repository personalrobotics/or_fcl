#ifndef FCLCOLLISIONCHECKER_H_
#define FCLCOLLISIONCHECKER_H_
#include <boost/unordered_map.hpp>
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

    virtual bool SetCollisionOptions(int collision_options) OR_FCL_DUMMY_IMPLEMENTATION;
    virtual int GetCollisionOptions() const OR_FCL_DUMMY_IMPLEMENTATION;
    virtual void SetTolerance(OpenRAVE::dReal tolerance) OR_FCL_DUMMY_IMPLEMENTATION;

    virtual bool InitEnvironment();
    virtual void DestroyEnvironment();

    virtual bool InitKinBody(OpenRAVE::KinBodyPtr pbody);
    virtual void RemoveKinBody(OpenRAVE::KinBodyPtr pbody);

    virtual bool CheckCollision(
        KinBodyConstPtr pbody1, CollisionReportPtr report = CollisionReportPtr()
    );
    virtual bool CheckCollision(
        KinBodyConstPtr pbody1, KinBodyConstPtr pbody2,
        CollisionReportPtr report = CollisionReportPtr()
    );
    virtual bool CheckCollision(
        LinkConstPtr plink, CollisionReportPtr report = CollisionReportPtr()
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
    ) OR_FCL_DUMMY_IMPLEMENTATION;
    virtual bool CheckCollision(
        KinBodyConstPtr pbody,
        std::vector<KinBodyConstPtr> const &vbodyexcluded,
        std::vector<LinkConstPtr> const &vlinkexcluded,
        CollisionReportPtr report = CollisionReportPtr()
    ) OR_FCL_DUMMY_IMPLEMENTATION;
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
    );

private:
    typedef boost::shared_ptr<fcl::BroadPhaseCollisionManager> BroadPhaseCollisionManagerPtr;
    typedef fcl::BVHModel<fcl::OBBRSS> BVHModel;
    typedef boost::shared_ptr<BVHModel> BVHModelPtr;
    typedef boost::shared_ptr<fcl::CollisionGeometry> CollisionGeometryPtr;
    typedef std::vector<fcl::CollisionObject *> CollisionGroup;

    std::string user_data_;
    int num_contacts_;
    int options_;
    BroadPhaseCollisionManagerPtr manager1_, manager2_;

    FCLUserDataPtr GetCollisionData(KinBodyConstPtr const &body) const;

    bool RunCheck(CollisionReportPtr report);

    void Synchronize(KinBodyConstPtr const &body,
                     CollisionGroup *group = NULL);
    void Synchronize(FCLUserDataPtr const &user_data, KinBodyConstPtr const &body,
                     CollisionGroup *group = NULL);
    void Synchronize(LinkConstPtr const &body,
                     CollisionGroup *group = NULL);
    void Synchronize(FCLUserDataPtr const &user_data, LinkConstPtr const &body,
                     CollisionGroup *group= NULL);

    static LinkConstPtr GetCollisionLink(fcl::CollisionObject const &o);
    static bool NarrowPhaseCheckCollision(
        fcl::CollisionObject *o1, fcl::CollisionObject *o2, void *data
    );

    static fcl::Vec3f ConvertVectorToFCL(OpenRAVE::Vector const &v);
    static fcl::Quaternion3f ConvertQuaternionToFCL(OpenRAVE::Vector const &v);
    static CollisionGeometryPtr ConvertGeometryToFCL(GeometryConstPtr const &geom);

    static OpenRAVE::Vector ConvertVectorToOR(fcl::Vec3f const &v);
    static OpenRAVE::CollisionReport::CONTACT ConvertContactToOR(
            fcl::Contact const &contact);


};

}

#endif
