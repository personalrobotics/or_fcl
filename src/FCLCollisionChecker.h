#ifndef FCLCOLLISIONCHECKER_H_
#define FCLCOLLISIONCHECKER_H_
#include <openrave/openrave.h>
#include <fcl/BVH/BVH_model.h>
#include <fcl/collision_object.h>

#define OR_FCL_DUMMY_IMPLEMENTATION { throw OpenRAVE::openrave_exception("not implemented", OpenRAVE::ORE_NotImplemented); }

namespace or_fcl {

class FCLCollisionChecker : public OpenRAVE::CollisionCheckerBase {
public:
    typedef OpenRAVE::KinBodyConstPtr KinBodyConstPtr;
    typedef OpenRAVE::KinBody::LinkConstPtr LinkConstPtr;
    typedef OpenRAVE::KinBody::Link::GeometryConstPtr GeometryConstPtr;
    typedef OpenRAVE::CollisionReportPtr CollisionReportPtr;
    typedef OpenRAVE::RAY RAY;
    typedef fcl::BVHModel<fcl::OBBRSS> BVHModel;
    typedef boost::shared_ptr<BVHModel> BVHModelPtr;

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
    ) OR_FCL_DUMMY_IMPLEMENTATION;
    virtual bool CheckCollision(
        KinBodyConstPtr pbody1, KinBodyConstPtr pbody2,
        CollisionReportPtr report = CollisionReportPtr()
    ) OR_FCL_DUMMY_IMPLEMENTATION;
    virtual bool CheckCollision(
        LinkConstPtr plink, CollisionReportPtr report = CollisionReportPtr()
    ) OR_FCL_DUMMY_IMPLEMENTATION;
    virtual bool CheckCollision(
        LinkConstPtr plink1, LinkConstPtr plink2,
        CollisionReportPtr report = CollisionReportPtr()
    ) OR_FCL_DUMMY_IMPLEMENTATION;
    virtual bool CheckCollision(
        LinkConstPtr plink, KinBodyConstPtr pbody,
        CollisionReportPtr report = CollisionReportPtr()
    ) OR_FCL_DUMMY_IMPLEMENTATION;
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
    ) OR_FCL_DUMMY_IMPLEMENTATION;
    virtual bool CheckStandaloneSelfCollision(
        LinkConstPtr plink,
        CollisionReportPtr report = CollisionReportPtr()
    ) OR_FCL_DUMMY_IMPLEMENTATION;

private:
    typedef boost::shared_ptr<fcl::CollisionGeometry> CollisionGeometryPtr;

    std::string user_data_;

    CollisionGeometryPtr ConvertGeometryToFCL(GeometryConstPtr const &geom) const;


};

}

#endif