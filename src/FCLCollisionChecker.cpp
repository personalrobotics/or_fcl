#include "FCLCollisionChecker.h"

namespace or_fcl {

FCLCollisionChecker::FCLCollisionChecker(OpenRAVE::EnvironmentBasePtr env)
    : OpenRAVE::CollisionCheckerBase(env)
{
}

FCLCollisionChecker::~FCLCollisionChecker()
{
}

}
