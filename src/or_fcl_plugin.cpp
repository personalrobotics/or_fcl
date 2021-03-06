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
#include <boost/make_shared.hpp>
#include <openrave/plugin.h>
#include "MarkPairsCollisionChecker.h"
#include "FCLCollisionChecker.h"

using OpenRAVE::EnvironmentBasePtr;
using OpenRAVE::InterfaceBasePtr;
using OpenRAVE::InterfaceType;
using OpenRAVE::PT_KinBody;
using OpenRAVE::PT_CollisionChecker;
using OpenRAVE::PLUGININFO;
using or_fcl::FCLCollisionChecker;

InterfaceBasePtr CreateInterfaceValidated(
        InterfaceType type, std::string const &interface_name,
        std::istream &input, EnvironmentBasePtr env)
{
    if (type == PT_CollisionChecker && interface_name == "fcl") {
        return boost::make_shared<FCLCollisionChecker>(env);
    }
    if (type == PT_CollisionChecker && interface_name == "fcl_mark_pairs") {
        return boost::make_shared<or_fcl::MarkPairsCollisionChecker>(env);
    }
    if (type == PT_KinBody && interface_name == "fcl_baked") {
        boost::shared_ptr<FCLCollisionChecker> cc
            = boost::dynamic_pointer_cast<FCLCollisionChecker>(
                env->GetCollisionChecker());
        if (!cc)
            return InterfaceBasePtr();
        return cc->BakeAttachKinBody();
    }
    return InterfaceBasePtr();
}

void GetPluginAttributesValidated(PLUGININFO &info)
{
    info.interfacenames[PT_CollisionChecker].push_back("fcl");
    info.interfacenames[PT_CollisionChecker].push_back("fcl_mark_pairs");
    info.interfacenames[PT_KinBody].push_back("fcl_baked");
}

RAVE_PLUGIN_API void DestroyPlugin()
{
}
