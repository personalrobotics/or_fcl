#!/usr/bin/env python
import openravepy, numpy

env = openravepy.Environment()
env.Load('/usr/share/openrave-0.9/data/wamtest1.env.xml')
env.SetViewer('qtcoin')

fcl = openravepy.RaveCreateCollisionChecker(env, 'fcl')
env.SetCollisionChecker(fcl)

with env:
    body = env.GetKinBody('BarrettWAM')
    print 'check1 =', env.CheckCollision(body)

with env:
    pose = numpy.eye(4)
    pose[0, 3] = 0.5
    body.SetTransform(pose)
    print 'check2 =', env.CheckCollision(body)
