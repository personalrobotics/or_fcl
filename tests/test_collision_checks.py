import unittest

class ORFCLTest(unittest.TestCase):
    is_setup=False

    def setUp(self):
        import openravepy
        if not self.is_setup:
            openravepy.RaveInitialize(True)
            self.penv = openravepy.Environment()
            self.cchecker = openravepy.RaveCreateCollisionChecker(self.penv, "fcl")
            self.penv.SetCollisionChecker(self.cchecker)
    
    def tearDown(self):
        for body in self.penv.GetBodies():
            self.penv.Remove(body)
        self.penv.Destroy()

    def loadObject(self, obj_xml_file, obj_name=None):
        '''
        Find and load the object
        @param obj_xml_file The kinbody xml file that describes the object
        @param obj_name The name to assign to the body
        '''
        from catkin.find_in_workspaces import find_in_workspaces
    
        paths = find_in_workspaces(project="or_fcl", search_dirs=['share'],
                                   path="tests/ordata", first_match_only=True)

        if not paths:
            return None
        
        path = paths[0]

        import os
        body = self.penv.ReadKinBodyXMLFile(os.path.join(path, obj_xml_file))
        if body is None:
            return None
        if obj_name is not None:
            body.SetName(obj_name)
        self.penv.Add(body)
        return body

    def assertChecks(self, b1, b2, assert_true=True):
        if assert_true:
            self.assertTrue(self.penv.CheckCollision(b1, b2))
            self.assertTrue(self.penv.CheckCollision(b1))
            self.assertTrue(self.penv.CheckCollision(b2))
        else:
            self.assertFalse(self.penv.CheckCollision(b1, b2))
            self.assertFalse(self.penv.CheckCollision(b1))
            self.assertFalse(self.penv.CheckCollision(b2))

    def test_collisionCheckBoxes(self):
        
        # First load the boxes
        box1 = self.loadObject("box.kinbody.xml", "box1")
        assert box1 is not None
        box2 = self.loadObject("box.kinbody.xml", "box2")
        assert box2 is not None

        # Next set them to be in collision
        b1_pose = box1.GetTransform()
        b1_pose[:3,3] = [1., 0., 0.]
        box1.SetTransform(b1_pose)

        b2_pose = box2.GetTransform()
        b2_pose[:3,3] = [.5, 0., 0.]
        box2.SetTransform(b2_pose)
        
        self.assertChecks(box1, box2, assert_true=True)

        # Now move them out of collision and assert false
        b2_pose[:3,3] = [3.01, 0., 0.]
        box2.SetTransform(b2_pose)
        self.assertChecks(box1, box2, assert_true=False)

    def test_collisionCheckSpheres(self):
        
        # First load the spheres
        sphere1 = self.loadObject("sphere.kinbody.xml", "sphere1")
        assert sphere1 is not None
        sphere2 = self.loadObject("sphere.kinbody.xml", "sphere2")
        assert sphere2 is not None

        s1_pose = sphere1.GetTransform()
        s2_pose = sphere2.GetTransform()

        # First randomly select several poses for the second sphere that are in collision
        import random, numpy
        for idx in range(10):            
            s1_pose[:3,3] = numpy.array([random.uniform(-5., 5.),
                                         random.uniform(-5., 5.),
                                         random.uniform(-5., 5.)])
            rad = random.uniform(0., 2.)
            a = random.uniform(0., 2.*numpy.pi)
            s2_pose[:3,3] = numpy.array([rad*numpy.cos(a), rad*numpy.sin(a), 0.]) + s1_pose[:3,3]

            sphere1.SetTransform(s1_pose)
            sphere2.SetTransform(s2_pose)
        
            self.assertChecks(sphere1, sphere2, assert_true=True)
        
        # Now select several that are out of collision
        for idx in range(10):            
            s1_pose[:3,3] = numpy.array([random.uniform(-5., 5.),
                                         random.uniform(-5., 5.),
                                         random.uniform(-5., 5.)])
            rad = random.uniform(2., 5.)
            a = random.uniform(0., 2.*numpy.pi)
            s2_pose[:3,3] = numpy.array([rad*numpy.cos(a), rad*numpy.sin(a), 0.]) + s1_pose[:3,3]

            sphere1.SetTransform(s1_pose)
            sphere2.SetTransform(s2_pose)
        
            self.assertChecks(sphere1, sphere2, assert_true=False)

    def test_collisionCheckCylinders(self):
        # First load the cylinders
        cylinder1 = self.loadObject("cylinder.kinbody.xml", "cylinder1")
        assert cylinder1 is not None
        cylinder2 = self.loadObject("cylinder.kinbody.xml", "cylinder2")
        assert cylinder2 is not None

        c1_pose = cylinder1.GetTransform()
        c2_pose = cylinder2.GetTransform()

        # First put the cylinders in collision and check
        c1_pose[:3,3] = [0., 0., 0.]
        c2_pose[:3,3] = [0.5, 0., 0.]
        cylinder1.SetTransform(c1_pose)
        cylinder2.SetTransform(c2_pose)
        self.assertChecks(cylinder1, cylinder2, assert_true=True)

        c2_pose[:3,3] = [2.01, 0., 0.]
        cylinder2.SetTransform(c2_pose)
        self.assertChecks(cylinder1, cylinder2, assert_true=False)

        # Height direction
        c2_pose[:3,3] = [0., 1.01, 0.]
        cylinder2.SetTransform(c2_pose)
        self.assertChecks(cylinder1, cylinder2, assert_true=False)

        c2_pose[:3,3] = [0., 0.99, 0.]
        cylinder2.SetTransform(c2_pose)
        self.assertChecks(cylinder1, cylinder2, assert_true=True)
        

if __name__ == '__main__':
    unittest.main()
