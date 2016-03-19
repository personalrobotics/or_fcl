# or_fcl

[![Build Status](https://travis-ci.com/personalrobotics/or_fcl.svg?token=37JV1kFUuyHcbn3JjJVG)](https://travis-ci.com/personalrobotics/or_fcl)

[OpenRAVE](http://www.openrave.org/) bindings for the [Flexible Collision
Checking Library](https://github.com/flexible-collision-library/fcl) (FCL).
This package provides an OpenRAVE [collision checking
plugin](http://openrave.org/docs/latest_stable/coreapihtml/arch_collisionchecker.html)
that uses FCL to perform collision checks.


## Dependencies

This package requires the following dependencies:

- [Boost](http://www.boost.org/)
- [FCL](https://github.com/flexible-collision-library/fcl) (tested with 0.3.2)
- [OpenRAVE](http://www.openrave.org/) (tested with 0.9)

If you are building or_fcl in a Catkin workspace, you may optionally install
the [openrave_catkin](https://github.com/personalrobotics/openrave_catkin)
package to automatically manage your `OPENRAVE_PLUGINS` environment variable.


## Installation Instructions

The `CMakeLists.txt` file in the root of this repository supports Catkin and
standalone CMake builds. See the appropriate section below for installation
instructions specific to your environment.


### Catkin Instructions

This preferred way of building or_fcl. In this case, you should already have
OpenRAVE installed as a system dependency.

```bash
$ . /my/workspace/devel/setup.bash
$ cd /my/workspace/src
$ git clone https://github.com/personalrobotics/openrave_catkin.git
$ git clone https://github.com/personalrobotics/or_fcl.git
$ cd ..
$ catkin_make
```

This will build the OpenRAVE plugins into the `share/openrave-0.9/plugins`
directory in your `devel` space. If you run `catkin_make install` the plugin
will be installed to the same directory in your `install` space. In both
cases, if you are using
[openrave_catkin](https://github.com/personalrobotics/openrave_catkin), the
corresponding directory will be automatically added to your `OPENRAVE_PLUGINS`
path using a Catkin environment hook.


### Standalone Instructions

You can build or_fcl entirely ROS-agnostic by setting the `USE_CATKIN`
variable:

```bash
$ git clone https://github.com/personalrobotics/or_fcl.git
$ mkdir build
$ cd build
$ cmake -DUSE_CATKIN:bool=0 ..
$ make
```
This will build the plugin in the `lib/` directory.  You will need to add this
directory to your `OPENRAVE_PLUGINS` path so that OpenRAVE can find it.


## Usage

Once or_fcl is installed, you can set FCL as your collision checker:
```python
env = openravepy.Environment()
collision_checker = openravepy.RaveCreateCollisionChecker(env, 'fcl')
env.SetCollisionChecker(collision_checker)
```
Any `CheckCollision` or `CheckSelfCollision` calls on `env` will now use FCL
instead of the default collision checker (typically ODE). See
[`scripts/text.py`](scripts/text.py) for a working example of using or_fcl to
perform a collision check.


## Troubleshooting

You may get this warning when calling `RaveCreateCollisionChecker`:
```
[plugindatabase.h:577 Create] Failed to create name fcl, interface collisionchecker
```
This means that the or_fcl plugin is not in your `OPENRAVE_PLUGINS` path. If
you are using `openrave_catkin`, try re-sourcing `setup.bash` in your Catkin
workspace. If you are using a standalone build, try manually appending the
`share/openrave-0.9/plugins` directory in your `CMAKE_INSTALL_PREFIX` to the
`OPENRAVE_PLUGINS` environment variable.


## License

or_fcl is licensed under a BSD license. See [`LICENSE`](LICENSE) for more
information.


## Contributors

or_fcl was developed by the
[Personal Robotics Lab](https://personalrobotics.ri.cmu.edu) in the
[Robotics Institute](http://ri.cmu.edu) at
[Carnegie Mellon University](http://www.cmu.edu). This plugin was written by
[Michael Koval](http://mkoval.org) with contributions from
[Chris Dellin](http://www.ri.cmu.edu/person.html?person_id=2267) and
[Jennifer King](http://www.ri.cmu.edu/person.html?person_id=2915).
