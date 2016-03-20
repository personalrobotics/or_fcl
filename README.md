# or_fcl

[![Build Status](https://travis-ci.com/personalrobotics/or_fcl.svg?token=37JV1kFUuyHcbn3JjJVG)](https://travis-ci.com/personalrobotics/or_fcl)

[OpenRAVE](http://www.openrave.org/) bindings for the [Flexible Collision
Library](https://github.com/flexible-collision-library/fcl) (FCL).  This
package provides an OpenRAVE [collision checking
plugin](http://openrave.org/docs/latest_stable/coreapihtml/arch_collisionchecker.html)
that uses FCL to perform collision checks.


## Dependencies

This package requires the following dependencies:

- [Boost](http://www.boost.org/)
- [FCL](https://github.com/flexible-collision-library/fcl) (tested with 0.3.2)
- [OpenRAVE](http://www.openrave.org/) (tested with 0.9)


## Installation Instructions

The `CMakeLists.txt` file in the root of this repository supports Catkin and
standalone CMake builds. See the appropriate section below for installation
instructions specific to your environment.


### Catkin Instructions

This preferred way of building or_fcl. In this case, you should already have
OpenRAVE installed as a system dependency.

```console
$ cd /my/workspace
$ git clone https://github.com/personalrobotics/or_fcl.git src/or_fcl
$ . devel/setup.sh
$ catkin_make -DCMAKE_BUILD_TYPE=Release
```

This will build the OpenRAVE plugins into the `share/openrave-0.9/plugins`
directory in your `devel` space. If you run `catkin_make install` the plugin
will be installed to the same directory in your `install` space. In either
case, this package registers a [Catkin environment
hook](http://docs.ros.org/fuerte/api/catkin/html/macros.html#catkin_add_env_hooks)
that adds the installation directory to the `OPENRAVE_PLUGINS` environment
variable. You will need re-source `setup.sh` for this to take effect.

### Standalone Instructions

You can build or_fcl entirely ROS-agnostic by setting the `USE_CATKIN`
variable:

```console
$ git clone https://github.com/personalrobotics/or_fcl.git
$ mkdir -p or_fcl/build
$ cd or_fcl/build
$ cmake -DCMAKE_BUILD_TYPE=Release -DUSE_CATKIN:bool=0 ..
$ make
$ make install
```
This will build the plugin in the `share/openrave-0.9/plugins` directory inside
your `${CMAKE_INSTALL_PREFIX}`, which likely defaults to `/usr/local`. You will
need to append this directory to your `OPENRAVE_PLUGINS` to use it, e.g.
```console
$ export OPENRAVE_PLUGINS="/usr/local/share/openrave-0.9/plugins:${OPENRAVE_PLUGINS}"
```


## Usage

Once or_fcl is installed, you can set FCL as your collision checker:
```python
env = openravepy.Environment()
collision_checker = openravepy.RaveCreateCollisionChecker(env, 'fcl')
env.SetCollisionChecker(collision_checker)
```
Any `CheckCollision` or `CheckSelfCollision` calls on `env` will now use FCL
instead of the default collision checker (typically ODE). See
[`scripts/test.py`](scripts/test.py) for a working example of using or_fcl to
perform a collision check.


## Benchmarks

The collision checking benchmarks in the
[or_benchmarks](https://github.com/personalrobotics/benchmarks/tree/master/or_benchmarks)
package indicate that or_fcl is significantly (3-5 times) faster than
OpenRAVE's included collision checkers for non-trivial queries:

<img src="https://raw.githubusercontent.com/personalrobotics/benchmarks/master/or_benchmarks/results/self_collision.cps.png"
     alt="self_collision_results" width="250"/>
<img src="https://raw.githubusercontent.com/personalrobotics/benchmarks/master/or_benchmarks/results/empty_env_collision.cps.png"
     alt="empty_env_collision_results" width="250"/>
<img src="https://raw.githubusercontent.com/personalrobotics/benchmarks/master/or_benchmarks/results/prkitchen_collision.cps.png"
     alt="prkitchen_env_results" width="250" />

The performance of a collision checker strongly depends on characteristics of
the environment (e.g. types of primitive geometry, convex vs. non-convex
meshes, distance between geometries). As such, these benchmarks may not
reflect the performance of or_fcl for your application.


## Troubleshooting

You may get this warning when calling `RaveCreateCollisionChecker`:
```
[plugindatabase.h:577 Create] Failed to create name fcl, interface collisionchecker
```
This means that the or_fcl plugin is not in a directory listed in the
`OPENRAVE_PLUGINS` environment variable. Try manually appending the
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
