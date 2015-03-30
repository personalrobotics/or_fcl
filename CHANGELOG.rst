^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package or_fcl
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.0 (2015-03-30)
------------------
* Changing cmake to reference src directory instead of include directory
* Made fuerte compatible. bypass broad phase checking.
* Untested self collision checking.
* Replaced some shared_ptr's with raw pointers.
* Handle adjacent links.
* Only compute AABBs when necessary.
* Implemented AttachedDOFs for CheckCollision.
* Implemented AO_ActiveDOFs for CheckSelfCollision
* Support for attachedbodies.
* Change the BVH representation at runtime.
* Tuned BVH and Broad-phase implementations.
* Added options for broad-phase checker.
* Fixed CO_Contacts logic.
* Cleaned up report logic.
* Initialize the collision report.
* Switched to GetAdjacentLinks.
* Implemented a bunch of overloads.
* Cleaned up contact saving code.
* Return contacts.
* Implemented another overload.
* Removed global synchronization function.
* Added a (currently broken) test case.
* Added RunCheck helper function.
* Fixed missing FCL dependency in CMake.
* Implemented the first checkCollision call.
* Reduced number of UserData queries.
* Refactoring.
* Finally ready to implement checkCollision.
* Initialize the FCL UserData.
* Prune some disabled links.
* Added a note about switching to weak_ptr.
* Setup the broad-phase checker.
* Synchronize OpenRAVE geometry to FCL.
* Implemented OpenRAVE -> FCL geometry converter.
* Skeleton implementation.
* Contributors: Jennifer King, Michael Koval
