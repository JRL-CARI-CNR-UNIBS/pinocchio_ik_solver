^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pinocchio_ik_solver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.0 (2026-10-22)
------------------
* First release: numerical IK plugin ``ik_solver/PinocchioIkSolver`` for any URDF chain.
* The URDF is reduced to ``joint_names``. The other joints are locked at ``locked_joints/<joint>``
  (default 0). ``base_frame`` and ``flange_frame`` can be any frame of the URDF, and the joint order
  can differ from the URDF.
* IK is multi-start Levenberg-Marquardt on the SE(3) log error, clamped to the joint limits. It
  starts from the seeds, then from random configurations. It stops after ``desired_solutions``
  distinct solutions (modulo 2pi) or at the stall limits. Responses include the 2pi copies within
  the limits and the translation and rotation residuals.
* New parameters: ``translation_tolerance``, ``rotation_tolerance``, ``max_iterations_per_attempt``,
  ``damping``, ``max_step``, ``duplicate_tolerance``, ``max_attempts``, ``random_seed``,
  ``locked_joints``.
* ROS-free library ``pinocchio_kinematics`` (``PinocchioKinematics``), usable without ``ik_solver``.
* Unit tests on a UR10e URDF, generated at build time from ``ur_description``.
* Apache-2.0 license.
