# @konitif/physics-mujoco

Optional MuJoCo worker provider implementing @konitif/physics. Physics remains the contract authority; this package supplies a specific execution adapter, fixed-step scheduling and headless experiment sessions.

The host supplies a compatible MuJoCo module URL, model data and simulation commands. No binary is downloaded or bundled. Importing creates no worker or clock. createMujocoBackend starts the worker explicitly; workerFactory allows host integration. Compositions own histories and the temporal cursor; renderers consume observations.

The distributed worker is a JavaScript ES module. No NAO resources, hardware discovery, robot connection or renderer are included.
