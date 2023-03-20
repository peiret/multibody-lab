clearvars
close all

% Create double pendulum model
model = DoublePendulum();
model.initViewer([-2,2],[-2,1]);

% Create simulation
sim = Simulation(model);

sim.integrator  = "EulerImplicit";
sim.timeEnd     = 10;
sim.timeStep    = 0.01;
sim.correctPosition = true;

% Initialize and run simulation
sim.initialize();
sim.run();