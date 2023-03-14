clearvars
close all % windows

%currentFileDir = string(fileparts(matlab.desktop.editor.getActiveFilename));
addpath("../models")
addpath("../source")

% Create double pendulum model
model = DoublePendulum();
model.initViewer([-2,2],[-2,1]);

% Create simulation
sim = Simulation(model);

sim.integrator  = "EulerImplicitCorrected";
sim.timeEnd     = 4;
sim.timeStep    = 0.01;

% Initialize and run simulation
sim.initialize();
sim.run();