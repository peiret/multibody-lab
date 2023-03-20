clearvars
close all % windows

% Create Pendulum model
model = Pendulum();
model.initViewer();

%% Setup simulation parameters
sim = Simulation(model);
sim.timeEnd = 2;
sim.initialize();
sim.run();

