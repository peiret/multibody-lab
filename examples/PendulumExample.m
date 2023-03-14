
clearvars
close all % windows

%currentFileDir = string(fileparts(matlab.desktop.editor.getActiveFilename));
addpath("../models")
addpath("../source")

% Create Pendulum model

% Model Parameters
length = 1;
mass = 1;

% Create model
model = Model();
model.name = "pendulum";

% First link of the pendulum
body = Body();
body.name                  	= "pendulum";
body.mass                  	= 1;
body.com                   	= [length/2; 0]; % center of mass
body.inertia              	= mass * length / 12;
body.geometry.points(:,1)  	= [0; 0];
body.geometry.points(:,2)  	= [length; 0];
body.geometry.lineColor    	= 'b';
body.geometry.lineWidth    	= 2;

% Add body to the model
model.addBody(body);

% Joint between the first link and the ground
joint = Joint();
joint.type              	= "revolute";
joint.parent              	= model.ground;
joint.child             	= body;
joint.pointParent         	= [0; 0];
joint.pointChild           	= body.geometry.points(:,1);

% Add joint to model
model.addJoint(joint);

% Angle between the first link and the ground
coord = Coordinate();
coord.type               	= "angular";
coord.component         	= "Body";
coord.body                	= body;
coord.initialPos        	= 0;
coord.initialVel          	= 0;

% Add coordinate to model
model.addCoordinate(coord);

% Initialize model and viewer
model.initModel();
model.initViewer([-1, +1], [-1, +1]);

%% Setup simulation parameters
sim = Simulation(model);
sim.integrator  = "EulerImplicitCorrected";
sim.timeEnd     = 2;
sim.timeStep    = 0.01;

% Initialize and run simulation
sim.initialize();
sim.run();



