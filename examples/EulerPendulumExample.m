
clearvars
close all % windows
addpath("../source")

% Create Euler's Pendulum model

% Model Parameters
length = 1;
mass = 1;

% Create model
model = Model();
model.name = "Euler's pendulum";

% First link of the pendulum
body = Body();
body.name                  	= "pendulum";
body.mass                  	= 1;
body.com                   	= [length/2; 0]; % center of mass
body.inertia              	= mass * length / 12;
body.geometry.points(:,1)  	= [0; 0];
body.geometry.points(:,2)  	= [length; 0];
body.geometry.lineColor    	= 'r';
body.geometry.lineWidth    	= 4;

% Add body to the model
model.addBody(body);

% Joint between the first link and the ground
joint = JointPinSlot();
joint.parent              	= model.ground;
joint.child             	= body;
joint.pointParent         	= [0; 0];
joint.pointChild           	= body.geometry.points(:,1);
joint.directionParent       = [1; 0];

% Add joint to model
model.addJoint(joint);

% Angle between the first link and the ground
pos = Coordinate();
pos.type                    = "cartesian";
pos.body                    = body;
pos.refPoint                = body.geometry.points(:,1);
pos.refDirection            = [1; 0];
pos.initValue               = 0;
pos.initSpeed               = 0;

% Angle between the first link and the ground
rot = Coordinate();
rot.type                    = "angular";
rot.body                    = body;
rot.initValue               = 0;
rot.initSpeed               = 0;

% Add coordinate to model
model.addCoordinate(pos);
model.addCoordinate(rot);

% Initialize model and viewer
model.initModel();
model.initViewer([-1, +1], [-1, +1]);
model.viewer.showAxes(false);

%% Setup simulation parameters
sim = Simulation(model);
sim.integrator  = "EulerImplicit";
sim.timeEnd     = 2;
sim.timeStep    = 0.005;
sim.correctPosition = true;

% Initialize and run simulation
sim.initialize();
sim.run();



