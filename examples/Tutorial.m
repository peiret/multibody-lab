clearvars
close all

%% Create Pendulum model

% Model Parameters
length = 1;
mass = 1;

% Create model
model = Model();
model.name = "pendulum";

%% Create pendulum body
body = Body();
body.name               = "pendulum";
body.mass               = 1;
body.com                = [length/2; 0]; % center of mass
body.inertia            = mass * length / 12;

% Define Body geometry
body.geometry.lineColor = 'r';
body.geometry.lineWidth = 4;
body.geometry.addPoint([0; 0]);
body.geometry.addPoint([length; 0]);

% Add body to the model
model.addBody(body);

%% Create revolute Joint with the ground
joint = JointRevolute();
joint.parent        = model.ground;
joint.child         = body;
joint.pointParent   = [0; 0];
joint.pointChild    = body.geometry.points(:,1);

% Add joint to model
model.addJoint(joint);

%% Angle between pendulum and the ground
coord = Coordinate();
coord.type          = "angular";
coord.body          = body;
coord.initValue     = 0;
coord.initSpeed     = 0;

% Add coordinate to model
model.addCoordinate(coord);

%% Initialize model and viewer
model.initModel();
model.initViewer();

%% Simulate pendulum

% Setup simulation parameters
sim = Simulation(model);
sim.timeEnd     = 5;
sim.timeStep    = 0.01;

% Initialize and run simulation
sim.initialize();
sim.run();
