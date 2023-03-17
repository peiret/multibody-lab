function model = Pendulum(mass, length, angle, speed)
% PENDULUM MODEL

% Model Parameters
if ~exist('mass', 'var')
    mass = 1;
end
if ~exist('length', 'var')
    length = 1;
end
if ~exist('angle', 'var')
    angle = pi/2;
end
if ~exist('speed', 'var')
    speed = 0;
end

% Create model
model = Model();
model.name = "pendulum";

% First link of the pendulum
body = Body();
body.name                  	= "pendulum";
body.mass                  	= 1;
body.com                   	= [0; -length/2]; % center of mass
body.inertia              	= mass * length^2 / 12;
body.geometry.points(:,1)  	= [0; 0];
body.geometry.points(:,2)  	= [0; -length];
body.geometry.lineColor    	= 'b';
body.geometry.lineWidth    	= 4;

% Add body to the model
model.addBody(body);

% Joint between the first link and the ground
joint = JointRevolute();
joint.parent              	= model.ground;
joint.child             	= body;
joint.pointParent         	= [0; 0];
joint.pointChild           	= body.geometry.points(:,1);

% Add joint to model
model.addJoint(joint);

% Angle between the first link and the ground
coord = Coordinate();
coord.type               	= "angular";
coord.body                	= body;
coord.initValue             = angle;
coord.initSpeed             = speed;

% Add coordinate to model
model.addCoordinate(coord);

% Initialize model and viewer
model.initModel();
end

