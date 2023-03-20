function model = SliderCrank()
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

mass = [0.5, 1];
length = [0.5, 1];
angle = 0;
speed = 0;

% Create model
model = Model();
model.name = "double_pendulum";

% First link of the pendulum
b1 = Body();
b1.name                     = "body_1";
b1.mass                     = mass(1);
b1.com                      = [0; 0]; % center of mass
b1.inertia                  = mass(1) * length(1)^2 / 12;
b1.geometry.points(:,1)     = [0; +length(1) / 2];
b1.geometry.points(:,2)     = [0; -length(1) / 2];
b1.geometry.lineColor       = 'r';
b1.geometry.lineWidth       = 4;

% Second link of the pendulum
b2 = Body();
b2.name                     = "body_2";
b2.mass                     = mass(2);
b2.com                      = [0; 0]; % center of mass
b2.inertia                  = mass(2) * length(2)^2 / 12;
b2.geometry.points(:,1)     = [0; +length(2) / 2];
b2.geometry.points(:,2)     = [0; -length(2) / 2];
b2.geometry.lineColor       = 'b';
b2.geometry.lineWidth       = 4;

% Joint between the first link and the ground
j1 = JointRevolute();
j1.parent                   = model.ground;
j1.child                    = b1;
j1.pointParent              = [0; 0];
j1.pointChild               = b1.geometry.points(:,1);

% Joint between the first and second link
j2 = JointRevolute();
j2.parent                   = b1;
j2.child                    = b2;
j2.pointParent              = b1.geometry.points(:,2);
j2.pointChild               = b2.geometry.points(:,1);

% Joint between the first and second link
j3 = JointPinSlot();
j3.parent                   = model.ground;
j3.child                    = b2;
j3.pointParent              = [0; 0];
j3.directionParent          = [0; 1];
j3.pointChild               = b2.geometry.points(:,2);

% Angle between the first link and the ground
c1 = Coordinate();
c1.type                     = "angular";
c1.body                     = b1;
c1.initValue                = angle;
c1.initSpeed                = speed;

% Add components to the model
model.addBody(b1);
model.addBody(b2);
model.addJoint(j1);
model.addJoint(j2);
model.addJoint(j3);
model.addCoordinate(c1);

% Model must be initialized after adding components
model.initModel();

end

