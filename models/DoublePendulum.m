function model = DoublePendulum()
% DOUBLE PENDULUM

% Model Parameters
length1 = 1;
length2 = 1;

% Create model
model = Model();
model.name = "double_pendulum";

% First link of the pendulum
b1 = Body();
b1.name                     = "body_1";
b1.mass                     = 1;
b1.com                      = [0; 0]; % center of mass
b1.inertia                  =  1;
b1.geometry.points(:,1)     = [0; +length1 / 2];
b1.geometry.points(:,2)     = [0; -length1 / 2];
b1.geometry.lineColor       = 'r';
b1.geometry.lineWidth       = 4;

% Second link of the pendulum
b2 = Body();
b2.name                     = "body_2";
b2.mass                     = 1;
b2.com                      = [0; 0]; % center of mass
b2.inertia                  = 1;
b2.geometry.points(:,1)     = [0; +length2 / 2];
b2.geometry.points(:,2)     = [0; -length2 / 2];
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

% Angle between the first link and the ground
c1 = Coordinate();
c1.type                     = "angular";
c1.body                     = b1;
c1.initValue                = pi;
c1.initSpeed                = 0;

% Angle between the first and second link
c2 = Coordinate();
c2.type                     = "angular";
c2.body                     = b2;
c2.initValue                = pi/2;
c2.initSpeed                = 0;

% Add components to the model
model.addBody(b1);
model.addBody(b2);
model.addJoint(j1);
model.addJoint(j2);
model.addCoordinate(c1);
model.addCoordinate(c2);

% Model must be initialized after adding components
model.initModel();

end

