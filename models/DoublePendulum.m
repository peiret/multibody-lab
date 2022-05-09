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
j1 = Joint();
j1.type                     = "revolute";
j1.parent                   = model.ground;
j1.child                    = b1;
j1.pointParent              = [0; 0];
j1.pointChild               = b1.geometry.points(:,1);

% Joint between the first and second link
j2 = Joint();
j2.type                     = "revolute";
j2.parent                   = b1;
j2.child                    = b2;
j2.pointParent              = b1.geometry.points(:,2);
j2.pointChild               = b2.geometry.points(:,1);

% Angle between the first link and the ground
c1 = Coordinate();
c1.type                     = "angular";
c1.component                = "Body";
c1.body                     = b1;
c1.initialPos               = pi;
c1.initialVel               = 0;

% Angle between the first and second link
c2 = Coordinate();
c2.type                     = "angular";
c2.component                = "Body";
c2.body                     = b2;
c2.initialPos               = pi/2;
c2.initialVel               = 0;

% Add components to the model
model.bodySet(1)            = b1;
model.bodySet(2)            = b2;
model.jointSet(1)           = j1;
model.jointSet(2)           = j2;
model.coordinateSet(1)      = c1;
model.coordinateSet(2)      = c2;

% Model must be initialized after adding components
model.initializeModel();

end

