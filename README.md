# Multibody Lab
Library to model and simulate __multibody systems__ in MATLAB.



## Content

- `/examples`: set of scripts with different models and simulations.
- `/models`: set of basic multibody models (pendulum, double pendulum, etc.).
- `/scripts`: set of MATLAB Live Scripts with exercises for lab sessions.
- `/source`: class definitions needed for this library to work.



## Documentation

This library provides a set of classes to model and simulate multibody systems. The most important ones are:
- `Model`: contains the model information such as, the bodies, joints and coordinate definitions.
- `Viewer`: generates the model graphics.
- `System`: takes care of formulating and solving the dynamic equations of the model.
- `Simulation`: integrates the dynamics equations over time.



### Model

A Model contains the following sets:
- `Model.bodySet` is an array of type `Body`
- `Model.jointSet` is an array of type `Joint`
- `Model.coordinateSet` is an array of type `Coordinate`

Create a `Model`:
```
model = Model();
model.name = "my_first_model";
```

Create a new `Body` with some properties:
```
body = Body();
body.name       = "body_1";     % name of the body
body.mass       = 1;            % mass in kg
body.com        = [0; 0];       % [x; y] center of mass (COM)
body.inertia    = 1;            % inertia about the COM
```

Add the `Body` to the `Model`:
```
model.addBody(body)
```

The `Model` has a pre-defined fixed `Body` named `ground`. This allows us to define a `Joint` between a body and the ground (every joint has `parent` and `child` bodies).

`JointRevolute` is a revolute joint that constrains the position of `pointChild` to `pointParent`. Create a `JointRevolute` and define its properties:
```
joint = JointRevolute();
joint.parent        = model.ground;
joint.child         = body;
joint.pointParent   = [0; 0];       % [x; y] in parent frame
joint.pointChild    = [0; 1];       % [x; y] in child frame
```

Add the `Joint` to the `Model`:
```
model.addJoint(joint);
```

Finally, it is necessary to define the independent coordinates of the model. In this case, we can use the orientation of the `Body`
```
coord = Coordinate();
coord.type          = "angular";     % angular or cartesian
coord.body          = body;
coord.reference     = model.ground;  % this is set by default
```
Add the `Coordinate` to the `Model`:
```
model.addCoordinate(coord);
```

Once all the elements of the `Model` have been defined, it is necessary to initialize the `Model`. This must be done after adding of removing any model components to the `bodySet`, `jointSet`, or `coordinateSet`.
```
model.initModel();
```



### Pendulum Example
The simulation of a pendulum model:

- First, create a `Model` using the predefined `Pendulum` in the `/models` folder and initialize the viewer to display the model geometry in a MATLAB figure window:
```
model = Pendulum();
model.initViewer();
```

- Then, create a `Simulation` with the `Model` that was just created, set the simulation end time, the time-step size:
```
sim = Simulation(model);
sim.timeEnd = 2;
sim.timeStep = 0.01;
```

- Finally, after setting the simulation parameters, the simulation must be initialized before it runs
```
sim.initialize();
sim.run();
```
