classdef Model < handle
%%
properties
    %%
    
    system          System
    viewer          Viewer

    name            string
	nBodies         double
	nJoints         double
    nConstraints    double
    nCoordinates    double

    bodySet         Body
	jointSet        Joint = JointRevolute
    coordinateSet   Coordinate
    
    ground          Body
	gravity         (2,1) double
    
    isInitialized   logical
    
end

methods
    %%
    
    function M = Model()
        %% This is the constructor of the class

        M.name = "model";

        M.nBodies           = 0;
        M.nJoints           = 0;
        M.nCoordinates      = 0;
        M.nConstraints      = 0;
        
        M.jointSet = M.jointSet([]);
        
        M.ground            = Body();
        M.ground.name       = "ground";
        M.ground.fixed      = true;
        
        M.gravity           = [0; -9.81];
    end
    
    function addBody(M, body)
        %% Add a body to the model
        M.bodySet(end + 1) = body;
        M.isInitialized = false;
    end
    
    function addJoint(M, joint)
        %% Add a joint to the model
        M.jointSet(end + 1) = joint;
        M.isInitialized = false;
    end
    
    function addCoordinate(M, coordinate)
        %% Add a coordinate to the model
        M.coordinateSet(end + 1) = coordinate;
        M.isInitialized = false;
    end
    
    function initModel(M)
        %% Must be called after adding or removing model components
        % calculates number of bodies, joints, coordinates
        
        for body = M.bodySet
            body.initBody();
        end
        
        for coord = M.coordinateSet
            coord.initCoordinate(M);
        end
        
        M.nBodies = length(M.bodySet);
        M.nJoints = length(M.jointSet);
        M.nCoordinates = length(M.coordinateSet);
        
        % Calculate number of constraint equations
        M.nConstraints = 0;
        for joint = M.jointSet
            joint.initJoint();
            M.nConstraints = M.nConstraints + joint.nConstraints;
        end
        
        if (3 * M.nBodies > M.nCoordinates + M.nConstraints)
            error("Not enough coordinates in the model");
            
        elseif (3 * M.nBodies < M.nCoordinates + M.nConstraints)
            error("System constraints and coordinates are redundant");
        end
        
        if ~isempty(M.system)
            M.system.initSystem();
        end
        if ~isempty(M.viewer)
            M.viewer.update();
        end
        M.isInitialized = true;
    end
    
    function system = initSystem(M)
        %% Initialize System from a model
        if isempty(M.system)
             M.system = System(M);
        end
        M.system.initSystem();
        M.system.updateModel();
        if ~isempty(M.viewer)
             M.viewer.update();
        end
        
        system = M.system;
    end
    
    function viewer = initViewer(M, xLim, yLim)
        %%
        if ~exist('xLim', 'var'), xLim = [-1,1]; end
        if ~exist('yLim', 'var'), yLim = [-1,1]; end
        
        if isempty(M.viewer)
             M.viewer = Viewer(M);
        end
        M.viewer.initViewer(xLim, yLim);
        viewer = M.viewer;
    end
    
    function update(M)
        %% Update model position and velocity
        if isempty(M.system)
            M.initSystem()
        end
        M.system.updateModel();
    end

    function printReport(M)
        %% Model Report:
        %   Outputs on the command window the model basic information

        fprintf(" Model: %s\n", M.name);
        fprintf("    Bodies = %i\n", M.nBodies);
        fprintf("    Joints = %i\n", M.nJoints);
        fprintf("    Dofs   = %i\n", M.nCoordinates);
    end

end

end
