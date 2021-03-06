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
	jointSet        Joint
    coordinateSet   Coordinate
    
    ground          Body
	gravity         (2,1) double
    
    
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
        
        M.ground            = Body();
        M.ground.name       = "ground";
        M.ground.fixed      = true;
        
        M.gravity           = [0; -9.81];
    end

    function initializeModel(M)
        %% Must be called after adding or removing model components
        % calculates number of bodies, joints, coordinates
        
        M.nBodies = length(M.bodySet);
        M.nJoints = length(M.jointSet);
        M.nCoordinates = length(M.coordinateSet);
        
        % Calculate number of constraint equations
        M.nConstraints = 0;
        for k = 1 : M.nJoints
            if strcmp(M.jointSet(k).type, "revolute")
                M.nConstraints = M.nConstraints + 2;
            else
                error("Joint %d type not supported", k);
            end
        end
        
        if (3 * M.nBodies > M.nCoordinates + M.nConstraints)
            error("Not enough coordinates in the model");
            
        elseif (3 * M.nBodies < M.nCoordinates + M.nConstraints)
            error("System constraints and coordinates are redundant");
        end

    end
    
    function system = initSystem(M)
        %%
        
        system = System(M);
        system.initSystem();
        
        M.system = system;
    end
    
    function viewer = initViewer(M, xLim, yLim)
        %%
        
        viewer = Viewer(M);
        viewer.initViewer();
        
        if exist("xLim", "var")
            viewer.axes.XLim = xLim;
        end
        
        if exist("yLim", "var")
            viewer.axes.YLim = yLim;
        end
        
        M.viewer = viewer;
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
