classdef Model < handle
%%
properties
    %%

    name            = "";
	nBodies         = 0;
	nJoints         = 0;
    nConstraints    = 0;
    nCoordinates    = 0;

	gravity         = [0, 9.81];

    bodySet         = Body();
	jointSet        = Joint();
    coordinateSet   = Coordinate();
    
    ground

end

methods
    %%
    
    function M = Model()
        %% This is the constructor of the class

        M.name = "model";

        M.nBodies = 0;
        M.nJoints = 0;
        M.gravity = [0; -9.81];
        
        M.ground = Body();
        M.ground.name = "ground";
        M.ground.fixed = true;
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
