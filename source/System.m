classdef System < handle

properties
    
    model

    % Number of dependent and independent coordinates
    nDep
    nInd
    nCon
    
    cooDep
    velDep
    accDep

    cooInd
    velInd
    accInd
    
    indJac % Jacobian matrix of velocity transformation cooDep to cooInd

    mass % mass matrix
    coriolis % velocity dependant terms
    forces % generalized applied forces

    conPos % position-level contstraints
    conVel % velocity-level contstraints
    conJac % constraint jacobian

    time
    
end % properties

methods
    function S = System(model)
    %% System

    S.model     = model;

    % Number of dependent and independent coordinates
    S.nDep      = 3 * model.nBodies;
    S.nInd      = 3 * model.nBodies - model.nConstraints;
    S.nCon      = model.nConstraints;


    S.cooDep    = zeros(S.nDep, 1);
    S.velDep    = zeros(S.nDep, 1);
    S.accDep    = zeros(S.nDep, 1);

    S.cooInd    = zeros(S.nInd, 1);
    S.velInd    = zeros(S.nInd, 1);
    S.accInd    = zeros(S.nInd, 1);

    S.mass      = zeros(S.nDep, S.nDep);    % mass matrix
    S.coriolis  = zeros(S.nDep, 1);         % velocity dependant terms
    S.forces    = zeros(S.nDep, 1);         % generalized applied forces

    S.conPos  	= zeros(S.nCon, 1);         % position-level contstraints
    S.conVel  	= zeros(S.nCon, 1);         % velocity-level contstraints
    S.conJac  	= zeros(S.nCon, S.nDep);    % constraint jacobian

    % 
    S.time      = 0;
    
    end
    
    function updateModel(S)
        %% Update model kinematics
        
        for k = 1 : S.model.nBodies
            S.model.bodySet(k).position	= S.cooDep(3*(k-1) + (1:2));
            S.model.bodySet(k).velocity	= S.velDep(3*(k-1) + (1:2));
            S.model.bodySet(k).angle   	= S.cooDep(3*(k-1) + 3);
            S.model.bodySet(k).angVel 	= S.velDep(3*(k-1) + 3);
            % TO-DO: update acceleration?
        end
        
    end
    
    function initSystem(S)
        %%
        S.updateSysPosition();
        S.updateSysVelocity();
    end
    
    function updateSysPosition(S)
        %% Update system position based on the model
        
        for k = 1 : S.model.nBodies
            S.cooDep(3*(k-1) + (1:2))   = S.model.bodySet(k).position;
            S.cooDep(3*(k-1) + 3)       = S.model.bodySet(k).angle;
        end
        
        idx = 0;
        for k = 1 : S.model.nJoints
            pos = S.model.jointSet(k).calcConstraintPos();
            len = length(pos);
            S.conPos(idx + (1:len)) = pos;
            idx = idx + len;
        end
        
        for k = 1 : S.model.nCoordinates
            S.cooInd(k) = S.model.coordinateSet(k).getValue();
        end
        
    end
    
    function updateSysVelocity(S)
        %% Update system velocity based on the model
        
        for k = 1 : S.model.nBodies
            S.velDep(3*(k-1) + (1:2))   = S.model.bodySet(k).velocity;
            S.velDep(3*(k-1) + 3)       = S.model.bodySet(k).angVel;
        end
        
        for k = 1 : S.model.nCoordinates
            S.cooInd(k) = S.model.coordinateSet(k).getVelocity();
        end
        
    end
    
    function updateJacobians(S)
        %% Update system jacobian matrices
        
        S.calcConstraintJacobian();
        S.calcIndependentCoordJacobian();
        
    end
    
    function calcConstraintJacobian(S)
        %% Calculate Constraint Velocity
        
        S.conJac = zeros(S.model.nConstraints, S.model.nBodies);
        idx = 0;
        
        for k = 1 : S.model.nJoints
            [blocks, bodies] = S.model.jointSet(k).getJacobianBlocks();
            nBlocks = length(bodies);
            blkSize = size(blocks, 1);
            
            for j = 1 : nBlocks
                bodyIdx     = find(S.model.bodySet == bodies(j));
                rowIdx      = idx + (1:blkSize);
                colIdx      = 3*(bodyIdx - 1) + (1:3);
                S.conJac(rowIdx, colIdx) = blocks(:,:,j);
            end
            
            idx = idx + blkSize;
        end
    end
    
    function calcIndependentCoordJacobian(S)
        %% TO-DO
        
        S.indJac = zeros(S.nInd, S.nDep);
        
        for k = 1 : S.model.nCoordinates
            [blocks, bodies] = S.model.coordinateSet(k).getJacobianBlocks();
            nBlocks = length(bodies);
            
            for j = 1 : nBlocks
                bodyIdx     = find(S.model.bodySet == bodies(j));
                colIdx      = 3*(bodyIdx - 1) + (1:3);
                S.indJac(k, colIdx) = blocks(:,:,j);
            end
        end
        
    end
    
    function updateSysDynamics(S)
        %% Update system dynamic equations (mass matrix, forces, etc.)
        
        % TO-DO
        
    end
    
    
    
end % methods

end % classdef
