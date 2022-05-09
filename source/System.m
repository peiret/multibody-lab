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
    force % generalized applied forces
    
    conPos % position-level contstraints
    conVel % velocity-level contstraints
    conJac % constraint jacobian
    conCoriolis
    conForce
    
    energyKin
    energyPot
    
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
        
        S.indJac    = zeros(S.nInd, S.nDep);
        
        S.mass      = zeros(S.nDep, S.nDep);    % mass matrix
        S.coriolis  = zeros(S.nDep, 1);         % velocity dependant terms
        S.force     = zeros(S.nDep, 1);         % generalized applied forces
        
        S.conPos  	= zeros(S.nCon, 1);         % position-level contstraints
        S.conVel  	= zeros(S.nCon, 1);         % velocity-level contstraints
        S.conJac  	= zeros(S.nCon, S.nDep);    % constraint jacobian
        S.conCoriolis = zeros(S.nCon, 1);       % constraint acceleration terms 
        S.conForce 	= zeros(S.nCon, 1);         % constraint forces (Lagrange multipliers)
        
        S.energyKin = 0;
        S.energyPot = 0;
        
    end
    
    function initSystem(S)
        %% Initialize Position and velocity to 
        
        S.updateSysPosition();
        S.initCoordinates();
        S.solvePositionProb();
        
        S.updateSysVelocity();
        S.initVelocity();
        S.solveVelocityProb();
        
    end
    
    function initCoordinates(S)
        %%
        for k = 1 : S.model.nCoordinates
            S.cooInd(k) = S.model.coordinateSet(k).initialPos;
        end
    end
    
    function initVelocity(S)
        %%
        for k = 1 : S.model.nCoordinates
            S.velInd(k) = S.model.coordinateSet(k).initialVel;
        end
    end
    
    function updateSysPosition(S)
        %% Update system position based on the model
        
        S.updateDependentCoordinates();
        S.updateIndependentCoordinates();
        S.updateConstraintPosition();
    end
    
    function updateDependentCoordinates(S)
        %% Update system dependent coordinates from model bodies
        for k = 1 : S.model.nBodies
            bodyIdx = 3*(k-1) + (1:3);
            S.cooDep(bodyIdx(1:2)) 	= S.model.bodySet(k).getCOMPosAbsolute();
            S.cooDep(bodyIdx(3))   	= S.model.bodySet(k).angle;
        end
    end
    
    function updateIndependentCoordinates(S)
        %% Update system independent coordinates from model coordinates
        for k = 1 : S.model.nCoordinates
            S.cooInd(k) = S.model.coordinateSet(k).getValue();
        end
    end
    
    function updateConstraintPosition(S)
        %% Update system constraint position from model joints
        idx = 0;
        for k = 1 : S.model.nJoints
            pos = S.model.jointSet(k).calcConstraintPos();
            len = length(pos);
            S.conPos(idx + (1:len)) = pos;
            idx = idx + len;
        end
    end
    
    function updateSysVelocity(S)
        %% Update system velocity based on the model
        S.updateDependentVelocity();
        S.updateIndependentVelocity();
        S.updateConstraintVelocity();
    end
    
    function updateDependentVelocity(S)
        %% Update system independent coordinates from model Bodies
        for k = 1 : S.model.nBodies
            bodyIdx = 3*(k-1) + (1:3);
            S.velDep(bodyIdx(1:2)) 	= S.model.bodySet(k).getCOMVelocity();
            S.velDep(bodyIdx(3))  	= S.model.bodySet(k).angVel;
        end
    end
    
    function updateIndependentVelocity(S)
        %% Update system dependent velocity from model coordinates
        for k = 1 : S.model.nCoordinates
            S.velInd(k) = S.model.coordinateSet(k).getVelocity();
        end
    end
    
    function updateConstraintVelocity(S)
        %% Update system constraint position from model joints
        idx = 0;
        for k = 1 : S.model.nJoints
            vel = S.model.jointSet(k).calcConstraintVel();
            len = length(vel);
            S.conVel(idx + (1:len)) = vel;
            idx = idx + len;
        end
    end
    
    function updateModel(S)
        %% Update model kinematics
        
        S.updateModelPosition();
        S.updateModelVelocity();
    end
    
    function updateModelPosition(S)
        %% Update Model Position using dependent coordinates
        
        for k = 1 : S.model.nBodies
            bodyIdx = 3*(k-1) + (1:3);
            S.model.bodySet(k).setOrientation(S.cooDep(bodyIdx(3)));
            S.model.bodySet(k).setCOMPosition(S.cooDep(bodyIdx(1:2)));
        end
        
    end
    
    function updateModelVelocity(S)
        %% Update Model Velocity using dependent coordinates
        
        for k = 1 : S.model.nBodies
            bodyIdx = 3*(k-1) + (1:3);
            S.model.bodySet(k).setAngularVelocity(S.velDep(bodyIdx(3)));
            S.model.bodySet(k).setCOMVelocity(S.velDep(bodyIdx(1:2)));
        end
    end
    
    function updateJacobians(S)
        %% Update system jacobian matrices
        
        S.calcConstraintJacobian();
        S.calcIndependentCoordJacobian();
        S.calcConstraintCoriolis();
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
    
    function calcConstraintCoriolis(S)
        
        S.conCoriolis = zeros(S.model.nConstraints, 1);
        idx = 0;
        
        for k = 1 : S.model.nJoints
            [blocks, bodies] = S.model.jointSet(k).getJacobianDerivativeBlocks();
            nBlocks = length(bodies);
            blkSize = size(blocks, 1);
            
            for j = 1 : nBlocks
                bodyIdx     = find(S.model.bodySet == bodies(j));
                rowIdx      = idx + (1:blkSize);
                colIdx      = 3*(bodyIdx - 1) + (1:3);
                
                S.conCoriolis(rowIdx) = S.conCoriolis(rowIdx) + blocks(:,:,j) * S.velDep(colIdx);
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
    
    function solvePositionProb(S, cooInd_0)
        %% Solve the position problem: calculate cooDep from cooInd
        
        if ~exist("cooInd_0", "var")
            cooInd_0 = S.cooInd;
        end
        
        q = S.cooDep;
        S.updateModel();
        
        epsilon = 1E-6;     % tolerance
        stepSize = 1;       % || q_k+1 - q_k ||
        
        while stepSize > epsilon
            
            S.updateJacobians();
            J = [S.conJac; S.indJac];
            
            S.updateSysPosition();
            beta = [S.conPos; S.cooInd - cooInd_0];

            q = q - (J \ beta);

            stepSize = norm(q - S.cooDep);

            S.cooDep = q;
            S.updateModelPosition();

            %viewer.update();

        end
    end
    
    function solveVelocityProb(S, velInd_0)
        %% Solve the velocity problem: calculate velDep from velInd
        
        if ~exist("velInd_0", "var")
            velInd_0 = S.velInd;
        end
        
        S.updateJacobians();
        vel         = [velInd_0; zeros(S.nCon,1)];
        Trans       = [S.indJac; S.conJac];
        S.velDep    = Trans \ vel;
        
    end
    
    function updateSysDynamics(S)
        %% Update system dynamic equations (mass matrix, forces, etc.)
        
        calcMassMatrix(S);
        calcGeneralizedForces(S);
        
    end
    
    function calcMassMatrix(S)
        %% Calculate Mass Matrix
        
        massDiag = zeros(S.nDep, 1);
        for k = 1 : S.model.nBodies
            body = S.model.bodySet(k);
            bodyIdx = 3*(k-1) + (1:3);
            massDiag(bodyIdx) = [body.mass; body.mass; body.inertia];
        end
        S.mass = diag(massDiag);
    end
    
    function calcGeneralizedForces(S)
        %%
        
        S.force = zeros(S.nDep, 1);
        for k = 1 : S.model.nBodies
            bodyIdx = 3*(k-1) + (1:3);
            S.force(bodyIdx) = [S.model.gravity; 0];
        end
    end
    
end % methods

end % classdef
