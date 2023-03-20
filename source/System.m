classdef System < handle

properties
    
    model       Model
    
    % Number of dependent and independent coordinates
    nDep        (:,1) double
    nInd        (:,1) double
    nCon        (:,1) double
    
    % Dependent coordinates, velocities and accelerations
    depenCoord      (:,1) double
    depenVeloc      (:,1) double
    depenAccel      (:,1) double
    
    % Independent coordinates, velocities and accelerations
    indepCoord      (:,1) double
    indepVeloc      (:,1) double
    indepAccel      (:,1) double
    
    indJac      (:,:) double % Jacobian matrix of independent coordinates
    
    mass        (:,:) double % mass matrix
    coriolis    (:,1) double % velocity dependant terms
    force       (:,1) double % generalized applied forces
    
    constCoord      (:,1) double % position-level contstraints
    constVeloc      (:,1) double % velocity-level contstraints
    constJacob      (:,:) double % constraint jacobian
    
    constCoriolis (:,1) double 
    constForce    (:,1) double 
    
    energyKin   double
    energyPot   double
    
end % properties

methods
    function S = System(model)
    %% System
        
        S.model     = model;
        model.system = S;
        
        % Number of dependent and independent coordinates
        S.nDep      = 3 * model.nBodies;
        S.nInd      = 3 * model.nBodies - model.nConstraints;
        S.nCon      = model.nConstraints;
        
        S.depenCoord    = zeros(S.nDep, 1);
        S.depenVeloc    = zeros(S.nDep, 1);
        S.depenAccel    = zeros(S.nDep, 1);
        
        S.indepCoord    = zeros(S.nInd, 1);
        S.indepVeloc    = zeros(S.nInd, 1);
        S.indepAccel    = zeros(S.nInd, 1);
        
        S.indJac    = zeros(S.nInd, S.nDep);
        
        S.mass      = zeros(S.nDep, S.nDep);    % mass matrix
        S.coriolis  = zeros(S.nDep, 1);         % velocity dependant terms
        S.force     = zeros(S.nDep, 1);         % generalized applied forces
        
        S.constCoord  	= zeros(S.nCon, 1);         % position-level contstraints
        S.constVeloc  	= zeros(S.nCon, 1);         % velocity-level contstraints
        S.constJacob  	= zeros(S.nCon, S.nDep);    % constraint jacobian
        S.constCoriolis = zeros(S.nCon, 1);       % constraint acceleration terms 
        S.constForce 	= zeros(S.nCon, 1);         % constraint forces (Lagrange multipliers)
        
        S.energyKin = 0;
        S.energyPot = 0;
        
    end
    
    function initSystem(S)
        %% Initialize Position and velocity to 
        % Update system position and solve position problem 
        S.updateSysPosition();
        S.initCoordinates();
        S.solvePositionProb();
        % Update system velocity and solve velocity problem 
        S.updateSysVelocity();
        S.initVelocity();
        S.solveVelocityProb();
    end
    
    function update(S)
        %% Update System (from Model)
        % Must be called before solving the system dynamics
        S.updateSysPosition();
        S.updateSysVelocity();
        S.updateJacobians();
        S.updateDynamics();
    end
    
    function updateModel(S)
        %% Update model kinematics
        S.updateModelPosition();
        S.updateModelVelocity();
    end
    
    function initCoordinates(S)
        %%
        for k = 1 : S.model.nCoordinates
            S.indepCoord(k) = S.model.coordinateSet(k).initValue;
        end
    end
    
    function initVelocity(S)
        %%
        for k = 1 : S.model.nCoordinates
            S.indepVeloc(k) = S.model.coordinateSet(k).initSpeed;
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
            S.depenCoord(bodyIdx(1:2)) 	= S.model.bodySet(k).getCOMPosAbsolute();
            S.depenCoord(bodyIdx(3))   	= S.model.bodySet(k).angle;
        end
    end
    
    function updateIndependentCoordinates(S)
        %% Update system independent coordinates from model coordinates
        for k = 1 : S.model.nCoordinates
            S.indepCoord(k) = S.model.coordinateSet(k).getValue();
        end
    end
    
    function updateConstraintPosition(S)
        %% Update system constraint position from model joints
        idx = 0;
        for k = 1 : S.model.nJoints
            pos = S.model.jointSet(k).calcConstraintPos();
            len = length(pos);
            S.constCoord(idx + (1:len)) = pos;
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
            S.depenVeloc(bodyIdx(1:2)) 	= S.model.bodySet(k).getCOMVelocity();
            S.depenVeloc(bodyIdx(3))  	= S.model.bodySet(k).angVel;
        end
    end
    
    function updateIndependentVelocity(S)
        %% Update system dependent velocity from model coordinates
        for k = 1 : S.model.nCoordinates
            S.indepVeloc(k) = S.model.coordinateSet(k).getVelocity();
        end
    end
    
    function updateConstraintVelocity(S)
        %% Update system constraint position from model joints
        idx = 0;
        for k = 1 : S.model.nJoints
            vel = S.model.jointSet(k).calcConstraintVel();
            len = length(vel);
            S.constVeloc(idx + (1:len)) = vel;
            idx = idx + len;
        end
    end
    
    function updateModelPosition(S)
        %% Update Model Position using dependent coordinates
        for k = 1 : S.model.nBodies
            bodyIdx = 3*(k-1) + (1:3);
            S.model.bodySet(k).setOrientation(S.depenCoord(bodyIdx(3)));
            S.model.bodySet(k).setCOMPosition(S.depenCoord(bodyIdx(1:2)));
        end
        
    end
    
    function updateModelVelocity(S)
        %% Update Model Velocity using dependent coordinates
        for k = 1 : S.model.nBodies
            bodyIdx = 3*(k-1) + (1:3);
            S.model.bodySet(k).setAngularVelocity(S.depenVeloc(bodyIdx(3)));
            S.model.bodySet(k).setCOMVelocity(S.depenVeloc(bodyIdx(1:2)));
        end
    end
    
    function updateJacobians(S)
        %% Update system jacobian matrices
        S.calcConstraintJacobian();
        S.calcIndependentCoordJacobian();
    end
    
    function calcConstraintJacobian(S)
        %% Calculate Constraint Velocity
        
        S.constJacob = zeros(S.model.nConstraints, S.model.nBodies);
        idx = 0;
        
        for k = 1 : S.model.nJoints
            [blocks, bodies] = S.model.jointSet(k).getJacobianBlocks();
            nBlocks = length(bodies);
            blkSize = size(blocks, 1);
            
            for j = 1 : nBlocks
                bodyIdx     = find(S.model.bodySet == bodies(j));
                rowIdx      = idx + (1:blkSize);
                colIdx      = 3*(bodyIdx - 1) + (1:3);
                S.constJacob(rowIdx, colIdx) = blocks(:,:,j);
            end
            
            idx = idx + blkSize;
        end
    end
    
    function calcConstraintCoriolis(S)
        
        S.constCoriolis = zeros(S.model.nConstraints, 1);
        idx = 0;
        
        for k = 1 : S.model.nJoints
            [blocks, bodies] = S.model.jointSet(k).getJacobianDerivativeBlocks();
            nBlocks = length(bodies);
            blkSize = size(blocks, 1);
            
            for j = 1 : nBlocks
                bodyIdx     = find(S.model.bodySet == bodies(j));
                rowIdx      = idx + (1:blkSize);
                colIdx      = 3*(bodyIdx - 1) + (1:3);
                
                S.constCoriolis(rowIdx) = S.constCoriolis(rowIdx) + blocks(:,:,j) * S.depenVeloc(colIdx);
            end
            
            idx = idx + blkSize;
        end
    end
    
    function calcIndependentCoordJacobian(S)
        %% Calculate Independent Coordinate Jacobian Matrix
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
    
    function solvePositionProb(S, indepCoord_0)
        %% Solve the position problem: calculate depenCoord from indepCoord
        
        if ~exist("indepCoord_0", "var")
            indepCoord_0 = S.indepCoord;
        end
        
        q = S.depenCoord;
        S.updateModel();
        
        epsilon = 1E-6;     % tolerance
        stepSize = 1;       % || q_k+1 - q_k ||
        
        while stepSize > epsilon
            
            S.updateJacobians();
            J = [S.constJacob; S.indJac];
            
            S.updateSysPosition();
            beta = [S.constCoord; S.indepCoord - indepCoord_0];

            q = q - (J \ beta);

            stepSize = norm(q - S.depenCoord);

            S.depenCoord = q;
            S.updateModelPosition();
        end
    end
    
    function solveVelocityProb(S, indepVeloc_0)
        %% Solve the velocity problem: calculate depenVeloc from indepVeloc
        
        if ~exist("indepVeloc_0", "var")
            indepVeloc_0 = S.indepVeloc;
        end
        
        S.updateJacobians();
        vel         = [indepVeloc_0; zeros(S.nCon,1)];
        Trans       = [S.indJac; S.constJacob];
        S.depenVeloc    = Trans \ vel;
        S.updateModelVelocity();
    end
    
    function updateDynamics(S)
        %% Update system dynamic equations (mass matrix, forces, etc.)
        calcMassMatrix(S);
        calcGeneralizedForces(S);
        S.calcConstraintCoriolis();
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
    
    function solveDynamics(S)
        %% Solving the dynamic equations
        % [ M -A'] [ depenAccel   ] = [ force - coriolis ]
        % [ A  0 ] [ constForce ] = [ -constCoriolis     ]
        
        auxMat = S.constJacob * (S.mass \ S.constJacob');
        auxVec = S.constJacob * (S.mass \ (S.force - S.coriolis));
        S.constForce = - auxMat \ (S.constCoriolis + auxVec);
        S.depenAccel = S.mass \ (S.constJacob' * S.constForce + S.force - S.coriolis);
    end
    
end % methods

end % classdef
