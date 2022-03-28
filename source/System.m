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
        
        S.mass      = zeros(S.nDep, S.nDep);    % mass matrix
        S.coriolis  = zeros(S.nDep, 1);         % velocity dependant terms
        S.force     = zeros(S.nDep, 1);         % generalized applied forces
        
        S.conPos  	= zeros(S.nCon, 1);         % position-level contstraints
        S.conVel  	= zeros(S.nCon, 1);         % velocity-level contstraints
        S.conJac  	= zeros(S.nCon, S.nDep);    % constraint jacobian
        
        S.energyKin = 0;
        S.energyPot = 0;
        
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
        %% Initialize Position and velocity to 
        
        S.updateSysPosition();
        S.initCoordinates();
        S.solvePositionProb();
        
        S.updateSysVelocity();
        S.initVelocity();
        
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
            S.velInd(k) = S.model.coordinateSet(k).getVelocity();
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
    
    function solvePositionProb(S, cooInd_0)
        %% Solve the position problem
        
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
            S.updateModel();

            %viewer.update();

        end
    end
    
    function updateSysDynamics(S)
        %% Update system dynamic equations (mass matrix, forces, etc.)
        
        m1 = S.model.bodySet(1).mass;
        m2 = S.model.bodySet(2).mass;
        I1 = S.model.bodySet(1).inertia;
        I2 = S.model.bodySet(2).inertia;
        L1 = norm(S.model.bodySet(1).geometry.points(:,1) - S.model.bodySet(1).geometry.points(:,2));
        L2 = norm(S.model.bodySet(2).geometry.points(:,1) - S.model.bodySet(2).geometry.points(:,2));
        
        th1 = S.cooInd(1);
        th2 = S.cooInd(2);
        dth1 = S.velInd(1);
        dth2 = S.velInd(2);
        
        g = - S.model.gravity(2);
        
        S.mass = [...
            I1 + (L1^2*m1)/4 + L1^2*m2,     (L1*L2*m2*cos(th1 - th2))/2;
            (L1*L2*m2*cos(th1 - th2))/2,   	(m2*L2^2)/4 + I2];
        
        S.coriolis = [...
            (L1*L2*dth2^2*m2*sin(th1 - th2))/2;
            -(L1*L2*dth1^2*m2*sin(th1 - th2))/2];
        
        S.force = [...
            - L1*g*m1*sin(th1) - L1*g*m2*sin(th1);
            -L2*g*m2*sin(th2)];
        
        
        %% TO-DO: calulate system energy
        % energyKin = 
        % energyPot = 
        
    end
    
    
    
end % methods

end % classdef
