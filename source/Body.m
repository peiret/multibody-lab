classdef Body < handle
properties
    
    name            string
    fixed           logical
    
    mass            double
    inertia         double
    com             (2,1) double

    % Appearance Properties
    geometry        Geometry

    % Kinematics
    position        (2,1) double
    velocity        (2,1) double
    
    angle           double
    angVel          double
    
    rotMatrix       (2,2) double
end

methods
    function B = Body()
        %% Body initialization
        
        % If the body is fixed to the ground
        B.fixed = false;
        
        % Mechanical Properties:
        B.mass = 1;
        B.inertia = 1; % about the center of mass
        B.com = [0; 0]; % relative to body frame
        
        % Appearance Properties
        B.geometry = Geometry(B);
        
        % Origin Kinematics
        B.position = [0; 0]; % position of the origin
        B.velocity = [0; 0]; % velocity of the origin
        
        % Orientation
        B.angle    = 0; % rotation
        B.angVel   = 0; % angular velocity
        B.updateRotationMatrix();
        
        B.geometry.body = B;
    end
    
    function vec = getVecGlobalAxes(B, vecRel)
        %%
        vec = B.rotMatrix * vecRel;
    end
    
    function vec = getVectorRel(B, vecAbs)
        %%
        vec = B.rotMatrix' * vecAbs;
    end
    
    function pos = getPosAbsolute(B, pointRel)
        %%
        pos = zeros(size(pointRel));
        for i = 1 : size(pos,2)
            pos(:,i) = B.rotMatrix * pointRel(:,i) + B.position;
        end
    end
    
    function pos = getPositionRel(B, pointAbs)
        %%
        pos = zeros(size(pointAbs));
        for i = 1 : size(pos,2)
            pos(:,i) = B.rotMatrix' * (pointAbs(:,i) - B.position);
        end
    end
    
    function pos = getCOMPosAbsolute(B)
        %%
        pos = B.getPosAbsolute(B.com);
    end
    
    function vel = getVelAbsolute(B, pointRel)
        %%
        pos = B.getVecGlobalAxes(pointRel);
        vel = B.velocity + B.angVel * [-pos(2); pos(1)];
    end
    
    function updateRotationMatrix(B)
        %% Calculate rotation matrix with current orientation
        si = sin(B.angle);
        co = cos(B.angle);
        B.rotMatrix = [co, -si; si, co];
    end
    
    function pointsAbs = getGeometryAbs(B)
        %% Get geometry points in absolute frame
        pointsRel = B.geometry.points;
        pointsAbs = zeros(size(pointsRel));
        for k = 1 : size(pointsAbs, 2)
            pointsAbs(:,k) = B.rotMatrix * pointsRel(:,k) + B.position;
        end
    end
    
    function setOrientation(B, angle)
        %%
        B.angle = angle;
        B.updateRotationMatrix();
    end
    
    function setCOMPosition(B, posAbs)
        %%
        B.position = posAbs - B.rotMatrix * B.com;
    end
    
    function setAngularVelocity(B, omega)
        %%
        B.angVel = omega;
    end
    
    function setCOMVelocity(B, vel)
        %%
        B.velocity = vel - B.angVel * B.rotMatrix * [-B.com(2); B.com(1)];
    end
    
    function vel = getCOMVelocity(B)
        %% Get absolute point velocity from position in local frame
        vel = B.getPointVelocity(B.com);
    end
    
    function vel = getPointVelocity(B, posRel)
        %% Get absolute point velocity from position in local frame
        vel = B.velocity + B.angVel * B.rotMatrix * [-posRel(2); posRel(1)];
    end
    
    function vel = getVelocityRelInBody(B, posAbs, velAbs)
        %%
        velTransport = B.getPointVelocity(B.getPositionRel(posAbs));
        vel = B.getVectorRel(velAbs - velTransport);
    end
end % methods

end % classdef
