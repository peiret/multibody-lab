classdef Body < handle
properties
    
    name            string
    fixed           logical
    
    mass            double
    inertia         double
    com             (2,1) double

    % Appearance Properties
    geometry        = struct('points', [], 'lineColor', '', 'lineWidth', 0);

    % Kinematics
    position        (2,1) double
    velocity        (2,1) double
    
    angle           double
    angVel          double
end

methods
    function B = Body()
        %% Body initialization
        
        % If the body is fixed to the ground
        B.fixed = false;
        
        % Mechanical Properties:
        B.mass = 1;
        B.inertia = 1; % about the center of mass
        B.com = [0; 0];
        
        % Appearance Properties
        B.geometry.points = [];
        B.geometry.color = 'k';
        
        % Origin Kinematics
        B.position = [0; 0]; % position of the origin
        B.velocity = [0; 0]; % velocity of the origin
        
        % Orientation
        B.angle    = 0; % rotation
        B.angVel   = 0; % angular velocity
        
    end
    
    function pos = getPosGlobalAxes(B, pointRel)
        %%
        Rot = B.getRotationMatrix();
        pos = Rot * pointRel;
    end
    
    function pos = getPosAbsolute(B, pointRel)
        %%
        Rot = B.getRotationMatrix();
        pos = Rot * pointRel + B.position;
    end
    
    function vel = getVelAbsolute(B, pointRel)
        pos = B.getPosAbsolute(pointRel);
        vel = B.velocity + B.angVel * [-pos(2); pos(1)];
    end
    
    function R = getRotationMatrix(B)
        %% Get rotation matrix
        
        si = sin(B.angle);
        co = cos(B.angle);
        R = [co, -si; si, co];
        
    end
    
    function pointsAbs = getGeometryAbs(B)
        %% Get geometry points in absolute frame
        pointsRel = B.geometry.points;
        pointsAbs = zeros(size(pointsRel));
        Rot = B.getRotationMatrix();
        for k = 1 : size(pointsAbs, 2)
            pointsAbs(:,k) = Rot * pointsRel(:,k) + B.position;
        end
    end
    
end % methods

end % classdef
