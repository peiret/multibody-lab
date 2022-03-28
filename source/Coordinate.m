classdef Coordinate < handle
% COORDINATE 

properties
    
    name                string
    component           string % = "Body", "Joint"
    type                string % = "angular", "cartesian"
    
    % only one is necessary
    body                Body
    joint               Joint
    
    point               (2,1) double % on body
    direction           (2,1) double % on ground
    
    initialPos          double
    initialVel          double
    
end

methods
    function C = Coordinate()
        %% Coorinate class constructor
        
        C.point       = [0; 0];
        C.direction   = [0; 0];
        C.initialPos  = 0;
        C.initialVel  = 0;
        
    end
    
    function value = getValue(C)
        %%
        if C.component == "Body"
            if C.type == "angular"
                value = C.body.angle;
            elseif C.type == "cartesian"
                pos = C.body.getPosAbsolute(C.point);
                dir = C.direction / norm(C.direction);
                value = pos' * dir;
            else
                error("Coordinate type not supported")
            end
        
        elseif C.component == "Joint" 
            if C.type == "angular"
                value = C.joint.child.angle - C.joint.parent.angle;
            else % elseif C.type == "cartesian"
                % TO-DO support relative cartessian coordinates
                error("Coordinate type not supported")
            end
            
        else
            error("Coordinate component not supported")
        end
    end
    
    function speed = getVelocity(C)
        
        if C.component == "Body"
            
            if C.type == "angular"
                speed = C.body.angVel;
                
            elseif C.type == "cartesian"
                vel = C.body.getVelAbsolute(C.point);
                dir = C.direction / norm(C.direction);
                speed = vel' * dir;
            else
                error("Coordinate type not supported")
            end
        
        elseif C.component == "Joint" 
            
            if C.type == "angular"
                speed = C.joint.child.angVel - C.joint.parent.angVel;
                
            else % elseif C.type == "cartesian"
                % TO-DO support relative cartessian coordinates
                error("Coordinate type not supported")
            end
            
        else
            error("Coordinate component not supported")
        end
    end
    
    function [blocks,bodies] = getJacobianBlocks(C)
        %%
        if C.component == "Body"
            blocks = C.getJacobianBlockBody();
            bodies = C.body;
            
        elseif C.component == "Joint"
            
            if C.joint.parent.fixed
                blocks = C.getJacobianBlockJointChild();
                bodies = C.joint.child;
                
            elseif C.joint.child.fixed
                blocks = C.getJacobianBlockJointParent();
                bodies = c.joint.parent;
                
            else
                blocks(:,:,1) = C.getJacobianBlockJointParent();
                blocks(:,:,2) = C.getJacobianBlockJointChild();
                bodies(1) = C.joint.parent;
                bodies(2) = C.joint.child;
            end
            
        else
            error("Coordinate component not supported")
        end
    end
    
    function block = getJacobianBlockBody(C)
        %%
        if C.type == "angular"
            block = [0, 0, 1];
            
        elseif C.type == "cartesian"
            posGlob = C.body.getPosGlobalAxes(C.point);
            dir = C.direction / norm(C.direction);
            block = dir' * [...
                1, 0, -posGlob(2);
                0, 1, +posGlob(1)];
        else
            error("Coordinate type not supported")
        end
    end
    
    function block = getJacobianBlockJointChild(C)
        %%
        if C.type == "angular"
            block = [0, 0, 1];
        else
            error("Coordinate type not supported")
        end
    end
    
    function block = getJacobianBlockJointParent(C)
        %%
        if C.type == "angular"
            block = [0, 0, -1];
        else
            error("Coordinate type not supported")
        end
    end
end
end

