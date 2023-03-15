classdef Coordinate < handle
% COORDINATE 

properties
    
    name                string
    type                string % = "angular", "cartesian"
    
    body                Body
    reference           Body % = ground by default
    
    % For cartesian coordinates
    point               (2,1) double % on body
    refPoint            (2,1) double % on reference
    refDirection        (2,1) double % on reference
    
    initValue         	double
    initSpeed           double
    
end

methods
    function C = Coordinate()
        %% Coorinate class constructor
        
        C.point         = [0; 0];
        C.refDirection  = [1; 0];
        C.initValue     = 0;
        C.initSpeed    	= 0;
        
    end
    
    function initCoordinate(C, model)
        % Check coordinate type
        if ~((C.type == "angular") || (C.type == "cartesian"))
            error("Invalid coordinate type!")
        end
        
        % Set ground as default reference
        if ~numel(C.reference), C.reference = model.ground; end
        
        % Normalize direction
        C.refDirection = C.refDirection / norm(C.refDirection);
    end
    
    function value = getValue(C)
        %%
        if C.type == "angular"
            value = C.body.angle - C.reference.angle;
            
        elseif C.type == "cartesian"
            posPoint = C.body.getPosAbsolute(C.point);
            posRef   = C.reference.getPosAbsolute(C.refPoint);
            dir      = C.reference.getVecGlobalAxes(C.refDirection);
            value    = (posPoint - posRef)' * dir;
            
        else
            error("Invalid coordinate type!")
        end
    end
    
    function speed = getVelocity(C)
        %%
        if C.type == "angular"
            speed = C.body.angVel - C.reference.angVel;

        elseif C.type == "cartesian"
            posPoint 	= C.body.getPosAbsolute(C.point);
            velPoint    = C.body.getPointVelocity(C.point);
            velRel      = C.reference.getVelocityRelInBody(posPoint, velPoint);
            speed       = velRel' * C.refDirection;
            
        else
            error("Invalid coordinate type!")
        end
    end
    
    function [blocks,bodies] = getJacobianBlocks(C)
        %%
        if C.reference.fixed
            blocks = C.getJacobianBlockBody();
            bodies = C.body;

        elseif C.body.fixed
            blocks = C.getJacobianBlockReference();
            bodies = C.reference;

        else
            blocks(:,:,1) = C.getJacobianBlockBody();
            blocks(:,:,2) = C.getJacobianBlockReference();
            bodies(1) = C.body;
            bodies(2) = C.reference;
        end
    end
    
    function block = getJacobianBlockBody(C)
        %%
        if C.type == "angular"
            block = [0, 0, 1];
            
        elseif C.type == "cartesian"
            dirVec    	= C.reference.getVecGlobalAxes(C.refDirection);
            posBodyCOM 	= C.body.getCOMPosAbsolute();
            posPoint  	= C.body.getPosAbsolute(C.point);
            orthDist    = (posPoint - posBodyCOM)' * [dirVec(2); -dirVec(1)];
        
            block = [dirVec(1), dirVec(2), orthDist];
        else
            error("Coordinate type not supported")
        end
    end
    
    function block = getJacobianBlockReference(C)
        %%
        if C.type == "angular"
            block = [0, 0, -1];
            
        elseif C.type == "cartesian"
            dirVec    	= C.reference.getVecGlobalAxes(C.refDirection);
            posRefCOM 	= C.reference.getCOMPosAbsolute();
            posPoint  	= C.body.getPosAbsolute(C.point);
            orthDist    = (posPoint - posRefCOM)' * [dirVec(2); -dirVec(1)];
        
            block = -[dirVec(1), dirVec(2), orthDist];
        else
            error("Coordinate type not supported")
        end
    end
end
end

