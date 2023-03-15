classdef JointPinSlot < Joint
    % PIN SLOT JOINT Summary of this class goes here
    %   Detailed explanation goes here

methods
    function J = JointPinSlot()
        J.type = "PinSlot";
        J.nConstraints = 1;
    end
    
    function initJoint(J)
        J.geometry = Geometry(J.child);
        J.geometry.addPoint(J.pointChild);
        J.geometry.lineColor = 'k';
        J.geometry.lineWidth = 2;
        J.geometry.marker = 'o';
        J.geometry.markerSize = 10;
    end
        
    function pos = calcConstraintPos(J)
        %%
        posParent   = J.parent.getPosAbsolute(J.pointParent);
        posChild    = J.child.getPosAbsolute(J.pointChild);
        dirVec      = J.parent.getVecGlobalAxes(J.directionParent);
        
        pos = (posChild - posParent)' * [-dirVec(2); dirVec(1)];
    end

    function vel = calcConstraintVel(J)
        %%
        posChild    = J.child.getPosAbsolute(J.pointChild);
        velChild    = J.child.getPointVelocity(J.pointChild);
        velRel      = J.parent.getVelocityRelInBody(posChild, velChild);
        dirVec      = J.parent.getVecGlobalAxes(J.directionParent);
        
        vel = velRel' * [-dirVec(2); dirVec(1)];
    end
    
    function block = getJacobianBlockParent(J)
        %%
        dirVec          = J.parent.getVecGlobalAxes(J.directionParent);
        parentCOMPos    = J.parent.getCOMPosAbsolute();
        childPos        = J.child.getPosAbsolute(J.pointChild);
        
        block = -[-dirVec(2), dirVec(1), dirVec' * (childPos - parentCOMPos)];
    end
    
    function block = getJacobianBlockChild(J)
        %%
        dirVec          = J.parent.getVecGlobalAxes(J.directionParent);
        childCOMPos     = J.child.getCOMPosAbsolute();
        childPos        = J.child.getPosAbsolute(J.pointChild);
        
        block = [-dirVec(2), dirVec(1), dirVec' * (childPos - childCOMPos)];
    end
    
    function block = getJacobianDerivBlockParent(J)
        %%
        dirVec          = J.parent.getVecGlobalAxes(J.directionParent);
        dirVecDiff      = J.parent.angVel * [-dirVec(2); dirVec(1)];
        
        parentCOMPos	= J.parent.getCOMPosAbsolute();
        parentCOMVel    = J.parent.getCOMVelocity();
        
        childPos        = J.child.getPosAbsolute(J.pointChild);
        childVel        = J.child.getVelAbsolute(J.pointChild);
        
        block = -[-dirVecDiff(2), dirVecDiff(1), ...
            dirVecDiff' * (childPos - parentCOMPos) + dirVec' * (childVel - parentCOMVel)];
    end
    
    function block = getJacobianDerivBlockChild(J)
        %%
        dirVec          = J.parent.getVecGlobalAxes(J.directionParent);
        dirVecDiff      = J.parent.angVel * [-dirVec(2); dirVec(1)];
        
        childCOMPos     = J.child.getCOMPosAbsolute();
        childCOMVel    = J.child.getCOMVelocity();
        
        childPos        = J.child.getPosAbsolute(J.pointChild);
        childVel        = J.child.getVelAbsolute(J.pointChild);
        
        block = [-dirVecDiff(2), dirVecDiff(1), ...
            dirVecDiff' * (childPos - childCOMPos) + dirVec' * (childVel - childCOMVel)];
    end
end
end

