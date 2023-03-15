classdef Joint < handle & matlab.mixin.Heterogeneous

properties
    
    name                string
    type                string

    parent              Body
    child               Body

    pointParent         (2,1) double
    pointChild          (2,1) double
    directionParent     (2,1) double
    directionChild      (2,1) double

    nConstraints        double
end

methods
    function J = Joint()
        %% Constructor of the Joint class

        J.type = "Abstract";

        J.pointParent       = [0; 0];
        J.pointChild        = [0; 0];
        J.directionParent   = [1; 0];
        J.directionChild    = [1; 0];

    end

    
    function [blocks, bodies] = getJacobianBlocks(J)
        %%
        
        if J.parent.fixed
            blocks = J.getJacobianBlockChild();
            bodies = J.child;
        elseif J.child.fixed
            blocks = J.getJacobianBlockParent();
            bodies = J.parent;
        else
            blocks(:,:,1) = J.getJacobianBlockParent();
            blocks(:,:,2) = J.getJacobianBlockChild();
            bodies(1) = J.parent;
            bodies(2) = J.child;
        end
            
    end
    
    function [blocks, bodies] = getJacobianDerivativeBlocks(J)
        %%
        
        if J.parent.fixed
            blocks = J.getJacobianDerivBlockChild();
            bodies = J.child;
        elseif J.child.fixed
            blocks = J.getJacobianDerivBlockParent();
            bodies = J.parent;
        else
            blocks(:,:,1) = J.getJacobianDerivBlockParent();
            blocks(:,:,2) = J.getJacobianDerivBlockChild();
            bodies(1) = J.parent;
            bodies(2) = J.child;
        end
            
    end
    
end

methods (Abstract)
    
    pos = calcConstraintPos(J)

    vel = calcConstraintVel(J)
    
    block = getJacobianBlockParent(J)
    
    block = getJacobianBlockChild(J)
    
    block = getJacobianDerivBlockParent(J)
    
    block = getJacobianDerivBlockChild(J)
    
end
end
