classdef Joint < handle

properties
    
    name                string
    type                string

    parent              Body
    child               Body

    pointParent         (2,1) double
    pointChild          (2,1) double
    directionParent     (2,1) double
    directionChild      (2,1) double

end
methods
    function J = Joint()
        %% Constructor of the Joint class

        J.type = "none";

        J.pointParent       = [0; 0];
        J.pointChild        = [0; 0];
        J.directionParent   = [1; 0];
        J.directionChild    = [1; 0];

    end

    function pos = calcConstraintPos(J)
        %%
        posParent = J.parent.getPosAbsolute(J.pointParent);
        posChild = J.child.getPosAbsolute(J.pointChild);
        pos = posChild - posParent;
    end

    function vel = calcConstraintVel(J)
        %%
        velParent = J.parent.getPointVelocity(J.pointParent);
        velChild = J.child.getPointVelocity(J.pointChild);
        vel = velChild - velParent;
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
    
    function block = getJacobianBlockParent(J)
        %%
        posGlob = J.parent.getPosGlobalAxes(J.pointParent - J.parent.com);
        block = -[...
            1, 0, -posGlob(2);
            0, 1, +posGlob(1)];
    end
    
    function block = getJacobianBlockChild(J)
        %%
        posGlob = J.child.getPosGlobalAxes(J.pointChild - J.child.com);
        block = [...
            1, 0, -posGlob(2);
            0, 1, +posGlob(1)];
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
    
    function block = getJacobianDerivBlockParent(J)
        %%
        posGlob = J.parent.getPosGlobalAxes(J.pointParent - J.parent.com);
        angVel  = J.parent.angVel;
        block = -[...
            0, 0, -angVel * posGlob(1);
            0, 0, -angVel * posGlob(2)];
    end
    
    function block = getJacobianDerivBlockChild(J)
        %%
        posGlob = J.child.getPosGlobalAxes(J.pointChild - J.child.com);
        angVel  = J.child.angVel;
        block = [...
            0, 0, -angVel * posGlob(1);
            0, 0, -angVel * posGlob(2)];
    end
end
end
