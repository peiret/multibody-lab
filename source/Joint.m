classdef Joint < handle

properties
    
    name = ""
    type

    parent
    child

    pointParent
    pointChild
    directionParent
    directionChild

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
        posGlob = J.parent.getPosGlobalAxes(J.pointParent);
        block = -[...
            1, 0, -posGlob(2);
            0, 1, +posGlob(1)];
    end
    
    function block = getJacobianBlockChild(J)
        %%
        posGlob = J.child.getPosGlobalAxes(J.pointChild);
        block = [...
            1, 0, -posGlob(2);
            0, 1, +posGlob(1)];
    end
end
end
