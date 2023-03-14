classdef JointRevolute < Joint
    %JOINTREVOLUTE Summary of this class goes here
    %   Detailed explanation goes here

methods
    function J = JointRevolute()
        J.type = "Revolute";
        J.nConstraints = 2;
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

