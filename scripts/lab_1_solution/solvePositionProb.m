function solvePositionProb(system, cooInd_0, cooDep_0, viewer)
% SOLVE POSITION PROB 

% init system and model
%q = cooDep_0;
q = system.cooDep;
system.updateModel();

epsilon = 1E-6; % tolerancia
step = 1; % || q_k+1 - q_k ||

while step > epsilon
    
    % q = system.cooDep;

    system.updateJacobians();
    J = [system.conJac; system.indJac];
    
    system.updateSysPosition();
    beta = [system.conPos; system.cooInd - cooInd_0];
    
    q = q - (J \ beta);
    
    step = norm(q - system.cooDep);
    
    system.cooDep = q;
    system.updateModel();
    
    %viewer.update();
    
end
end

