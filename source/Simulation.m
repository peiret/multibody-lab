classdef Simulation < handle
    % SIMULATION
    
properties
    
    model           Model
    system          System
    viewer          Viewer
    storage         Storage
    
    integrator      string % "EulerExplicit" "EulerImplicit"
    correctPosition logical
    correctVelocity logical
    
    time            double
    timeStart       double
    timeEnd         double
    timeStep        double
    
    nSteps          int32
    elapsedTime     double
end

methods
    function Sim = Simulation(model)
        %% Constructor of the Simulation class
        
        Sim.model = model;
        Sim.viewer = model.viewer;
        
        if isempty(model.system)
            Sim.system = model.initSystem();
        else
            Sim.system = model.system;
        end
        
        Sim.integrator      = "EulerImplicit";
        Sim.timeStart       = 0;
        Sim.timeEnd         = 1;
        Sim.timeStep        = 0.01;
        
        Sim.correctPosition = true;
        Sim.correctVelocity = true;
        
        Sim.storage = Storage(model.system);
    end
    
    function initialize(Sim)
        %%
        
        Sim.time = Sim.timeStart;
        Sim.nSteps = ceil((Sim.timeEnd - Sim.timeStart) / Sim.timeStep);
        
        Sim.system.initSystem();
        Sim.system.updateModel();
        
        if ~isempty(Sim.storage)
            Sim.storage.initData(Sim.nSteps + 1);
            Sim.storage.saveStep(Sim.time); %save initial state
        end
    end
    
    function run(Sim)
        %% Run Simulation
        
        for k = 1 : Sim.nSteps
            
            % Begin step
            clock = tic;
            Sim.step();
            
            % Constraint correction after integration
            if Sim.correctPosition, Sim.constraintPositionCorrection();
            end
            if Sim.correctVelocity, Sim.constraintVelocityCorrection();
            end
            
            % End step
            Sim.elapsedTime = toc(clock);
            
            % Save Storage
            if ~isempty(Sim.storage), Sim.storage.saveStep(Sim.time);
            end
            
            % Update Viewer
            if ~isempty(Sim.viewer) && ishandle(Sim.viewer.figure)
                Sim.viewer.update();
                pause(max(Sim.timeStep - toc(clock), 1E-6));
            end
        end
        
    end
    
    function step(Sim)
        %%
        if strcmp(Sim.integrator, "EulerExplicit")
            Sim.stepEulerExplicit();
            
        elseif strcmp(Sim.integrator, "EulerImplicit")
            Sim.stepEulerImplicit();
            
        else % Use any other integrator function
            feval(Sim.integrator, Sim);
        end
    end
    
    function stepEulerExplicit(Sim)
        %% Step simulation using the Euler method
        Sim.system.update();
        Sim.system.solveDynamics();
        
        Sim.system.depenCoord = Sim.system.depenCoord + Sim.timeStep * Sim.system.depenVeloc;
        Sim.system.depenVeloc = Sim.system.depenVeloc + Sim.timeStep * Sim.system.depenAccel;
        
        Sim.system.updateModel();
    end
    
    function stepEulerImplicit(Sim)
        %% Step simulation using the semi-implicit Euler method
        Sim.system.update();
        Sim.system.solveDynamics();
        
        Sim.system.depenVeloc = Sim.system.depenVeloc + Sim.timeStep * Sim.system.depenAccel;
        Sim.system.depenCoord = Sim.system.depenCoord + Sim.timeStep * Sim.system.depenVeloc;
        
        Sim.system.updateModel();
    end
    
    function constraintPositionCorrection(Sim)
        %% Correct the constraint position error
        Sim.system.updateIndependentCoordinates();
        Sim.system.solvePositionProb();
        Sim.system.updateModel();
    end
    
    function constraintVelocityCorrection(Sim)
        %% Correct the constraint velocity error
        Sim.system.updateIndependentVelocity();
        Sim.system.solveVelocityProb();
        Sim.system.updateModel();
    end
end
end

