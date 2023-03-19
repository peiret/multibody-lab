classdef Simulation < handle
    % SIMULATION
    
properties
    
    model           Model
    system          System
    viewer          Viewer
    storage         Storage
    
    integrator      string % "Euler", "ode45", ...
    
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
            
            clock = tic;
            Sim.step();
            Sim.elapsedTime = toc(clock);
            
            if ~isempty(Sim.storage)
                Sim.storage.saveStep(Sim.time);
            end
            
            if ~isempty(Sim.viewer) && ishandle(Sim.viewer.figure)
                Sim.viewer.update();
                pause(max(Sim.timeStep - toc(clock), 1E-6));
            end
        end
        
    end
    
    function step(Sim)
        %%
        if strcmp(Sim.integrator, "Euler")
            Sim.stepEuler();
            
        elseif strcmp(Sim.integrator, "EulerImplicit")
            Sim.stepEulerImplicit();
            
        elseif strcmp(Sim.integrator, "EulerImplicitCorrected")
            Sim.stepEulerImplicitCorrected();
            
        else % Use any other integrator function
            feval(Sim.integrator, Sim);
        end
    end
    
    function stepEuler(Sim)
        %% Step simulation using the Euler method
        Sim.system.updateSystem();
        Sim.system.solveDynamics();
        
        Sim.system.cooDep = Sim.system.cooDep + Sim.timeStep * Sim.system.velDep;
        Sim.system.velDep = Sim.system.velDep + Sim.timeStep * Sim.system.accDep;
        
        Sim.system.updateModel();
    end
    
    function stepEulerImplicit(Sim)
        %% Step simulation using the semi-implicit Euler method
        Sim.system.updateSystem();
        Sim.system.solveDynamics();
        
        Sim.system.velDep = Sim.system.velDep + Sim.timeStep * Sim.system.accDep;
        Sim.system.cooDep = Sim.system.cooDep + Sim.timeStep * Sim.system.velDep;
        
        Sim.system.updateModel();
    end
    
    function stepEulerImplicitCorrected(Sim)
        %% Step simulation using the semi-implicit Euler method with corrected position
        Sim.system.updateSystem();
        Sim.system.solveDynamics();
        
        Sim.system.velDep = Sim.system.velDep + Sim.timeStep * Sim.system.accDep;
        Sim.system.cooDep = Sim.system.cooDep + Sim.timeStep * Sim.system.velDep;
        
        Sim.system.updateModel();
        Sim.system.updateIndependentCoordinates();
        Sim.system.solvePositionProb();
        Sim.system.updateModel();
    end
end
end

