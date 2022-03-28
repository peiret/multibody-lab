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
        
        Sim.integrator      = "none";
        Sim.timeStart       = 0;
        Sim.timeEnd         = 0;
        Sim.timeStep        = 0;
        
    end
    
    function initialize(Sim)
        %%
        
        Sim.time = Sim.timeStart;
        Sim.nSteps = ceil((Sim.timeEnd - Sim.timeStart) / Sim.timeStep);
        
        Sim.system.initSystem();
        
        if ~isempty(Sim.storage)
            Sim.storage.initData(Sim.nSteps + 1);
            Sim.storage.saveStep(Sim.time); %save initial state
        end
    end
    
    function storage = initStorage(Sim)
        %%
        storage = Storage(Sim.system);
        Sim.storage = storage;
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
                pause(Sim.timeStep - toc(clock));
            end
        end
        
    end
    
    function step(Sim)
        %%
        if strcmp(Sim.integrator, "Euler")
            Sim.stepEuler();
        else
            error("Integrator not compatible.");
        end
    end
    
    
    function stepEuler(Sim)
        %% Step simulation using the Euler method
        
        %% TO-DO: implement the forward Euler time-stepping
        % - Update system independent coordinates and velocities
        % - Update model
        
    end
end
end

