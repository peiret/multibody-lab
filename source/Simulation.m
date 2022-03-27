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
        
    end
    
    function run(Sim)
        %% RunSimulation
        %	Returns whether or not the simulation succeeded
        while Sim.time < Sim.timeEnd
            Sim.step();
            Sim.storage.saveStep();
            
            if ~isempty(Sim.viewer) && ishandle(Sim.viewer.figure)
                Sim.viewer.update();
            end
        end
        
    end
    
    function step(Sim)
        %%
        if strcmp(Sim.integrator, "Euler")
            Sim.stepEuler();
            
        elseif strcmp(Sim.integrator, "ode45")
            Sim.stepMatlab();
            
        else
            error("Integrator not compatible.");
        end
    end
    
    
    function stepEuler(Sim)
        %%
        sys = Sim.system;
        
        sys.updateSysDynamics();
        
        M = sys.mass;
        f = sys.force;
        c = sys.coriolis;
        
        qdd = M \ (f - c);
        
        % update sys.cooDep 
        
    end
    
    
    function stepMatlab(Sim)
        %%
        
    end
    
    function storage = initStorage(Sim)
        %%
        storage = Storage(Sim.system);
        Sim.storage = storage;
    end
end
end

