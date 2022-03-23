classdef Simulation < handle
    % SIMULATION
    
properties
    
    model           Model
    system          System
    viewer          Viewer
    storage         Storage
    
    integrator      % "EulerForward", "ode45", ...
    
    timeStart       double
    timeEnd         double
    timeStep        double
end

methods
    function Sim = Simulation(model)
        %% Constructor of the Simulation class
        
        Sim.model           = model;
        Sim.system          = model.initSystem();
        Sim.viewer          = model.viewer;
        
        Sim.integrator      = "none";
        
        Sim.timeStart       = 0;
        Sim.timeEnd         = 0;
        Sim.timeStep        = 0;
        
    end
    
    function initialize(Sim)
        %%
        
    end
    
    function done = run(Sim)
        %% RunSimulation
        %	Returns whether or not the simulation succeeded
        done = false;
        while Sim.step()
            Sim.storage()
        end
        
    end
    
    function done = step(Sim)
        done = false;
    end
    
    function storage = initStorage(Sim)
        %%
        storage = Storage(Sim.system);
        Sim.storage = storage;
    end
end
end

