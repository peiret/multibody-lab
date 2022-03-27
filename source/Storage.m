classdef Storage < handle
    % STORAGE
    
properties
    
    system          System
    data            double
    variables       string
    
    idx             double
end

methods
    function Sto = Storage(system)
        %% Constructor of Storage class
        Sto.system = system;
    end
    
    function addVariable(Sto, varName)
        %%
        Sto.variables(end + 1) = string(varName);
    end
    
    function initData(Sto, nSteps)
        %% Initialize Data structure and preallocate memory
        nVal = 0;
        for varName = Sto.variables
            nVal = nVal + numel(Sto.system.(varName));
        end
        Sto.data = zeros(nSteps, nVal);
        Sto.idx = 0;
    end
    
    function saveStep(Sto)
        %% Save Data 
        
        col = 0;
        for varName = Sto.variables
            var = Sto.system.(varName);
            nVar = numel(val);
            for k = 1 : nVar
                Sto.data(Sto.idx, col + k) = var(k);
            end
            col = col + nVar;
        end
        
        % Increase table index
        Sto.idx = Sto.idx + 1;
    end
    
    function save(Sto, fileName)
        %%
        [dir,name,ext] = fileparts(fileName);
        % check extension
        % TO-DO save data to file
        
        % generate header
        
    end
end
end

