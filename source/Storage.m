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
        %% Add varible to variables
        Sto.variables(end + 1) = string(varName);
    end
    
    function initData(Sto, nSteps)
        %% Initialize Data structure and preallocate memory
        
        nVal = 1; % always store time
        for varName = Sto.variables
            nVal = nVal + numel(Sto.system.(varName));
        end
        Sto.data = zeros(nSteps, nVal);
        Sto.idx = 0;
    end
    
    function saveStep(Sto, time)
        %% Save Data 
        
        % Increase table index
        Sto.idx = Sto.idx + 1;
        col = 1;
        
        Sto.data(Sto.idx, col) = time;
        
        for varName = Sto.variables
            var = Sto.system.(varName);
            nVar = numel(var);
            for k = 1 : nVar
                Sto.data(Sto.idx, col + k) = var(k);
            end
            col = col + nVar;
        end
        
    end
    
    function save(Sto, fileName)
        %% Save data in a file
        
        T = array2table(Sto.data);
        T.Properties.VariableNames{1} = 'time';
        
        col = 1;
        for varName = Sto.variables
            var = Sto.system.(varName);
            nVar = numel(var);
            if nVar > 1
                for k = 1 : nVar
                    T.Properties.VariableNames{col + k} = char(varName + "_" + k);
                end
            else
                T.Properties.VariableNames{col + 1} = char(varName);
            end
            col = col + nVar;
        end
        
        
        % save file
        writetable(T, fileName)
        
    end
end
end

