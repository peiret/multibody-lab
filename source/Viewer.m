classdef Viewer < handle
    
properties
    
    model           Model
    
    figure
    axes 
    
    geometrySet     Geometry
    
end % properties

methods
    function V = Viewer(model)
        %% Viewer of a mutlibody model
        V.model = model;
        model.viewer = V;
        
        V.figure                = figure;
        V.figure.Units          = 'normalized';
        V.figure.OuterPosition  = [0 0.05 0.5 0.6];
        V.figure.Visible        = 'on';
        V.figure.Color          = 'w';
        
        V.axes = gca;
    end
    
    function initViewer(V, xLim, yLim)
        %% Update view
        if ~exist('xLim', 'var'), xLim = [-1,1]; end
        if ~exist('yLim', 'var'), yLim = [-1,1]; end
        
        % Reset axes
        cla(V.axes);
        hold(V.axes, 'on');
        axis(V.axes, 'equal', 'off');
        
        % Reset geometrySet
        V.geometrySet = V.geometrySet([]);
        
        plot(V.axes, [-0.2, 0.2], [0, 0], 'k');
        plot(V.axes, [0, 0], [-0.2, 0.2], 'k');
        
        for body = V.model.bodySet
            V.addGeometry(body.geometry);
        end
        
        for joint = V.model.jointSet
            V.addGeometry(joint.geometry);
        end
        
        for g = V.geometrySet
            g.plot(V.axes);
        end
        
        V.axes.XLim = xLim;
        V.axes.YLim = yLim;

    end
    
    function addGeometry(V, geometry)
        %% Add geometry to the viewer
        V.geometrySet(end+1) = geometry;
    end
    
    function update(V)
        %%
        for g = V.geometrySet
            g.update();
        end
    end
    
    function clear(V)
        %%
        cla(V.axes);
    end
    
    function showAxes(V, flag)
        %% Show axis (true/false)
        if ~exist('flag', 'var') || flag
          	axis(V.axes, 'on');
        else
          	axis(V.axes, 'off');
        end
    end
    
end % methods

end % classdef

