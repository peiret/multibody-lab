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
        
        V.figure                = figure;
        V.figure.Units          = 'normalized';
        V.figure.OuterPosition  = [0 0.05 0.5 0.6];
        V.figure.Visible        = 'on';
        V.figure.Color          = 'w';
        
        V.axes = gca;
        hold(V.axes, 'on');
        axis(V.axes, 'equal');
        
    end
    
    function initViewer(V, xlim, ylim)
        %% Update view
        if ~exist('xlim', 'var'), xlim = [-1,1]; end
        if ~exist('ylim', 'var'), ylim = [-1,1]; end
        
        plot(V.axes, xlim, [0, 0], 'k');
        plot(V.axes, [0, 0], ylim, 'k');
        
        for body = V.model.bodySet
            V.addGeometry(body.geometry);
        end
        
        for joint = V.model.jointSet
            V.addGeometry(joint.geometry);
        end
        
        for g = V.geometrySet
            g.plot(V.axes);
        end
        
        V.axes.XLim = xlim;
        V.axes.YLim = ylim;

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

