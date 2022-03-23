classdef Viewer < handle
    
properties
    
    model
    
    figure
    axes
    
    shapes = gobjects(0)
    
    
end % properties

methods
    function V = Viewer(model)
        %% Viewer of a mutlibody model
        
        V.model     = model;
        
        V.figure                = figure;
        V.figure.Units          = 'normalized';
        V.figure.OuterPosition  = [0 0.05 0.5 0.6];
        V.figure.Visible        = 'on';
        V.figure.Color          = 'w';
        
        V.axes = gca;
        hold(V.axes, 'on');
        axis(V.axes, 'equal');
        %axis(V.axes, 'off');
        
        V.initViewer();
        
    end
    
    function initViewer(V)
        %% Update view
        
        plot(V.axes, [0 1], [0 0], 'r');
        plot(V.axes, [0 0], [0 1], 'g');
        
        for i = 1 : V.model.nBodies
            body = V.model.bodySet(i);
            geoAbs = body.getGeometryAbs();
            
            V.shapes(i) = plot(V.axes, geoAbs(1,:), geoAbs(2,:),...
                'Color', body.geometry.lineColor,...
                'LineWidth', body.geometry.lineWidth,...
                'Marker', 'o',...
                'MarkerSize', 12,...
                'MarkerFaceColor', 'w');
        end

    end
    
    function update(V)
        %%
        for i = 1 : V.model.nBodies
            body = V.model.bodySet(i);
            geoAbs = body.getGeometryAbs();
            shape = V.shapes(i);
            
            shape.XData = geoAbs(1,:);
            shape.YData = geoAbs(2,:);
            shape.Color = body.geometry.lineColor;
            shape.LineWidth = body.geometry.lineWidth;
            
        end
    end
    
    function clear(V)
        %%
        cla(V.axes);
    end
    
end % methods

end % classdef

