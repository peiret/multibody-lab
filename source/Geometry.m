classdef Geometry < handle
    % GEOMETRY
    
properties
    points              (2,:) double
    body                Body
    
    lineWidth           double      = 1
    lineColor           char        = 'k'
    
    marker              char        = 'none'
    markerSize          double      = 4
    markerEdgeColor     char        = 'k'
    markerFaceColor     char        = 'w'
    
    graphObject = gobjects(0)
end

methods
    function G = Geometry(fixedToBody)
        %%
        G.body      = fixedToBody;
        G.points    = zeros(2,0);
        
    end
    
    function addPoint(G, point)
        %% Add point to geometry
        G.points(:, end+1) = point;
    end

    function pointsAbs = getPointsInGround(G)
        %% Get geometry points in ground frame
        pointsAbs = G.body.getPosAbsolute(G.points);
    end
    
    function plot(G, plotAxes)
        %%
        pos = G.getPointsInGround();
        G.graphObject = plot(plotAxes, pos(1,:), pos(2,:),...
                'Color', G.lineColor,...
                'LineWidth', G.lineWidth,...
                'Marker', G.marker,...
                'MarkerSize', G.markerSize,...
                'MarkerFaceColor', G.markerFaceColor,...
                'MarkerEdgeColor', G.markerEdgeColor);
    end
    
    function update(G)
        %%
        pos = G.getPointsInGround();

        G.graphObject.XData = pos(1,:);
        G.graphObject.YData = pos(2,:);
        G.graphObject.Color = G.lineColor;
        G.graphObject.LineWidth = G.lineWidth;
    end
end
end

