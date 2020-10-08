classdef Environment < handle
    
    properties
        
        % Environment area
        workspace = [-3 3 -3 3 -0.5 2];
        
        %% Placement Transforms
        % Generally centrePoints +/- an offset
        floor = [0 0 0];
        extinguisher = [0 0 0];
        table = [0 0 0];
        wall = [0 0 0];
        mirrorwall = [0 0 0];
        
        %% 3D model parameters
        % Ply data
        faceData;
        vertexData;
        plyData;
        
    end
    
    %% Public Methods
    methods (Static)
        
        function generateObjects()
            generateEnvironment(centrePoint);
            generateCans(centrePoint);
            
        end
    end
    
    %% Private Methods
    methods (Access = private)
        function centrePoint = getMidpoint(workspace)
            centrePoint = [(workspace(1) + workspace(2))/2, (workspace(3) + workspace(4))/2, 0];
        end
    end
end
