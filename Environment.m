classdef Environment < handle
    
    properties
        
        %% Object Transforms
        % Environment Objects
        table = [0 3 0];
        floor = [0 0 0];
        wall = [0 5 1.5];
        mirror = [-2 2.5 1.5];
        shelf = [0 .5 0];
        ext = [-1.6 0 0];
        
        % Cans
        red = [-0.5 3 1.08];
        green = [-1 3 1.08];
        blue = [1 3 1.08];
        
        %% 3D model parameters
        % Ply data
        f;
        v;
        data;
        
    end
    
    %% Public Methods
    methods (Static)
        
        function generateObjects(obj)
            obj.generateEnvironment(obj);
            obj.generateCans(obj);
        end
    end
    
    %% Private Methods
    methods (Static)
        
        function generateEnvironment(obj)
            %% Insert PLY data
            % Table
            [f,v,data] = plyread('table.ply','tri');
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            tableMesh_h = trisurf(f,v(:,1) + obj.table(1),v(:,2) + obj.table(2), v(:,3) + obj.table(3) ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            hold on
            
            % Floor
            [f,v,data] = plyread('Floor.ply','tri');
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            floorMesh_h = trisurf(f,v(:,1) + obj.floor(1),v(:,2) + obj.floor(2), v(:,3) + obj.floor(3) ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            hold on
            
            % Wall
            [f,v,data] = plyread('wall.ply','tri');
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            wallMesh_h = trisurf(f,v(:,1) + obj.wall(1),v(:,2) + obj.wall(2), v(:,3) +  obj.wall(3) ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            hold on
            
            % Mirror Wall
            [f,v,data] = plyread('mirrorwall.ply','tri');
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            mirrorMesh_h = trisurf(f,v(:,1) + obj.mirror(1),v(:,2) + obj.mirror(2), v(:,3) + obj.mirror(3) ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            hold on
            
            % Shelf
            [f,v,data] = plyread('shelf.ply','tri');
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            shelfMesh_h = trisurf(f,v(:,1) + obj.shelf(1),v(:,2) + obj.shelf(2), v(:,3) + obj.shelf(3) ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            hold on
            
            % Extinguisher
            [f,v,data] = plyread('extinguisher.ply','tri');
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            extinguisherMesh_h = trisurf(f,v(:,1) + obj.ext(1),v(:,2) + obj.ext(2), v(:,3) + obj.ext(3)...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            hold on
            
            
        end
        
        function generateCans(obj)
            %% Insert PLY Data
            % Red Can
            [f,v,data] = plyread('can.ply','tri');
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            redCanMesh_h = trisurf(f,v(:,1) + obj.red(1),v(:,2) + obj.red(3), v(:,3) + obj.red(3) ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            hold on
            
            % Green Can
            [f,v,data] = plyread('greencan.ply','tri');
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            greenCanMesh_h = trisurf(f,v(:,1) + obj.green(1),v(:,2) + obj.green(2), v(:,3) + obj.green(3) ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            hold on
            
            % Blue Can
            [f,v,data] = plyread('bluecan.ply','tri');
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            blueCanMesh_h = trisurf(f,v(:,1) + obj.blue(1),v(:,2) + obj.blue(2), v(:,3) + obj.blue(3) ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            hold on
            
        end
        
    end
end
