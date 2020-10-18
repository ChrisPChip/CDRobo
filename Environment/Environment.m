classdef Environment < handle
    
    properties
        
        %% Object Transforms
        % Environment Objects
        table = [0 0 0];
        floor = [0 0 0];
        wall = [0 2 0];
        mirror = [2 0 0];
        shelf = [0.5 0 1.078];
        ext = [-1.6 1.8 0];
        tray = [-0.5 0 1.078];
        estop = [-1 0.5 1.078];
        
        % Cans
%         redPose;
%         greenPose;
%         bluePose;
        
        %% 3D model parameters
        % Ply data
        f;
        v;
        data;
        
    end
    
    %% Public Methods
    methods (Static)
        
        function generateObjects(obj)
            obj.generateStatics(obj);
%             obj.generateRedCan(obj, r);
%             obj.generateGreenCan(obj, g);
%             obj.generateBlueCan(obj, b);
        end
    end
    
    %% Private Methods
    methods (Static)
        
        function generateStatics(obj)
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
            [f,v,data] = plyread('shelf2.ply','tri');
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
            
            % Tray
            [f,v,data] = plyread('tray.ply','tri');
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            trayMesh_h = trisurf(f,v(:,1) + obj.tray(1),v(:,2) + obj.tray(2), v(:,3) + obj.tray(3) ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            hold on
            
            % ESTOP
            [f,v,data] = plyread('estop.ply','tri');
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            estopMesh_h = trisurf(f,v(:,1) + obj.estop(1),v(:,2) + obj.estop(2), v(:,3) + obj.estop(3) ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            hold on
            
            
        end

        function generateRedCan(obj, r)
            %% Insert PLY Data
            % Red Can
            [f,v,data] = plyread('redcan.ply','tri');
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            obj.redCanMesh_h = trisurf(f,v(:,1) + r(1),v(:,2) + r(2), v(:,3) + r(3) ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
%             hold on
        end
        
        function generateGreenCan(obj, g)
            % Green Can
            [f,v,data] = plyread('greencan.ply','tri');
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            obj.greenCanMesh_h = trisurf(f,v(:,1) + g(1),v(:,2) + g(2), v(:,3) + g(3) ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
%             hold on
        end
        
        function [loc] = generateBlueCan(obj, b)
            % Blue Can
            [f,v,data] = plyread('bluecan.ply','tri');
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            blueCanMesh_h = trisurf(f,v(:,1) + b(1),v(:,2) + b(2), v(:,3) + b(3) ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            hold on
            
            obj.bluePose  = transl(b);
            VertexCount = size(v,2);
            midPoint = sum(v)/VertexCount;
            PartVertices = v - repmat(midPoint, VertexCount, 1);
            loc = blueCanMesh_h;
            
            updatedPoints = [obj.bluePose * [PartVertices, ones(VertexCount, 1)]']';
            
            blueCanMesh_h.Vertices = updatedPoints(:, 1:3);
            
        end
        
    end
    
end

