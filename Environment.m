classdef Environment < handle
    
    properties
        
        % Environment area
%         workspace = [-3 3 -3 3 -0.5 2];
        
        %% 3D model parameters
        % Ply data
        f;
        v;
        data;
        
        %         tableMesh_h;
        %         floorMesh_h;
        %         wallMesh_h;
        %         mirrorMesh_h;
        %         blueCanMesh_h;
        %         greenCanMesh_h;
        %         redCanMesh_h;
        %         shelfMesh_h;
        %         extinguisherMesh_h;
        
    end
    
    %% Public Methods
    methods (Static)
        
        function generateObjects(obj)
            obj.generateEnvironment();
            obj.generateCans();
        end
    end
    
    %% Private Methods
    methods (Static)
        
        function generateEnvironment()
            % Table
            [f,v,data] = plyread('table.ply','tri');
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            tableMesh_h = trisurf(f,v(:,1),v(:,2)+3, v(:,3) ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            hold on
            
            % Floor
            [f,v,data] = plyread('Floor.ply','tri');
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            floorMesh_h = trisurf(f,v(:,1),v(:,2), v(:,3) ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            hold on
            
            % Wall
            [f,v,data] = plyread('wall.ply','tri');
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            wallMesh_h = trisurf(f,v(:,1),v(:,2)+5, v(:,3)+1.5 ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            hold on
            
            % Mirror Wall
            [f,v,data] = plyread('mirrorwall.ply','tri');
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            mirrorMesh_h = trisurf(f,v(:,1)-2,v(:,2)+2.5, v(:,3)+1.5 ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            hold on
            
            % Shelf
            [f,v,data] = plyread('shelf.ply','tri');
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            shelfMesh_h = trisurf(f,v(:,1),v(:,2)+.5, v(:,3) ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            hold on
            
            % Extinguisher
            [f,v,data] = plyread('extinguisher.ply','tri');
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            extinguisherMesh_h = trisurf(f,v(:,1)-1.6,v(:,2), v(:,3)...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            hold on
            
            
        end
        
        function generateCans()
            
            % Red Can
            [f,v,data] = plyread('can.ply','tri');
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            redCanMesh_h = trisurf(f,v(:,1)-.5,v(:,2)+3, v(:,3)+1.08 ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            hold on
            
            % Green Can
            [f,v,data] = plyread('greencan.ply','tri');
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            greenCanMesh_h = trisurf(f,v(:,1)-1,v(:,2)+3, v(:,3)+1.08 ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            hold on
            
            % Blue Can
            [f,v,data] = plyread('bluecan.ply','tri');
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            blueCanMesh_h = trisurf(f,v(:,1)+1,v(:,2)+3, v(:,3)+1.08 ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            hold on
            
        end
        
    end
end
