function guiEnvironment

table = [0 0 0];
floor = [0 0 0];
wall = [0 2 0];
mirror = [2 0 0];
shelf = [0 -0.75 1.078];
ext = [-1.6 1.8 0];
tray = [-0.5 0 1.078];
estop = [-1 0.5 1.078];

% Table
            [f,v,data] = plyread('table.ply','tri');
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            tableMesh_h = trisurf(f,v(:,1) + table(1),v(:,2) + table(2), v(:,3) + table(3) ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            hold on
            
            % Floor
            [f,v,data] = plyread('Floor.ply','tri');
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            floorMesh_h = trisurf(f,v(:,1) + floor(1),v(:,2) + floor(2), v(:,3) + floor(3) ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            hold on
            
            % Wall
            [f,v,data] = plyread('wall.ply','tri');
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            wallMesh_h = trisurf(f,v(:,1) + wall(1),v(:,2) + wall(2), v(:,3) +  wall(3) ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            hold on
            
            % Mirror Wall
            [f,v,data] = plyread('mirrorwall.ply','tri');
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            mirrorMesh_h = trisurf(f,v(:,1) + mirror(1),v(:,2) + mirror(2), v(:,3) + mirror(3) ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            hold on
            
           % Shelf
            [f,v,data] = plyread('shelf2.ply','tri');
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            shelfMesh_h = trisurf(f,v(:,1) + shelf(1),v(:,2) + shelf(2), v(:,3) + shelf(3) ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            hold on 
            
            % Extinguisher
            [f,v,data] = plyread('extinguisher.ply','tri');
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            extinguisherMesh_h = trisurf(f,v(:,1) + ext(1),v(:,2) + ext(2), v(:,3) + ext(3)...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            hold on
            
            % Tray
            [f,v,data] = plyread('tray.ply','tri');
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            trayMesh_h = trisurf(f,v(:,1) + tray(1),v(:,2) + tray(2), v(:,3) + tray(3) ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            hold on
            
            % ESTOP
            [f,v,data] = plyread('estop.ply','tri');
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            estopMesh_h = trisurf(f,v(:,1) + estop(1),v(:,2) + estop(2), v(:,3) + estop(3) ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            hold on

end