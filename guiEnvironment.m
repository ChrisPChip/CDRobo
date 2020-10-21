function guiEnvironment

table = [0 0 0];
floor = [0 0 0];
wall = [0 2 0];
mirror = [2 0 0];
shelf = [0 -0.95 1.078];
ext = [-1.6 1.8 0];
tray = [-0.9 0 1.078];
estop = [-1 0.5 1.078];

%% CHECK COLLISION BOXES HERE

% tableC = [0 0 0.55];
% tableS = [2 1.5 1.1];
% 
% % Shelf1
% shelf1C = [0 -.7 1.4];
% shelf1S = [0.8 0.2 0.02];
% % Shelf2
% shelf2C = [0 -.7 1.4255];
% shelf2S = [0.8 0.2 0.02];
% 
% % Shelf3
% shelf3C = [0 -.7 1.6925];
% shelf3S = [0.8 0.2 0.02];
% 
% % Shelf4
% shelf4C = [0 -.7 1.9595];
% shelf4S = [0.8 0.2 0.02];
% 
% % Shelf5
% shelf5C = [0 -.7 2.2265];
% shelf5S = [0.8 0.2 0.02];
% 
% plotOptions.plotFaces = true;
% [vertex1,faces1,Normal1] = RectangularPrism(tableC + tableS/2, tableC - tableS/2,plotOptions);
% [vertex2,faces2,Normal2] = RectangularPrism(shelf1C + shelf1S, shelf1C - shelf1S,plotOptions);
% [vertex3,faces3,Normal3] = RectangularPrism(shelf2C + shelf2S, shelf2C - shelf2S,plotOptions);
% [vertex4,faces4,Normal4] = RectangularPrism(shelf3C + shelf3S, shelf3C - shelf3S,plotOptions);
% [vertex5,faces5,Normal5] = RectangularPrism(shelf4C + shelf4S, shelf4C - shelf4S,plotOptions);
% [vertex6,faces6,Normal6] = RectangularPrism(shelf5C + shelf4S, shelf5C - shelf5S,plotOptions);

%%

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