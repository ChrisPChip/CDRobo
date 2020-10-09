function readData(object)
[f,v,data] = plyread(num2str(object)','.ply','tri');
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
FloorMesh_h = trisurf(f,v(:,1),v(:,2), v(:,3) ...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
hold on
end