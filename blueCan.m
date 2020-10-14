function [blueLoc] = blueCan(partLocation)

[f,v,data] = plyread('bluecan.ply','tri');

% Vertex Count
canVertexCount = size(v, 1);
midPoint = sum(v)/canVertexCount;
canVertices = v - repmat(midPoint, canVertexCount, 1);

canPose  = transl([0 0 0]) * partLocation;

vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

blueCanMesh_h = trisurf(f,v(:,1) + partLocation(1),v(:,2) + partLocation(2), v(:,3) + partLocation(3) ...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
hold on

blueLoc = blueCanMesh_h;

updatedPoints = [canPose * [canVertices, ones(canVertexCount, 1)]']';

blueCanMesh_h.Vertices = updatedPoints(:,1:3);

end