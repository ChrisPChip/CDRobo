function [redLoc] = redCan(partLocation)

[f,v,data] = plyread('redcan.ply','tri');

% Vertex Count
canVertexCount = size(v, 1);
midPoint = sum(v)/canVertexCount;
canVertices = v - repmat(midPoint, canVertexCount, 1);

canPose  = transl([0 0 0]) * partLocation;

vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

redCanMesh_h = trisurf(f,v(:,1) + partLocation(1),v(:,2) + partLocation(2), v(:,3) + partLocation(3) ...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
hold on

redLoc = redCanMesh_h;

updatedPoints = [canPose * [canVertices, ones(canVertexCount, 1)]']';

redCanMesh_h.Vertices = updatedPoints(:,1:3);

end