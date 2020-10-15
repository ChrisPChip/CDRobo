function [greenLoc] = greenCan(partLocation)

[f,v,data] = plyread('greencan.ply','tri');

% Vertex Count
canVertexCount = size(v, 1);
midPoint = sum(v)/canVertexCount;
canVertices = v - repmat(midPoint, canVertexCount, 1);

canPose  = transl([0 0 0]) * partLocation;

vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

greenCanMesh_h = trisurf(f,v(:,1) + partLocation(1),v(:,2) + partLocation(2), v(:,3) + partLocation(3) ...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
hold on

greenLoc = greenCanMesh_h;

updatedPoints = [canPose * [canVertices, ones(canVertexCount, 1)]']';

greenCanMesh_h.Vertices = updatedPoints(:,1:3);

end