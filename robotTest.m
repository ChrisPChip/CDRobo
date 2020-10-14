clear;
clc;

workspace = [-1 1 -1 1 0 1.5];

name = ['KinovaGen3_',datestr(now,'yyyymmddTHHMMSSFFF')];

L(1) = Link('d',0.2848,'a',0,'alpha',-pi/2,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
L(2) = Link('d',0.0054,'a',0.41,'alpha',-pi,'offset',-pi/2,'qlim',[deg2rad(-90),deg2rad(90)]);
L(3) = Link('d',0.0064,'a',0,'alpha',-pi/2,'offset',-pi/2,'qlim',[deg2rad(-147.8),deg2rad(147.8)]);
L(4) = Link('d',0.2084 + 0.1059,'a',0,'alpha',-pi/2,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
L(5) = Link('d',0,'a',0,'alpha',pi/2,'offset',0,'qlim',[deg2rad(-120.3),deg2rad(120.3)]);
L(6) = Link('d',0.1059 + 0.06153 + 0.120,'a',0,'alpha',0,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);

test = SerialLink(L,'name',name);

test.plot(zeros(1, test.n), 'workspace', workspace, 'scale', 0.1);



for linkIndex = 0:test.n
    
        [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['kinova3link',num2str(linkIndex),'.ply'],'tri'); % #ok<AGROW>

    test.faces{linkIndex+1} = faceData;
    test.points{linkIndex+1} = vertexData;
end

% Display robot
test.plot3d(zeros(1,test.n),'noarrow','workspace',workspace);
if isempty(findobj(get(gca,'Children'),'Type','Light'))
    camlight
end
test.delay = 0;

% Try to correctly colour the arm (if colours are in ply file data)
for linkIndex = 0:test.n
    handles = findobj('Tag', test.name);
    h = get(handles,'UserData');
    try
        h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
            , plyData{linkIndex+1}.vertex.green ...
            , plyData{linkIndex+1}.vertex.blue]/255;
        h.link(linkIndex+1).Children.FaceColor = 'interp';
    catch ME_1
        disp(ME_1);
        continue;
    end
end

test.teach;
view(3);