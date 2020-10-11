classdef KinovaGen3 < handle
    properties
        %> Robot model
        model;
        
        %>
        workspace = [-1 1 -1 1 0 3];
        
        %> Flag to indicate if gripper is used
        useGripper = false;
    end
    
    methods%% Class for KinovaGen3 robot simulation
        function self = KinovaGen3(useGripper)
            self.useGripper = useGripper;
            
            self.GetKinovaRobot();
            
            self.PlotAndColourRobot();
        end
        
        %% GetUR5Robot
        % Given a name (optional)
        function GetKinovaRobot(self)
            %     if nargin < 1
            % Create a unique name (ms timestamp after 1ms pause)
            pause(0.001);
            name = ['KinovaGen3_',datestr(now,'yyyymmddTHHMMSSFFF')];
            
            % Create the KinovaGen3 6DoF model
            % THETA, D, A, ALPHA
            % ROBOT BASE IS L(0)
            % L(0) = Link('alpha', 0, 'a', 0, 'd', 0.1564, 'offset', -pi/2);
      L(1) = Link('d',0.2848,'a',0,'alpha',-pi/2,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
      L(2) = Link('d',0.0054,'a',0.41,'alpha',-pi,'offset',-pi/2,'qlim',[deg2rad(-90),deg2rad(90)]);
      L(3) = Link('d',0.0064,'a',0,'alpha',-pi/2,'offset',-pi/2,'qlim',[deg2rad(-147.8),deg2rad(147.8)]);
      L(4) = Link('d',0.2084+.1059,'a',0,'alpha',-pi/2,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
      L(5) = Link('d',0,'a',0,'alpha',pi/2,'offset',0,'qlim',[deg2rad(-120.3),deg2rad(120.3)]);
      L(6) = Link('d',0.1059+.06153,'a',0,'alpha',0,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);

            
            % Incorporate offsets
            
            % Incorporate joint limits
            
            self.model = SerialLink(L,'name',name);
            
            % Rotate robot to the correct orientation
            self.model.base = self.model.base * transl(0, 3, 1);
            
        %    self.model.plot(zeros(1, self.model.n), 'workspace', self.workspace, 'scale', 0.1);
        end
%% PlotAndColourRobot
% Given a robot index, add the glyphs (vertices and faces) and
% colour them in if data is available 
function PlotAndColourRobot(self)%robot,workspace)
    for linkIndex = 0:self.model.n
        if self.useGripper && linkIndex == self.model.n
            [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['kinova3link',num2str(linkIndex),'Gripper.ply'],'tri'); %#ok<AGROW>
        else
            [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['kinova3link',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
        end
        self.model.faces{linkIndex+1} = faceData;
        self.model.points{linkIndex+1} = vertexData;
    end

    % Display robot
    self.model.plot3d(zeros(1,self.model.n),'noarrow','workspace',self.workspace);
    if isempty(findobj(get(gca,'Children'),'Type','Light'))
        camlight
    end  
    self.model.delay = 0;

    % Try to correctly colour the arm (if colours are in ply file data)
    for linkIndex = 0:self.model.n
        handles = findobj('Tag', self.model.name);
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
end        
    end
end
