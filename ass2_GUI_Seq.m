function varargout = ass2_GUI_Seq(varargin)
% ASS2_GUI_SEQ MATLAB code for ass2_GUI_Seq.fig
%      ASS2_GUI_SEQ, by itself, creates a new ASS2_GUI_SEQ or raises the existing
%      singleton*.
%
%      H = ASS2_GUI_SEQ returns the handle to a new ASS2_GUI_SEQ or the handle to
%      the existing singleton*.
%
%      ASS2_GUI_SEQ('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in ASS2_GUI_SEQ.M with the given input arguments.
%
%      ASS2_GUI_SEQ('Property','Value',...) creates a new ASS2_GUI_SEQ or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before ass2_GUI_Seq_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to ass2_GUI_Seq_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help ass2_GUI_Seq

% Last Modified by GUIDE v2.5 21-Oct-2020 15:05:55

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @ass2_GUI_Seq_OpeningFcn, ...
    'gui_OutputFcn',  @ass2_GUI_Seq_OutputFcn, ...
    'gui_LayoutFcn',  [] , ...
    'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT

% --- Executes just before ass2_GUI_Seq is made visible.
function ass2_GUI_Seq_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to ass2_GUI_Seq (see VARARGIN)

% Choose default command line output for ass2_GUI_Seq
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% Initialise slider values
set(handles.Q1Val, 'String', get(handles.Q1Slider, 'Value'));
set(handles.Q2Val, 'String', get(handles.Q2Slider, 'Value'));
set(handles.Q3Val, 'String', get(handles.Q3Slider, 'Value'));
set(handles.Q4Val, 'String', get(handles.Q4Slider, 'Value'));
set(handles.Q5Val, 'String', get(handles.Q5Slider, 'Value'));
set(handles.Q6Val, 'String', get(handles.Q6Slider, 'Value'));

% This sets up the initial plot - only do when we are invisible
% so window can get raised using ass2_GUI_Seq.
if strcmp(get(hObject,'Visible'),'off')
    
    guiEnvironment();
    
    L1 = Link('d',0.2848,'a',0,'alpha',-pi/2,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
    L2 = Link('d',0.0054,'a',0.41,'alpha',-pi,'offset',-pi/2,'qlim',[deg2rad(-90),deg2rad(90)]);
    L3 = Link('d',0.0064,'a',0,'alpha',-pi/2,'offset',-pi/2,'qlim',[deg2rad(-147.8),deg2rad(147.8)]);
    L4 = Link('d',0.2084+.1059,'a',0,'alpha',-pi/2,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
    L5 = Link('d',0,'a',0,'alpha',pi/2,'offset',0,'qlim',[deg2rad(-120.3),deg2rad(120.3)]);
    L6 = Link('d',0.1059+.06153,'a',0,'alpha',0,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
    model = SerialLink([L1 L2 L3 L4 L5 L6],'name','kinova3');
    model.base = model.base * transl(0, 0, 1.078);
    
    for linkIndex = 0:model.n
        [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['kinova3link',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
        model.faces{linkIndex+1} = faceData;
        model.points{linkIndex+1} = vertexData;
    end
    
    % Display robot
    workspace = [-.5 .5 -.5 .5 0 1];
    model.plot3d(zeros(1,model.n),'noarrow','workspace',workspace);
    if isempty(findobj(get(gca,'Children'),'Type','Light'))
        camlight
    end
    model.delay = 0;
    
    % Try to correctly colour the arm (if colours are in ply file data)
    for linkIndex = 0:model.n
        handles = findobj('Tag', model.name);
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
    
    q0 = [0 0 0 0 0 0];
    model.animate(q0);
    
    data = guidata(hObject);
    data.model = model;
    guidata(hObject,data);
    
    
    hold on;
    axis off;
    view(3);
    
end

% UIWAIT makes ass2_GUI_Seq wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = ass2_GUI_Seq_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

% --- Executes on button press in Update_PB.
function Update_PB_Callback(hObject, eventdata, handles)
cla;
axes(handles.axes1);


% --------------------------------------------------------------------
function FileMenu_Callback(hObject, eventdata, handles)
% hObject    handle to FileMenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function OpenMenuItem_Callback(hObject, eventdata, handles)
% hObject    handle to OpenMenuItem (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
file = uigetfile('*.fig');
if ~isequal(file, 0)
    open(file);
end

% --------------------------------------------------------------------
function PrintMenuItem_Callback(hObject, eventdata, handles)
% hObject    handle to PrintMenuItem (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
printdlg(handles.figure1)

% --------------------------------------------------------------------
function CloseMenuItem_Callback(hObject, eventdata, handles)
% hObject    handle to CloseMenuItem (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
selection = questdlg(['Close ' get(handles.figure1,'Name') '?'],...
    ['Close ' get(handles.figure1,'Name') '...'],...
    'Yes','No','Yes');
if strcmp(selection,'No')
    return;
end

delete(handles.figure1)


% --- Executes on selection change in popupmenu1.
function popupmenu1_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = get(hObject,'String') returns popupmenu1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu1


% --- Executes during object creation, after setting all properties.
function popupmenu1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

set(hObject, 'String', {'plot(rand(5))', 'plot(sin(1:0.01:25))', 'bar(1:.5:10)', 'plot(membrane)', 'surf(peaks)'});

%% ROBOT TEACH FUNCTION
% --- Executes on button press in Teach_Toggle.
function Teach_Toggle_Callback(hObject, eventdata, handles)
% hObject    handle to Teach_Toggle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.model.teach();
% Hint: get(hObject,'Value') returns toggle state of Teach_Toggle

%% END EFFECTOR INCREMENTING
% --- Executes on button press in Increment_X.
function Increment_X_Callback(hObject, eventdata, handles)
% hObject    handle to Increment_X (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
q = handles.model.getpos;
tr = handles.model.fkine(q);
tr(1,4) = tr(1,4) + 0.01;
newQ = handles.model.ikcon(tr,q);
handles.model.animate(newQ);


% --- Executes on button press in Decrement_X.
function Decrement_X_Callback(hObject, eventdata, handles)
% hObject    handle to Decrement_X (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
q = handles.model.getpos;
tr = handles.model.fkine(q);
tr(1,4) = tr(1,4) - 0.01;
newQ = handles.model.ikcon(tr,q);
handles.model.animate(newQ);


% --- Executes on button press in Increment_Y.
function Increment_Y_Callback(hObject, eventdata, handles)
% hObject    handle to Increment_Y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
q = handles.model.getpos;
tr = handles.model.fkine(q);
tr(2,4) = tr(2,4) + 0.01;
newQ = handles.model.ikcon(tr,q);
handles.model.animate(newQ);


% --- Executes on button press in Decrement_Y.
function Decrement_Y_Callback(hObject, eventdata, handles)
% hObject    handle to Decrement_Y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
q = handles.model.getpos;
tr = handles.model.fkine(q);
tr(2,4) = tr(2,4) - 0.01;
newQ = handles.model.ikcon(tr,q);
handles.model.animate(newQ);


% --- Executes on button press in Increment_Z.
function Increment_Z_Callback(hObject, eventdata, handles)
% hObject    handle to Increment_Z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
q = handles.model.getpos;
tr = handles.model.fkine(q);
tr(3,4) = tr(3,4) + 0.01;
newQ = handles.model.ikcon(tr,q);
handles.model.animate(newQ);


% --- Executes on button press in Decrement_Z.
function Decrement_Z_Callback(hObject, eventdata, handles)
% hObject    handle to Decrement_Z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
q = handles.model.getpos;
tr = handles.model.fkine(q);
tr(3,4) = tr(3,4) - 0.01;
newQ = handles.model.ikcon(tr,q);
handles.model.animate(newQ);

%% JOINT ANGLE SLIDERS

% --- Executes on slider movement.
function Q1Slider_Callback(hObject, eventdata, handles)
% hObject    handle to Q1Slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
q1Val = get(hObject, 'value');
assignin('base', 'q1Val', q1Val);
set(handles.Q1Val, 'String', num2str(q1Val));

q = handles.model.getpos;
q1 = deg2rad(get(handles.Q1Slider, 'value'));
q2 = deg2rad(get(handles.Q2Slider, 'value'));
q3 = deg2rad(get(handles.Q3Slider, 'value'));
q4 = deg2rad(get(handles.Q4Slider, 'value'));
q5 = deg2rad(get(handles.Q5Slider, 'value'));
q6 = deg2rad(get(handles.Q6Slider, 'value'));
handles.model.animate([q1 q2 q3 q4 q5 q6]);

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function Q1Slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Q1Slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function Q2Slider_Callback(hObject, eventdata, handles)
% hObject    handle to Q2Slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
q2Val = get(hObject, 'value');
assignin('base', 'q2Val', q2Val);
set(handles.Q2Val, 'String', num2str(q2Val));

q = handles.model.getpos;
q1 = deg2rad(get(handles.Q1Slider, 'value'));
q2 = deg2rad(get(handles.Q2Slider, 'value'));
q3 = deg2rad(get(handles.Q3Slider, 'value'));
q4 = deg2rad(get(handles.Q4Slider, 'value'));
q5 = deg2rad(get(handles.Q5Slider, 'value'));
q6 = deg2rad(get(handles.Q6Slider, 'value'));
handles.model.animate([q1 q2 q3 q4 q5 q6]);
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function Q2Slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Q2Slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function Q3Slider_Callback(hObject, eventdata, handles)
% hObject    handle to Q3Slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
q3Val = get(hObject, 'value');
assignin('base', 'q3Val', q3Val);
set(handles.Q3Val, 'String', num2str(q3Val));

q = handles.model.getpos;
q1 = deg2rad(get(handles.Q1Slider, 'value'));
q2 = deg2rad(get(handles.Q2Slider, 'value'));
q3 = deg2rad(get(handles.Q3Slider, 'value'));
q4 = deg2rad(get(handles.Q4Slider, 'value'));
q5 = deg2rad(get(handles.Q5Slider, 'value'));
q6 = deg2rad(get(handles.Q6Slider, 'value'));
handles.model.animate([q1 q2 q3 q4 q5 q6]);
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function Q3Slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Q3Slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function Q4Slider_Callback(hObject, eventdata, handles)
% hObject    handle to Q4Slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
q4Val = get(hObject, 'value');
assignin('base', 'q4Val', q4Val);
set(handles.Q4Val, 'String', num2str(q4Val));

q = handles.model.getpos;
q1 = deg2rad(get(handles.Q1Slider, 'value'));
q2 = deg2rad(get(handles.Q2Slider, 'value'));
q3 = deg2rad(get(handles.Q3Slider, 'value'));
q4 = deg2rad(get(handles.Q4Slider, 'value'));
q5 = deg2rad(get(handles.Q5Slider, 'value'));
q6 = deg2rad(get(handles.Q6Slider, 'value'));
handles.model.animate([q1 q2 q3 q4 q5 q6]);
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function Q4Slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Q4Slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function Q5Slider_Callback(hObject, eventdata, handles)
% hObject    handle to Q5Slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
q5Val = get(hObject, 'value');
assignin('base', 'q5Val', q5Val);
set(handles.Q5Val, 'String', num2str(q5Val));

q = handles.model.getpos;
q1 = deg2rad(get(handles.Q1Slider, 'value'));
q2 = deg2rad(get(handles.Q2Slider, 'value'));
q3 = deg2rad(get(handles.Q3Slider, 'value'));
q4 = deg2rad(get(handles.Q4Slider, 'value'));
q5 = deg2rad(get(handles.Q5Slider, 'value'));
q6 = deg2rad(get(handles.Q6Slider, 'value'));
handles.model.animate([q1 q2 q3 q4 q5 q6]);
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function Q5Slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Q5Slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function Q6Slider_Callback(hObject, eventdata, handles)
% hObject    handle to Q6Slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
q6Val = get(hObject, 'value');
assignin('base', 'q6Val', q6Val);
set(handles.Q6Val, 'String', num2str(q6Val));

q = handles.model.getpos;
q1 = deg2rad(get(handles.Q1Slider, 'value'));
q2 = deg2rad(get(handles.Q2Slider, 'value'));
q3 = deg2rad(get(handles.Q3Slider, 'value'));
q4 = deg2rad(get(handles.Q4Slider, 'value'));
q5 = deg2rad(get(handles.Q5Slider, 'value'));
q6 = deg2rad(get(handles.Q6Slider, 'value'));
handles.model.animate([q1 q2 q3 q4 q5 q6]);

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function Q6Slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Q6Slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on button press in StopPB.
function StopPB_Callback(hObject, eventdata, handles)
% hObject    handle to StopPB (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
f = msgbox('Emergency Stop has been Activated. Close Screen to Reset Safety', 'Safety Tripped','error');
pause(60);
delete(f);
uiwait();


% --- Executes on button press in StartPB.
function StartPB_Callback(hObject, eventdata, handles)
% hObject    handle to StartPB (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
r = [-.9 0.2 1.14];
g = [-.9 0 1.14];
b = [-.9 -0.2 1.14];

% Create starting can transforms
redStart = transl(r);
greenStart = transl(g);
blueStart = transl(b);

% Transforms for Robot Use
redPos = transl(r)*troty(pi/2)*trotx(pi/2)*trotz(pi);
greenPos = transl(g)*troty(pi/2)*trotx(pi/2)*trotz(pi);
bluePos = transl(b)*troty(pi/2)*trotx(pi/2)*trotz(pi);

redCanLocation = redCan(redStart);
greenCanLocation = greenCan(greenStart);
blueCanLocation = blueCan(blueStart);

%% Get user to select colour configuration

str = input('Which colour should go on top? (Red, Green, Blue)? ','s');

[redGoalCo, greenGoalCo, blueGoalCo] = selectColour(str);

redGoalPose = transl(redGoalCo)*troty(pi/2)*trotz(pi);
greenGoalPose = transl(greenGoalCo)*troty(pi/2)*trotz(pi);
blueGoalPose = transl(blueGoalCo)*troty(pi/2)*trotz(pi);

%% Boxes
tableC = [0 0 0.55];
tableS = [2 1.5 1.1];

% Shelf1
shelf1C = [0 -.7 1.4];
shelf1S = [0.8 0.2 0.02];
% Shelf2
shelf2C = [0 -.7 1.4255];
shelf2S = [0.8 0.2 0.02];

% Shelf3
shelf3C = [0 -.7 1.6925];
shelf3S = [0.8 0.2 0.02];

% Shelf4
shelf4C = [0 -.7 1.9595];
shelf4S = [0.8 0.2 0.02];

% Shelf5
shelf5C = [0 -.7 2.2265];
shelf5S = [0.8 0.2 0.02];

plotOptions.plotFaces = false;
[vertex1,faces1,Normal1] = RectangularPrism(tableC + tableS/2, tableC - tableS/2,plotOptions);
[vertex2,faces2,Normal2] = RectangularPrism(shelf1C + shelf1S, shelf1C - shelf1S,plotOptions);
[vertex3,faces3,Normal3] = RectangularPrism(shelf2C + shelf2S, shelf2C - shelf2S,plotOptions);
[vertex4,faces4,Normal4] = RectangularPrism(shelf3C + shelf3S, shelf3C - shelf3S,plotOptions);
[vertex5,faces5,Normal5] = RectangularPrism(shelf4C + shelf4S, shelf4C - shelf4S,plotOptions);
[vertex6,faces6,Normal6] = RectangularPrism(shelf5C + shelf4S, shelf5C - shelf5S,plotOptions);
axis equal

r = [-.9 0.2 1.14];
g = [-.9 0 1.14];
b = [-.9 -0.2 1.14];

q0 = [0 0 0 0 0 0];
%ce
Roll = [0, .53, -67.836, -.032, -67.836, 81.57, -67.836 ];
Pitch = [0, -.96, 5.01, -75.57, 5.01, -78.49, 5.01];
Yaw = [0, 2.45, -97.55, -.0249, -97.55, 70.372, -97.55];

current = handles.model.fkine(q0);
handles.model.animate(q0)

positions = [current(1,4),current(2,4),current(3,4);
    r(1),r(2),r(3);       %Red Can
    redGoalCo(1),redGoalCo(2),redGoalCo(3);      %Final Red Can
    g(1),g(2),g(3);       %Green Can
    greenGoalCo(1),greenGoalCo(2),greenGoalCo(3);        %Final Green Can
    b(1),b(2),b(3);      %Blue Can
    blueGoalCo(1),blueGoalCo(2),blueGoalCo(3)];      %Final Blue Can

for k = 1:6
    q0 = handles.model.getpos;
    q1 = handles.model.ikcon(transl(positions(k+1,:)),q0);
    x = positions(k,1);
    y = positions(k,2);
    z = positions(k,3);
    fx = (positions(k+1,1)-positions(k,1))/10;
    fy = (positions(k+1,2)-positions(k,2))/10;
    fz = (positions(k+1,3)-positions(k,3))/10;
    qWaypoints = [q0 ; handles.model.ikcon(transl(x,y,z),q0)];
    [~,all] = handles.model.fkine(qWaypoints(end,:));
    for j = 1:6
        for i = 1 : size(all,3)-1
            for faceIndex1 = 1:size(faces1,1)
                futureWaypoints = [qWaypoints ; handles.model.ikcon(transl(x+fx,y+fy,z+fz),qWaypoints(end,:))];
                [~,newall] = handles.model.fkine(futureWaypoints(end,:));
                vertOnPlane1 = vertex1(faces1(faceIndex1,1)',:);
                vertOnPlane2 = vertex2(faces2(faceIndex1,1)',:);
                vertOnPlane3 = vertex3(faces3(faceIndex1,1)',:);
                vertOnPlane4 = vertex4(faces4(faceIndex1,1)',:);
                vertOnPlane5 = vertex5(faces5(faceIndex1,1)',:);
                vertOnPlane6 = vertex6(faces6(faceIndex1,1)',:);
                [intersects1,check1]=LinePlaneIntersection(Normal1(faceIndex1,:),vertOnPlane1,newall(1:3,4,i)',newall(1:3,4,i+1)');
                [intersects2,check2]=LinePlaneIntersection(Normal2(faceIndex1,:),vertOnPlane2,newall(1:3,4,i)',newall(1:3,4,i+1)');
                [intersects3,check3]=LinePlaneIntersection(Normal3(faceIndex1,:),vertOnPlane3,newall(1:3,4,i)',newall(1:3,4,i+1)');
                [intersects4,check4]=LinePlaneIntersection(Normal4(faceIndex1,:),vertOnPlane4,newall(1:3,4,i)',newall(1:3,4,i+1)');
                [intersects5,check5]=LinePlaneIntersection(Normal5(faceIndex1,:),vertOnPlane5,newall(1:3,4,i)',newall(1:3,4,i+1)');
                [intersects6,check6]=LinePlaneIntersection(Normal6(faceIndex1,:),vertOnPlane6,newall(1:3,4,i)',newall(1:3,4,i+1)');
                
                if (check1 == 1 && IsIntersectionPointInsideTriangle(intersects1,vertex1(faces1(faceIndex1,:)',:))) || (check2 == 1 && IsIntersectionPointInsideTriangle(intersects2,vertex2(faces2(faceIndex1,:)',:))) || (check3 == 1 && IsIntersectionPointInsideTriangle(intersects3,vertex3(faces3(faceIndex1,:)',:)))...
                        || (check4 == 1 && IsIntersectionPointInsideTriangle(intersects4,vertex4(faces4(faceIndex1,:)',:)))|| (check5 == 1 && IsIntersectionPointInsideTriangle(intersects5,vertex5(faces5(faceIndex1,:)',:)))|| (check6 == 1 && IsIntersectionPointInsideTriangle(intersects6,vertex6(faces6(faceIndex1,:)',:)))
                    disp('Avoiding Collision')
                    x = x-.2;
                    y = y+.2;
                    z = z+.2;
                end
            end
        end
        qWaypoints = [qWaypoints; handles.model.ikcon(transl(x,y,z),qWaypoints(end,:))];
        x=x+fx;
        y=y+fy;
        z =z+fz;
        
    end
    qWaypoints = [qWaypoints; handles.model.ikcon(transl(x,y,z),q1)];
    
    % RMRC
    % steps = size(qWaypoints,1);
    steps = 40;
    deltaT = 0.05;
    x0 = zeros(3,steps);
    s = lspb(0,1,steps);
    delta = 2*pi/steps;
    epsilon = 0.1;
    W = diag([1 1 1 0.1 0.1 0.1]);
    m = zeros(steps,1);
    qdot = zeros(steps,6);
    theta = zeros(3,steps);
    positionError = zeros(3,steps);
    angleError = zeros(3,steps);
    x1 = handles.model.fkine(q0);
    x2 = handles.model.fkine(qWaypoints(end,:));
    for i = 1:steps
        x0(:,i) = x1(1:3,4)*(1-s(i)) + s(i)*x2(1:3,4);
        theta(1,i) = Roll(k);
        theta(2,i) = Pitch(k);
        theta(3,i) = Yaw(k);
    end
    
    for i = 1:steps-1
        T = handles.model.fkine(real(qWaypoints(i,:)));
        deltaX = x0(:,i+1) - T(1:3,4);
        Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));
        Ra = T(1:3,1:3);
        Rdot = (1/deltaT)*(Rd - Ra);
        S = Rdot*Ra';
        linear_velocity = (1/deltaT)*deltaX;
        angular_velocity = [S(3,2);S(1,3);S(2,1)];
        xdot = W*[linear_velocity;angular_velocity];
        J = handles.model.jacob0(real(qWaypoints(i,:)));
        m(i) = sqrt(det(J*J'));
        if m(i) < epsilon  % If manipulability is less than given threshold
            lambda = (1 - m(i)/epsilon)*5E-2;
        else
            lambda = 0;
        end
        invJ = inv(J'*J + lambda *eye(6))*J';
        qdot(i,:) = (invJ*xdot)';
        for j = 1:6
            if qWaypoints(i,j) + deltaT*qdot(i,j) < handles.model.qlim(j,1)
                qdot(i,j) = 0; % Stop the motor
            elseif qWaypoints(i,j) + deltaT*qdot(i,j) > handles.model.qlim(j,2)
                qdot(i,j) = 0; % Stop the motor
            end
        end
        qWaypoints(i+1,:) = real(qWaypoints(i,:)) + deltaT*real(qdot(i,:));
        positionError(:,i) = x0(:,i+1) - T(1:3,4);
    end
    for anim=1:steps
        handles.model.animate(qWaypoints(anim,:));
        if k == 2
            redStart = handles.model.fkine(qWaypoints(anim,:));
            delete(redCanLocation);
            redCanLocation = redCan(redStart*troty(pi/2));
        elseif k == 4
            greenStart = handles.model.fkine(qWaypoints(anim,:));
            delete(greenCanLocation);
            greenCanLocation = greenCan(greenStart*troty(pi/2));
        elseif k == 6
            blueStart = handles.model.fkine(qWaypoints(anim,:));
            delete(blueCanLocation);
            blueCanLocation = blueCan(blueStart*troty(pi/2));
        end
        drawnow();
    end
    [~,all] = handles.model.fkine(q1);
    for i = 1 : size(all,3)-1
        for faceIndex1 = 1:size(faces1,1)
            vertOnPlane1 = vertex1(faces1(faceIndex1,1)',:);
            vertOnPlane2 = vertex2(faces2(faceIndex1,1)',:);
            vertOnPlane3 = vertex3(faces3(faceIndex1,1)',:);
            vertOnPlane4 = vertex4(faces4(faceIndex1,1)',:);
            vertOnPlane5 = vertex5(faces5(faceIndex1,1)',:);
            vertOnPlane6 = vertex6(faces6(faceIndex1,1)',:);
            [intersects1,check1]=LinePlaneIntersection(Normal1(faceIndex1,:),vertOnPlane1,newall(1:3,4,i)',newall(1:3,4,i+1)');
            [intersects2,check2]=LinePlaneIntersection(Normal2(faceIndex1,:),vertOnPlane2,newall(1:3,4,i)',newall(1:3,4,i+1)');
            [intersects3,check3]=LinePlaneIntersection(Normal3(faceIndex1,:),vertOnPlane3,newall(1:3,4,i)',newall(1:3,4,i+1)');
            [intersects4,check4]=LinePlaneIntersection(Normal4(faceIndex1,:),vertOnPlane4,newall(1:3,4,i)',newall(1:3,4,i+1)');
            [intersects5,check5]=LinePlaneIntersection(Normal5(faceIndex1,:),vertOnPlane5,newall(1:3,4,i)',newall(1:3,4,i+1)');
            [intersects6,check6]=LinePlaneIntersection(Normal6(faceIndex1,:),vertOnPlane6,newall(1:3,4,i)',newall(1:3,4,i+1)');
            
            if (check1 == 1 && IsIntersectionPointInsideTriangle(intersects1,vertex1(faces1(faceIndex1,:)',:))) || (check2 == 1 && IsIntersectionPointInsideTriangle(intersects2,vertex2(faces2(faceIndex1,:)',:))) || (check3 == 1 && IsIntersectionPointInsideTriangle(intersects3,vertex3(faces3(faceIndex1,:)',:)))...
                    || (check4 == 1 && IsIntersectionPointInsideTriangle(intersects4,vertex4(faces4(faceIndex1,:)',:)))|| (check5 == 1 && IsIntersectionPointInsideTriangle(intersects5,vertex5(faces5(faceIndex1,:)',:)))|| (check6 == 1 && IsIntersectionPointInsideTriangle(intersects6,vertex6(faces6(faceIndex1,:)',:)))
                disp('Cannot get to final destination, please clear the area')
                %uiwait
            end
        end
    end
    disp('Moving to final');
    qWaypoints = [qWaypoints(end,:); q1];
    
    %  RMRC To final Position
    
    q0 = handles.model.getpos;
    x1 = handles.model.fkine(q0);
    x2 = handles.model.fkine(q1);
    for i = 1:steps
        x0(:,i) = x1(1:3,4)*(1-s(i)) + s(i)*x2(1:3,4);
        theta(1,i) = Roll(k+1);
        theta(2,i) = Pitch(k+1);
        theta(3,i) = Yaw(k+1);
    end
    
    for i = 1:steps-1
        T = handles.model.fkine(real(qWaypoints(i,:)));
        deltaX = x0(:,i+1) - T(1:3,4);
        Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));
        Ra = T(1:3,1:3);
        Rdot = (1/deltaT)*(Rd - Ra);
        S = Rdot*Ra';
        linear_velocity = (1/deltaT)*deltaX;
        angular_velocity = [S(3,2);S(1,3);S(2,1)];
        xdot = W*[linear_velocity;angular_velocity];
        J = handles.model.jacob0(real(qWaypoints(i,:)));
        m(i) = sqrt(det(J*J'));
        if m(i) < epsilon
            lambda = (1 - m(i)/epsilon)*5E-2;
        else
            lambda = 0;
        end
        invJ = inv(J'*J + lambda *eye(6))*J';
        qdot(i,:) = (invJ*xdot)';
        for j = 1:6
            if qWaypoints(i,j) + deltaT*qdot(i,j) < handles.model.qlim(j,1)
                qdot(i,j) = 0; % Stop the motor
            elseif qWaypoints(i,j) + deltaT*qdot(i,j) > handles.model.qlim(j,2)
                qdot(i,j) = 0; % Stop the motor
            end
        end
        qWaypoints(i+1,:) = real(qWaypoints(i,:)) + deltaT*real(qdot(i,:));
        positionError(:,i) = x0(:,i+1) - T(1:3,4);
    end
    for anim = 1:steps
        handles.model.animate(qWaypoints(anim,:));
        if k == 2
            redStart = handles.model.fkine(qWaypoints(anim,:));
            delete(redCanLocation);
            redCanLocation = redCan(redStart*troty(pi/2));
        elseif k == 4
            greenStart = handles.model.fkine(qWaypoints(anim,:));
            delete(greenCanLocation);
            greenCanLocation = greenCan(greenStart*troty(pi/2));
        elseif k == 6
            blueStart = handles.model.fkine(qWaypoints(anim,:));
            delete(blueCanLocation);
            blueCanLocation = blueCan(blueStart*troty(pi/2));
        end
        drawnow();
    end
end

%% Retreat from symbol

pStar = [ 662 362 362 662; 362 362 662 662];


P=[-1.2,-1.2,-1.2,-1.2;
    -0.25,0.25,0.25,-0.25;
    2,2,1.5,1.5];

q0 = [0; 0; pi/2; pi/10; 0; pi/4];

cam = CentralCamera('focal', 0.08, 'pixel', 10e-5, ...
    'resolution', [1024 1024], 'centre', [512 512],'name','kinova');

fps = 25;
lambda = 0.6;
depth = mean (P(1,:));
Tc0= handles.model.fkine(q0);
handles.model.animate(q0');
drawnow
cam.T = Tc0;
% cam.plot_camera('Tcam',Tc0, 'label','scale',0.05);
% plot_sphere(P, 0.05, 'b')
lighting gouraud
light
p = cam.plot(P, 'Tcam', Tc0);
cam.clf()
cam.plot(pStar, '*');
cam.hold(true);
cam.plot(P, 'Tcam', Tc0, 'o');
pause(2)
cam.hold(true);
cam.plot(P);
vel_p = [];
uv_p = [];
history = [];
ksteps = 0;
for i=1:10
    ksteps = ksteps + 1;
    
    % compute the view of the camera
    uv = cam.plot(P);
    
    % compute image plane error as a column
    e = pStar-uv;   % feature error
    e = e(:);
    Zest = [];
    
    % compute the Jacobian
    if isempty(depth)
        % exact depth from simulation
        pt = homtrans(inv(Tcam), P);
        J = cam.visjac_p(uv, pt(3,:) );
    elseif ~isempty(Zest)
        J = cam.visjac_p(uv, Zest);
    else
        J = cam.visjac_p(uv, depth );
    end
    
    % compute the velocity of camera in camera frame
    try
        v = lambda * pinv(J) * e;
    catch
        status = -1;
        return
    end
    fprintf('v: %.3f %.3f %.3f %.3f %.3f %.3f\n', v);
    
    %compute robot's Jacobian and inverse
    J2 = handles.model.jacobn(q0);
    Jinv = pinv(J2);
    % get joint velocities
    qp = Jinv*v;
    
    
    %Maximum angular velocity cannot exceed 180 degrees/s
    ind=find(qp>pi);
    if ~isempty(ind)
        qp(ind)=pi;
    end
    ind=find(qp<-pi);
    if ~isempty(ind)
        qp(ind)=-pi;
    end
    
    %Update joints
    q = q0 - (1/fps)*qp;
    deltaT = 0.05;
    s = lspb(0,1,steps);
    qMatrix = nan(steps,6);
    for wtg = 1:steps
        qMatrix(i,:) = (1-s(wtg))*q0+s(wtg)*qp;
    end
    handles.model.animate(q');
    
    %Get camera location
    Tc = handles.model.fkine(q);
    cam.T = Tc;
    
    drawnow
    
    % update the history variables
    hist.uv = uv(:);
    vel = v;
    hist.vel = vel;
    hist.e = e;
    hist.en = norm(e);
    hist.jcond = cond(J);
    hist.Tcam = Tc;
    hist.vel_p = vel;
    hist.uv_p = uv;
    hist.qp = qp;
    hist.q = q;
    
    history = [history hist];
    
    pause(1/fps)
    
    if ~isempty(200) && (ksteps > 200)
        break;
    end
    
    %update current joint position
    q0 = q;
end

% --- Executes on button press in ResumePB.
function ResumePB_Callback(hObject, eventdata, handles)
% hObject    handle to ResumePB (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
uiresume;
