function varargout = ass2_GUI(varargin)
% ASS2_GUI MATLAB code for ass2_GUI.fig
%      ASS2_GUI, by itself, creates a new ASS2_GUI or raises the existing
%      singleton*.
%
%      H = ASS2_GUI returns the handle to a new ASS2_GUI or the handle to
%      the existing singleton*.
%
%      ASS2_GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in ASS2_GUI.M with the given input arguments.
%
%      ASS2_GUI('Property','Value',...) creates a new ASS2_GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before ass2_GUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to ass2_GUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help ass2_GUI

% Last Modified by GUIDE v2.5 21-Oct-2020 03:47:41

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @ass2_GUI_OpeningFcn, ...
    'gui_OutputFcn',  @ass2_GUI_OutputFcn, ...
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

% --- Executes just before ass2_GUI is made visible.
function ass2_GUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to ass2_GUI (see VARARGIN)

% Choose default command line output for ass2_GUI
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
% so window can get raised using ass2_GUI.
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
    t1 = [0:0.5:20]';
    q1 = [0 pi/3 pi/2 0 pi/4 0];
    Trajred = jtraj(q0,q1,t1);
    
    for i = 1:size(Trajred,1)
        model.animate(Trajred(i,:));
        drawnow();
    end
    data = guidata(hObject);
    data.model = model;
    guidata(hObject,data);
    
    
    hold on;
    axis off;
    view(3);  
    
end

% UIWAIT makes ass2_GUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = ass2_GUI_OutputFcn(hObject, eventdata, handles)
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



% popup_sel_index = get(handles.popupmenu1, 'Value');
% switch popup_sel_index
%     case 1
%         plot(rand(5));
%     case 2
%         plot(sin(1:0.01:25.99));
%     case 3
%         bar(1:.5:10);
%     case 4
%         plot(membrane);
%     case 5
%         surf(peaks);
% end


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
f = msgbox('Emergency Stop has been Activated. Close Screen to Reset Safety', 'Error','error');
uiwait(f);


% --- Executes on button press in StartPB.
function StartPB_Callback(hObject, eventdata, handles)
% hObject    handle to StartPB (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
uiresume;


% --- Executes on selection change in ColourSelect.
function ColourSelect_Callback(hObject, eventdata, handles)
% hObject    handle to ColourSelect (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
colour = get(handles.ColourSelect, 'Value');
switch colour
    case 1
        str = input(Red);
    case 2
        str = input(Green);
    case 3
        str = input(Blue);
    case 4
        str = input(Default);
end
% Hints: contents = cellstr(get(hObject,'String')) returns ColourSelect contents as cell array
%        contents{get(hObject,'Value')} returns selected item from ColourSelect


% --- Executes during object creation, after setting all properties.
function ColourSelect_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ColourSelect (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
