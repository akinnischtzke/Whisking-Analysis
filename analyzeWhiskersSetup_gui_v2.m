function varargout = analyzeWhiskersSetup_gui_v2(varargin)
%ANALYZEWHISKERSSETUP_GUI_V2 MATLAB code file for analyzeWhiskersSetup_gui_v2.fig
%      ANALYZEWHISKERSSETUP_GUI_V2, by itself, creates a new ANALYZEWHISKERSSETUP_GUI_V2 or raises the existing
%      singleton*.
%
%      H = ANALYZEWHISKERSSETUP_GUI_V2 returns the handle to a new ANALYZEWHISKERSSETUP_GUI_V2 or the handle to
%      the existing singleton*.
%
%      ANALYZEWHISKERSSETUP_GUI_V2('Property','Value',...) creates a new ANALYZEWHISKERSSETUP_GUI_V2 using the
%      given property value pairs. Unrecognized properties are passed via
%      varargin to analyzeWhiskersSetup_gui_v2_OpeningFcn.  This calling syntax produces a
%      warning when there is an existing singleton*.
%
%      ANALYZEWHISKERSSETUP_GUI_V2('CALLBACK') and ANALYZEWHISKERSSETUP_GUI_V2('CALLBACK',hObject,...) call the
%      local function named CALLBACK in ANALYZEWHISKERSSETUP_GUI_V2.M with the given input
%      arguments.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help analyzeWhiskersSetup_gui_v2

% Last Modified by GUIDE v2.5 17-Jan-2018 13:18:25

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @analyzeWhiskersSetup_gui_v2_OpeningFcn, ...
    'gui_OutputFcn',  @analyzeWhiskersSetup_gui_v2_OutputFcn, ...
    'gui_LayoutFcn',  [], ...
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


% --- Executes just before analyzeWhiskersSetup_gui_v2 is made visible.
function analyzeWhiskersSetup_gui_v2_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   unrecognized PropertyName/PropertyValue pairs from the
%            command line (see VARARGIN)

% Choose default command line output for analyzeWhiskersSetup_gui_v2
handles.output = hObject;
handles.runTrace = 1;
% Update handles structure
guidata(hObject, handles);

% UIWAIT makes analyzeWhiskersSetup_gui_v2 wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = analyzeWhiskersSetup_gui_v2_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on selection change in popupmenu1.
function popupmenu1_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu1 contents as cell array
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

m = dir('*.mp4');
if ~isempty(m)
    fnames = {m(:).name};
%    fnames = {};
%    a=1;
%     for i = 1:size(m,1)
%         fnames{a} = m(i).name; a=a+1;
%         end
%     end
%     if a == 1
%         fnames = {'No whisking videos'};
%     end
else
    fnames = {'No whisking videos'};
end
handles.fnamelist = fnames;
guidata(hObject, handles);

set(hObject, 'String', fnames);



% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if isfield(handles,'vidobj')
    handles.vidobj = [];
end
% if isfield(handles,'measurements')
%     handles.measurements = [];
% end

axes(handles.axes1);
cla;
set(handles.text6, 'String', ' '); drawnow 
set(handles.text7, 'String', ' '); drawnow 

popup_sel_index = get(handles.popupmenu1, 'Value');
f = handles.fnamelist;
filename = cell2mat(f(popup_sel_index));

set(handles.text6, 'String', 'Loading video file...'); drawnow 

obj = VideoReader(filename); %create video object for selected filename
handles.vidobj = obj;
set(handles.text6, 'String', 'Video file loaded'); drawnow 

%Loading measurements data. Here, will only save 1/4 of the measurements
%inforamation to speed things up. Will have to reload the whole thing at
%the end, but i think that's better.

if ~exist('measurements','var') || ~isfield(handles,'measurements')
    set(handles.text7, 'String', 'Loading measurements file...'); drawnow 
    measurements = LoadMeasurements([filename(1:end-4), '.measurements']);
    handles.measurements = measurements;
    clear measurements
end

set(handles.text7, 'String', 'Measurements loaded'); drawnow 

%Read video frame and get coordinates for eye ROI
im = rgb2gray(readFrame(obj));
imshow(im), axis on, axis square

handles.im = im;
handles.filename = filename(1:end-4);

handles.nFrames = floor(obj.FrameRate*obj.Duration);
set(handles.text17, 'String', sprintf('%s','out of ',num2str(handles.nFrames),' frames')); drawnow 

guidata(hObject, handles);


% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

axes(handles.axes1), cla, axis
image(handles.im)
hold on

disp('Upper right XY boundary for whisker quadrant: ');
[xThresh1,yThresh1] = ginput(1);
disp('Lower left XY boundary for whisker quadrant: ');
[xThresh2,yThresh2] = ginput(1);
%Display first frame with chosen thresholds
w = xThresh1 - xThresh2;
h = yThresh2 - yThresh1;
r = rectangle('Position',[xThresh2 yThresh1 w h],'EdgeColor','b');

handles.r = r.Position;
handles.xThresh1 = xThresh1;
handles.xThresh2 = xThresh2;
handles.yThresh1 = yThresh1;
handles.yThresh2 = yThresh2;

guidata(hObject, handles);


% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

r = handles.r;

axes(handles.axes1), cla, axis
image(handles.im)
hold on
rectangle('Position',r,'EdgeColor','b')

disp('Enter [x y] coordinates for front (near the nose) face edge (e.g. [118,145]): ');
fstart = ginput(1);
disp('Enter [x y] coordinates for back face edge (e.g. [147,183]): ');
fend = ginput(1);

%Display line for edge of face
if (fend(1) - fstart(1)) > (fend(2) - fstart(2))
    dx = 1;
    dy = (fend(2) - fstart(2))/(fend(1) - fstart(1));
else
    dx = (fend(1) - fstart(1))/(fend(2) - fstart(2));
    dy = 1;
end

faceEdgeX = fstart(1):dx:fend(1);
faceEdgeY = fstart(2):dy:fend(2);
faceAngle = abs(rad2deg(atan((fend(1) - fstart(1))/(fend(2) - fstart(2)))));

plot(faceEdgeX,faceEdgeY,'.y')

handles.faceEdgeX = faceEdgeX;
handles.faceEdgeY = faceEdgeY;
handles.faceAngle = faceAngle;

guidata(hObject, handles);



function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double

handles.minDistance = str2double(get(hObject,'String'));
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit4_Callback(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit4 as text
%        str2double(get(hObject,'String')) returns contents of edit4 as a double
handles.minLength = str2double(get(hObject,'String'));
guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function edit4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit2_Callback(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit2 as text
%        str2double(get(hObject,'String')) returns contents of edit2 as a double


% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function edit3_Callback(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit3 as text
%        str2double(get(hObject,'String')) returns contents of edit3 as a double
handles.testFrames = str2double(get(hObject,'String'));
handles.runTrace = 1;
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function edit3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



% --- Executes on button press in pushbutton6.
function pushbutton6_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles = guidata(hObject);

%Put in default values to try first
if ~isfield(handles,'minLength'), handles.minLength = 20; end
if ~isfield(handles,'minDistance'), handles.minDistance = 40; end
if ~isfield(handles,'currFrames'), handles.currFrames = 1; end
if ~isfield(handles,'mIndex'),handles.mIndex = 1; end
    
parameters = struct('xThresh1',handles.xThresh1,...
    'yThresh1',handles.yThresh1,...
    'xThresh2',handles.xThresh2,...
    'yThresh2',handles.yThresh2,...
    'faceEdgeX',handles.faceEdgeX,...
    'faceEdgeY',handles.faceEdgeY,...
    'faceAngle',handles.faceAngle,... 
    'framesPlot',handles.nFrames,...
    'minLength',handles.minLength,...
    'minDistance',handles.minDistance,...
    'medAngleLast',[]);

vidobj = handles.vidobj;
nFrames = vidobj.Duration*vidobj.FrameRate;

if ~isfield(handles,'checkbox1Status'), handles.checkbox1Status = 0; end
disp(handles.checkbox1Status)


if isequal(handles.checkbox1Status,1)
    currFrames = handles.startFrame;
    vidobj.CurrentTime = currFrames/vidobj.FrameRate;
    ind = [];%currFrames-1;
    disp('Starting over...')
else
    currFrames = vidobj.CurrentTime*vidobj.FrameRate;
    ind = [];%handles.mIndex;
    disp('Continuing on...')
end

if ~isfield(handles,'testFrames') %If starting with default values, have to set the values first
    handles.testFrames = 100;
    %guidata(hObject, handles);
end

stopStatus = 0;
handles.stopPressed = 0;

p1 = handles.axes1;
p3 = handles.axes3;
set(p3,'YDir','reverse')
title(p3,'Current frame - previous frame'), colorbar(p3)
%if isfield(handles,'ax1'),delete(handles.ax1),end

ax1 = handles.axes4;%axes('Position',[0.3936 0.0209 0.5342 0.2335]);
axes(ax1),yyaxis left,ax1.YColor = 'g'; ylabel('median angle')
yyaxis right,ax1.YColor = 'b'; ylabel('sum frame diff')

plotAngle = nan(1,500);
plotsumDiff = nan(1,500);

currFrame = handles.im;
measurements = handles.measurements;

handles.ax1 = ax1;
handles.measurements = [];
guidata(hObject, handles);

while (stopStatus == 0) && hasFrame(vidobj)%(currFrames <= handles.testFrames)
    
    %Get the current frame
    im = readFrame(vidobj);
    
    %Get all traced objects in the current frame
    [indList,ind] = getInds(currFrames-1,ind,measurements);
    
    %Goes through each object in this frame for analysis; returns median
    %angle per frame & coordinates of objects for plotting
    
    [medAngle,isWhisker,fX,fY,tX,tY] = checkWhiskerObjs(indList,parameters,measurements);
    
    %Plot frame output (at least for first few hundred frames)
    plotOutput(im,parameters,isWhisker,fX,fY,tX,tY,currFrames,nFrames,handles.axes1)
    hold off
    
    %Plot differential frame method
    nextFrame = rgb2gray(im);
    temp = (nextFrame-currFrame);
    d = temp;
    d2 = reshape(d,[numel(d),1]);
    sumDiff = sum(d2);
    plotsumDiff = [plotsumDiff,sumDiff]; 
    plotsumDiff(1) = [];
    image(p3,temp)%,set(p3,'YDir','reverse'),% axis square, axis on
    
    %Plot median angle per frame & object motion output
    plotAngle = [plotAngle medAngle];
    plotAngle(1) = [];
    frameList = currFrames-500+1:currFrames;
    ax1.XLim = [frameList(1) frameList(end)];
    yyaxis left, line(frameList,plotAngle,'Color','g','LineWidth',2)%,ylabel('median angle')
    yyaxis right,line(frameList,plotsumDiff,'Color','b','LineWidth',2)%,ylabel('sum frame diff')
 
    currFrames = currFrames+1;
    currFrame = nextFrame;
    parameters.medAngleLast = medAngle;

    handles = guidata(hObject);
    stopStatus = handles.stopPressed;
    if stopStatus == 1
        break;
    end
    
end
handles.measurements = measurements;
handles.mIndex = ind;
guidata(hObject, handles);

% --- Executes on button press in pushbutton5.
function pushbutton5_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

filename = handles.filename;
parameters = struct('xThresh1',handles.xThresh1,...
    'yThresh1',handles.yThresh1,...
    'xThresh2',handles.xThresh2,...
    'yThresh2',handles.yThresh2,...
    'faceEdgeX',handles.faceEdgeX,...
    'faceEdgeY',handles.faceEdgeY,...
    'faceAngle',handles.faceAngle,... 
    'framesPlot',handles.nFrames,...
    'minLength',handles.minLength,...
    'minDistance',handles.minDistance,...
    'medAngleLast',[]);

save([filename '.mat'],'parameters')
disp('Parameters saved!')

% --- Executes on button press in pushbutton7.
function pushbutton7_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton8.
function pushbutton8_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

filename = handles.filename;
load(filename)
traceAnalysis_new(filename,parameters,1);%,handles.vidobj,handles.measurements) %Use filename
drawnow

%Differential frame method
disp('Analyzing differental frame method...')
whiskingMotion = whiskingAnalysis_objMotion(handles.vidobj);

save(filename,'whiskingMotion','-append')

% --- Executes during object creation, after setting all properties.
function text6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to text6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


textLabel = {' '};
set(hObject, 'String', textLabel);

% handles.text6 = textLabel;
% guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function text7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to text7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

textLabel = {' '};
set(hObject, 'String', textLabel);

% --- Executes on button press in pushbutton9.
function pushbutton9_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

newDir = uigetdir();
cd(newDir)
m = dir('*.mp4');
if ~isempty(m)
    fnames = {m(:).name};
else
    fnames = {'No videos'};
end

handles.currDir = newDir;
handles.fnamelist = fnames;
guidata(hObject, handles);

set(handles.text10,'String',newDir)
set(handles.popupmenu1,'String',fnames);



% --- Executes during object creation, after setting all properties.
function text10_CreateFcn(hObject, eventdata, handles)
% hObject    handle to text10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

handles.currDir = cd;
textLabel = {cd};
set(hObject, 'String', textLabel);
guidata(hObject, handles);


% --- Executes on button press in pushbutton10.
function pushbutton10_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%pause(1), disp('stop button pressed')
handles.stopPressed = 1;
guidata(hObject, handles);


% --- Executes on button press in checkbox1.
function checkbox1_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox1

handles.checkbox1Status = get(hObject,'Value');

vidobj = handles.vidobj;

if isequal(handles.checkbox1Status,1)
    vidobj.CurrentTime = 0;
    handles.vidobj = vidobj;
    handles.currFrames = 1;
else
    handles.currFrames = vidobj.CurrentTime*vidobj.FrameRate;
end


guidata(hObject, handles);



function edit7_Callback(hObject, eventdata, handles)
% hObject    handle to edit7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit7 as text
%        str2double(get(hObject,'String')) returns contents of edit7 as a double

handles.startFrame = str2double(get(hObject,'String'));
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function edit7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

handles.startFrame = str2double(get(hObject,'String'));
guidata(hObject, handles);


% --- Executes on button press in pushbutton11.
function pushbutton11_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

vidobj =  handles.vidobj;
vidobj.CurrentTime = handles.startFrame/vidobj.FrameRate;
handles.vidobj = vidobj;
guidata(hObject, handles);
