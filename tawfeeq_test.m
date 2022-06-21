function varargout = tawfeeq_test(varargin)
% Begin initialization code
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @tawfeeq_test_OpeningFcn, ...
                   'gui_OutputFcn',  @tawfeeq_test_OutputFcn, ...
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
% End initialization code 


% --- Executes just before tawfeeq_test is made visible.
function tawfeeq_test_OpeningFcn(hObject, ~, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   unrecognized PropertyName/PropertyValue pairs from the
%            command line (see VARARGIN)

% Choose default command line output for tawfeeq_test
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes tawfeeq_test wait for user response (see UIRESUME)
% uiwait(handles.figure1);
buat_axes = axes('unit', 'normalized', 'position', [0 0  1 1]);
background = imread('bg5.jpg');
imagesc(background);
set(buat_axes, 'handlevisibility', 'off', 'visible', 'off')

% --- Outputs from this function are returned to the command line.
function varargout = tawfeeq_test_OutputFcn( ~, ~, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(~ , ~, handles)
a=getappdata(0,'a');
gray_img=rgb2gray(a);
setappdata(0,'filename', gray_img);
axes(handles.axes2);
imshow(gray_img);

% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(~, ~, handles)     %Black and white
a=getappdata(0,'a');
bw_img=im2bw(a,.46);
axes(handles.axes2);
imshow(bw_img);
setappdata(0,'filename',bw_img);
b=bw_img;

% --- Executes on button press in pushbutton5.
function pushbutton5_Callback(hObject, eventdata, handles)    %Reset
a=getappdata(0,'a');
imshow(a);


% --- Executes on button press in pushbutton6.
function pushbutton6_Callback(hObject, eventdata, handles)    %exit
msgbox('Thanks for using my image processing tool by batch');
pause(1);
close();
close();

% --- Executes on button press in pushbutton9.
function pushbutton9_Callback(hObject, eventdata, handles)
length=imdistline();
msgbox('Measured in Pixels');
distance = getDistance(length);

% --- Executes on button press in pushbutton12.
function pushbutton12_Callback(hObject, eventdata, handles)
I=getappdata(0,'a');
I2=flipdim(I,2);
axes(handles.axes2);
imshow(I2);


% --- Executes on button press in pushbutton13.
function pushbutton13_Callback(hObject, eventdata, handles)
a=getappdata(0,'a');
IM2=imcomplement(a);
axes(handles.axes2);
imshow(IM2);

% --- Executes on button press in pushbutton10.
function pushbutton10_Callback(hObject, eventdata, handles)
a=getappdata(0,'a');
red=a;
red(:,:,2:3)=0;
setappdata(0,'filename', red);
axes(handles.axes2);
imshow(red);

% --- Executes on button press in pushbutton14.
function pushbutton14_Callback(hObject, eventdata, handles)
a=getappdata(0,'a');
green=a;
green(:,:,1)=0;
green(:,:,3)=0;
setappdata(0,'filename', green);
axes(handles.axes2);
imshow(green);


% --- Executes on button press in pushbutton15.
function pushbutton15_Callback(hObject, eventdata, handles)
a=getappdata(0,'a');
blue=a;
blue(:,:,1)=0;
blue(:,:,2)=0;
setappdata(0,'filename', blue);
axes(handles.axes2);
imshow(blue);


% --- Executes on button press in pushbutton16.
function pushbutton16_Callback(hObject, eventdata, handles)
a=getappdata(0,'a');
noise=imnoise(a, 'salt & pepper');
axes(handles.axes2);
imshow(noise);

% --- Executes on button press in pushbutton17.
function pushbutton17_Callback(hObject, eventdata, handles)
a=getappdata(0,'a');
noise=imnoise(a, 'gaussian');
axes(handles.axes2);
imshow(noise);


% --- Executes on button press in pushbutton18.
function pushbutton18_Callback(~, eventdata, handles)
I=getappdata(0,'a');
I3=flip(I,1);
axes(handles.axes2);
imshow(I3);


% --- Executes on button press in pushbutton19.
function pushbutton19_Callback(hObject, eventdata, handles)
I=getappdata(0,'a');
I2=flipdim(I,2);
I3=flipdim(I,1);
I4=flipdim(I3,2);
axes(handles.axes2);
imshow(I4);

% --- Executes on button press in pushbutton20.
function pushbutton20_Callback(hObject, eventdata, handles)
I=getappdata(0,'a');
I=rgb2gray(I);
BW2=edge(I,'canny');
axes(handles.axes2);
imshow(BW2);

% --- Executes on button press in pushbutton46                  %bluring
function pushbutton46_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton46 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
a=getappdata(0, 'a');
radius = 10;
j= fspecial('disk',radius);
k = imfilter(a,j);
imshow(k);
% --- Executes on button press in pushbutton21.
function pushbutton21_Callback(hObject, eventdata, handles)
I=getappdata(0,'a');
I=rgb2gray(I);
BW1=edge(I,'sobel');
axes(handles.axes2);
imshow(BW1);

% --- Executes on button press in pushbutton22.


% --- Executes on button press in pushbutton38.
function pushbutton38_Callback(hObject, eventdata, handles)
a=getappdata(0,'a');
input=a;
input=rgb2gray(input);
axes(handles.axes2);
imhist(input);


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
%a=uigetfile();
a=uigetfile({'.jpg';'.bmp'},'File Selector');
filename=a;
setappdata(0,'filename',filename);
a=imread(a);
axes(handles.axes1);
imshow(a);
setappdata(0,'a',a);
setappdata(0,'filename',a);
%plot(handles.axes1,'a');

% --- Executes on button press in pushbutton43.
function pushbutton43_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton43 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global image 

counter = 1;  %initialize filename increment
vid = videoinput('winvideo',1);
set(vid, 'ReturnedColorSpace', 'RGB');
image = getsnapshot(vid);
axes(handles.axes1);
imshow(image)
savename = strcat('C:\matlab\1602-20-735-043\Face capturing' ,num2str(counter), '.jpg'); %this is where and what your image will be saved
imwrite(image, savename);
counter = counter +1;
a=image;
setappdata(0,'a',a);
%plot(handles.axes1,'a');



function pushbutton22_Callback(hObject, eventdata, handles)
a=getappdata(0, 'a');
rotate=imrotate(a,45);
axes(handles.axes2);
imshow(rotate);

% --- Executes on button press in pushbutton23.
function pushbutton23_Callback(hObject, eventdata, handles)
a=getappdata(0, 'a');
rotate=imrotate(a,90);
axes(handles.axes2);
imshow(rotate);


% --- Executes on button press in pushbutton24.
function pushbutton24_Callback(hObject, eventdata, handles)
a=getappdata(0, 'a');
rotate=imrotate(a,135);
axes(handles.axes2);
imshow(rotate);


% --- Executes on button press in pushbutton25.
function pushbutton25_Callback(hObject, eventdata, handles)
a=getappdata(0, 'a');
rotate=imrotate(a,360);
axes(handles.axes2);
imshow(rotate);


% --- Executes on button press in pushbutton26.
function pushbutton26_Callback(hObject, eventdata, handles)
a=getappdata(0, 'a');
rotate=imrotate(a,225);
axes(handles.axes2);
imshow(rotate);



% --- Executes on button press in pushbutton27.
function pushbutton27_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton27 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
a=getappdata(0, 'a');
rotate=imrotate(a,270);
axes(handles.axes2);
imshow(rotate);


% --- Executes on button press in pushbutton28.
function pushbutton28_Callback(hObject, eventdata, handles)
a=getappdata(0, 'a');
rotate=imrotate(a,315);
axes(handles.axes2);
imshow(rotate);


% --- Executes on button press in pushbutton29.
function pushbutton29_Callback(hObject, eventdata, handles)
a=getappdata(0, 'a');
rotate=imrotate(a,180);
axes(handles.axes2);
imshow(rotate);



function edit1_Callback(hObject, eventdata, handles)

function edit1_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on button press in pushbutton42.
function pushbutton41_Callback(hObject, eventdata, handles)

% --- Executes on button press in pushbutton47.
function pushbutton47_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton47 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
a=getappdata(0, 'a');
%Invert the image.
AInv = imcomplement(a);
%Apply the dehazing algorithm.
BInv = imreducehaze(AInv,'ContrastEnhancement','none');
%Invert the results.
enhancement = imcomplement(BInv);
%Display the original image and the enhanced images, side-by-side.
axes(handles.axes2);
imshow(enhancement);

% --- Executes on button press in pushbutton48.
function pushbutton48_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton48 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
a=getappdata(0, 'a');
AInv = imcomplement(a);

BInv = imreducehaze(AInv, 'Method','approx','ContrastEnhancement','boost');
BImp = imcomplement(BInv);
axes(handles.axes2);
imshow(BImp);


% --- Executes on button press in pushbutton49.
function pushbutton49_Callback(hObject, eventdata, ~)
% hObject    handle to pushbutton49 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
imsave


% --- Executes on button press in pushbutton64.
function pushbutton64_Callback(~, ~, ~)
% hObject    handle to pushbutton64 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
clear all;
cam=webcam();
cam.Resolution='320x240';
video_frame=snapshot(cam);

video_player = vision.VideoPlayer('Position', [100 100 320 240]);
face_Detector=vision.CascadeObjectDetector();
point_tracker=vision.PointTracker('MaxBidirectionalError',2);

run_loop=true;
while run_loop
    video_frame=snapshot(cam);
    gray_frame=rgb2gray(video_frame);
    
       face_rectangle=face_Detector.step(gray_frame);
        if face_rectangle
          rectangle=bbox2points(face_rectangle(1,:));
        end
        
    video_frame = insertShape(video_frame,'Polygon',rectangle,'LineWidth',3);
    step(video_player, video_frame);
    run_loop = isOpen(video_player);
end 
clear cam;
release(video_player);
release(point_tracker);
release(face_Detector)