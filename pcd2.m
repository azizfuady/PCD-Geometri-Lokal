function varargout = pcd2(varargin)
% PCD2 MATLAB code for pcd2.fig
%      PCD2, by itself, creates a new PCD2 or raises the existing
%      singleton*.
%
%      H = PCD2 returns the handle to a new PCD2 or the handle to
%      the existing singleton*.
%
%      PCD2('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in PCD2.M with the given input arguments.
%
%      PCD2('Property','Value',...) creates a new PCD2 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before pcd2_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to pcd2_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help pcd2

% Last Modified by GUIDE v2.5 27-Apr-2019 20:13:52

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @pcd2_OpeningFcn, ...
                   'gui_OutputFcn',  @pcd2_OutputFcn, ...
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


% --- Executes just before pcd2 is made visible.
function pcd2_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to pcd2 (see VARARGIN)

% Choose default command line output for pcd2
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes pcd2 wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = pcd2_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

% BUKA GAMBAR
% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[nama_file,nama_path] = uigetfile({'*.jpg';'*.png';},...
    'Buka Gambar');
if ~isequal (nama_file,0)
    handles.data1 = imread(fullfile(nama_path,nama_file));
    guidata(hObject,handles);
    axes(handles.axes1);
    imshow(handles.data1);
    title('Gambar Asli');
else
    return
end


% PUTAR KANAN 90
% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
img = handles.data1;

[rowsi,colsi,z]= size(img); 
angle=270;
rads=2*pi*angle/360;  

rowsf=ceil(rowsi*abs(cos(rads))+colsi*abs(sin(rads)));                      
colsf=ceil(rowsi*abs(sin(rads))+colsi*abs(cos(rads)));                     

C=uint8(zeros([rowsf colsf 3 ]));

xo=ceil(rowsi/2);                                                            
yo=ceil(colsi/2);

midx=ceil((size(C,1))/2);
midy=ceil((size(C,2))/2);

for i=1:size(C,1)
    for j=1:size(C,2)                                                       

         x= (i-midx)*cos(rads)+(j-midy)*sin(rads);                                       
         y= -(i-midx)*sin(rads)+(j-midy)*cos(rads);                             
         x=round(x)+xo;
         y=round(y)+yo;

         if (x>=1 && y>=1 && x<=size(img,1) &&  y<=size(img,2) ) 
              C(i,j,:)=img(x,y,:);  
         end

    end
end

axes(handles.axes2);
imshow(C);
title('Hasil Rotasi 90 derajat CW')
handles.data2 = C;
guidata(hObject,handles);


% PUTAR KIRI 90
% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
img = handles.data1;

[rowsi,colsi,z]= size(img); 
angle=90;
rads=2*pi*angle/360;  

rowsf=ceil(rowsi*abs(cos(rads))+colsi*abs(sin(rads)));                      
colsf=ceil(rowsi*abs(sin(rads))+colsi*abs(cos(rads)));                     

C=uint8(zeros([rowsf colsf 3 ]));

xo=ceil(rowsi/2);                                                            
yo=ceil(colsi/2);

midx=ceil((size(C,1))/2);
midy=ceil((size(C,2))/2);

for i=1:size(C,1)
    for j=1:size(C,2)                                                       

         x= (i-midx)*cos(rads)+(j-midy)*sin(rads);                                       
         y= -(i-midx)*sin(rads)+(j-midy)*cos(rads);                             
         x=round(x)+xo;
         y=round(y)+yo;

         if (x>=1 && y>=1 && x<=size(img,1) &&  y<=size(img,2) ) 
              C(i,j,:)=img(x,y,:);  
         end

    end
end

axes(handles.axes2);
imshow(C);
title('Hasil Rotasi 90 derajat CCW')
handles.data2 = C;
guidata(hObject,handles);


% FLIP HORIZONTAL
% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
img = handles.data1;
fliph = img(:,end:-1:1,:);           %# horizontal flip
axes(handles.axes2);
imshow(fliph);
title('Hasil Flip Horizontal')
handles.data2 = fliph;
guidata(hObject,handles);


% FLIP VERTIKAL
% --- Executes on button press in pushbutton5.
function pushbutton5_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
img = handles.data1;
flipv = img(end:-1:1,:,:);           %# vertical flip
axes(handles.axes2);
imshow(flipv);
title('Hasil Flip Vertikal')
handles.data2 = flipv;
guidata(hObject,handles);


% DETEKSI TEPI SOBEL
% --- Executes on button press in pushbutton6.
function pushbutton6_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
img = handles.data1;

B=rgb2gray(img);
I=double(B);

for i=1:size(I,1)-2
for j=1:size(I,2)-2
%Sobel mask for x-direction:
mx=((2*I(i+2,j+1)+I(i+2,j)+I(i+2,j+2))-(2*I(i,j+1)+I(i,j)+I(i,j+2)));
%Sobel mask for y-direction:
my=((2*I(i+1,j+2)+I(i,j+2)+I(i+2,j+2))-(2*I(i+1,j)+I(i,j)+I(i+2,j)));

B(i,j)=sqrt(mx.^2+my.^2);
end
end

%Define a threshold value
Thresh=100;
B=max(B,Thresh);
B(B==round(Thresh))=0;
B=uint8(B);

axes(handles.axes2);
imshow(~B);
title('Hasil Deteksi Tepi Sobel')
handles.data2 = ~B;
guidata(hObject,handles);


% MEDIAN FILTER
% --- Executes on button press in pushbutton7.
function pushbutton7_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
img = handles.data1;

%READ AN 2D IMAGE
A = rgb2gray(img);

%PAD THE MATRIX WITH ZEROS ON ALL SIDES
modifyA=zeros(size(A)+2);
B=zeros(size(A));

%COPY THE ORIGINAL IMAGE MATRIX TO THE PADDED MATRIX
        for x=1:size(A,1)
            for y=1:size(A,2)
                modifyA(x+1,y+1)=A(x,y);
            end
        end
      %LET THE WINDOW BE AN ARRAY
      %STORE THE 3-by-3 NEIGHBOUR VALUES IN THE ARRAY
      %SORT AND FIND THE MIDDLE ELEMENT
       
for i= 1:size(modifyA,1)-2
    for j=1:size(modifyA,2)-2
        window=zeros(9,1);
        inc=1;
        for x=1:3
            for y=1:3
                window(inc)=modifyA(i+x-1,j+y-1);
                inc=inc+1;
            end
        end
       
        med=sort(window);
        %PLACE THE MEDIAN ELEMENT IN THE OUTPUT MATRIX
        B(i,j)=med(5);
       
    end
end
%CONVERT THE OUTPUT MATRIX TO 0-255 RANGE IMAGE TYPE
B=uint8(B);

axes(handles.axes2);
imshow(B);
title('Hasil Median Filter');
handles.data2 = B;
guidata(hObject,handles);


% SIMPAN GAMBAR
% --- Executes on button press in pushbutton8.
function pushbutton8_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
frame = getframe(handles.axes2);
im = frame2im(frame);
imwrite(im, 'hasil.jpg')


%FACE DETECTION
% --- Executes on button press in pushbutton9.
function pushbutton9_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%Detect objects using Viola-Jones Algorithm

%To detect Face
FDetect = vision.CascadeObjectDetector;

%Read the input image
I = handles.data1;

%Returns Bounding Box values based on number of objects
BB = step(FDetect,I);

axes(handles.axes2);
imshow(I);
hold on
for i = 1:size(BB,1)
    rectangle('Position',BB(i,:),'LineWidth',5,'LineStyle','-','EdgeColor','r');
end
title('Face Detection');
hold off;
handles.data2 = I;
guidata(hObject,handles);


% TERAPKAN PERUBAHAN
% --- Executes on button press in pushbutton10.
function pushbutton10_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
img =  handles.data2;
axes(handles.axes1);
imshow(img);
title('');
handles.data1 = img;
guidata(hObject,handles);


% --- Executes on button press in pushbutton11.
function pushbutton11_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
img = handles.data1;

image = rgb2gray(img);
[m,n]=size(image);
z=ones(5);
[p,q]=size(z);
w=1:p;
x=round(median(w));
anz=zeros(m+2*(x-1),n+2*(x-1));
for i=x:(m+(x-1))
    for j=x:(n+(x-1))
        anz(i,j)=img(i-(x-1),j-(x-1));
    end
end

sum=0;
x=0;
y=0;
for i=1:m
    for j=1:n
        for k=1:p
            for l=1:q 
                sum= sum+anz(i+x,j+y)*z(k,l);
                y=y+1;
            end
            y=0;
            x=x+1;
        end
        x=0;
        ans(i,j)=(1/(p*q))*(sum);
        sum=0;
    end
end

axes(handles.axes2);
imshow(uint8(ans));
title('Hasil Fillter Rata-rata 5*5');
handles.data2 = B;
guidata(hObject,handles);