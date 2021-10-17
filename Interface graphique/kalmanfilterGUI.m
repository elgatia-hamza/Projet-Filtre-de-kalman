function varargout = kalmanfilterGUI(varargin)
% KALMANFILTERGUI MATLAB code for kalmanfilterGUI.fig
%      KALMANFILTERGUI, by itself, creates a new KALMANFILTERGUI or raises the existing
%      singleton*.
%
%      H = KALMANFILTERGUI returns the handle to a new KALMANFILTERGUI or the handle to
%      the existing singleton*.
%
%      KALMANFILTERGUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in KALMANFILTERGUI.M with the given input arguments.
%
%      KALMANFILTERGUI('Property','Value',...) creates a new KALMANFILTERGUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before kalmanfilterGUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to kalmanfilterGUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help kalmanfilterGUI

% Last Modified by GUIDE v2.5 11-Jun-2020 19:43:49

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @kalmanfilterGUI_OpeningFcn, ...
                   'gui_OutputFcn',  @kalmanfilterGUI_OutputFcn, ...
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


% --- Executes just before kalmanfilterGUI is made visible.
function kalmanfilterGUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to kalmanfilterGUI (see VARARGIN)

% Choose default command line output for kalmanfilterGUI
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes kalmanfilterGUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = kalmanfilterGUI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double


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



function edit4_Callback(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit4 as text
%        str2double(get(hObject,'String')) returns contents of edit4 as a double


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


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%% definir notre meta-variables (i.e. duree et decalage entre les observations)
duree = str2num(get(handles.edit1,'string'));  %duree de vol d'oiseau
%if str2num == 0
dt = .1;  %les observations presque continue,
%mais nous supposerons qu'on ?chantillonne ? plusieurs reprises au fil du temps ? un intervalle fixe

%% D?finir les equations de mise a jour (matrices de coefficients): un modele base sur la physique pour savoir ou nous nous attendons ? ce que l'oiseau soit [transition d'etat (etat + vitesse)] + [controle d'entree (acceleration)]
A = [1 dt; 0 1] ; % matrice de transition d'etat: vol attendu de la caille (prediction d'etat)
B = [dt^2/2; dt]; % matrice de controle d'entree: effet attendu de l'acceleration d'entree sur l'etat.
C = [1 0]; % matrice de mesure: la mesure attendue compte tenu de l'?tat pr?vu
%comme nous ne mesurons que la position (trop difficile pour l'observateur de calculer la vitesse), nous r?glons la variable de vitesse sur zero.

%% d?finir les variables principales
u = str2num(get(handles.edit4,'string')); % definir l'amplitude d'acceleration
X= [0; 0]; %etat initial - il a deux composantes: [position; vitesse] d'oiseau.
X_est = X;  %x_est de l'estimation initiale de l'emplacement d'oiseau (ce que nous mettons a jour)
bruitAccelOiseau_amp = str2num(get(handles.edit2,'string')); %bruit de processus: la variabilite de la vitesse a laquelle l'oiseau accelere (ecart-type d'accel?ration: metres / sec ^ 2)
bruitObservation_amp = str2num(get(handles.edit3,'string'));  %bruit de mesure: incertitude l'observateur (ecart-type de l'emplacement, en m?tres)
var = bruitObservation_amp^2;% var: convertir le bruit de mesure (ecart-type) en matrice de covariance = variance
cov = bruitAccelOiseau_amp^2 * [dt^4/4 dt^3/2; dt^3/2 dt^2]; % cov: convertir le bruit de processus (ecart-type) en matrice de covariance
P = cov; % P : estimation de la variance de position initiale d'oiseau (matrice de covariance)


%% initialise les variables de r?sultat
% Initialiser pour la vitesse
X_pos = []; % trajectoire REELLE de vol d'oiseaux.
v = []; % Vitesse REELLE d'oiseaux.
X_pos_obs = []; % trajectoire observer d'oiseaux.



%% simulation ce que l'observateur voit au fil du temps
axes(handles.axes1);
for t = 0 : dt: duree
    % Generer le vol Oiseau
    bruitAccelOiseau = bruitAccelOiseau_amp * [(dt^2/2)*randn; dt*randn];
    X= A * X+ B * u + bruitAccelOiseau;
    % G?n?rez ce que l'observateur voit
    bruitObservation = bruitObservation_amp * randn;
    y = C * X+ bruitObservation;
    X_pos = [X_pos; X(1)];
    X_pos_obs = [X_pos_obs; y];
    v = [v; X(2)];
    %iteratively plot what the ninja sees
    plot(0:dt:t, X_pos, '-r.',0:dt:t, X_pos_obs, '-k.')
    %plot()
    axis([0 duree -30 80])
    legend('observation','position oiseau reelle');
    pause(0.01);
end

%tracer la vitesse, juste pour montrer qu'elle augmente constamment, en raison de l'acc?l?ration constante
axes(handles.axes2);
plot(0:dt:t, v, '-b.')
legend('vitesse');


%% Faites le filtrage kalman
% initier les variables d'estimation
X_pos_est = []; % position estimer d'oiseau.
v_est = []; % vitesse estimer d'oiseau.
X= [0; 0]; % reinitialise etat.
P_est = P;
P_amp_est = [];
predic_etat = [];
predic_var = [];
for t = 1:length(X_pos)
    % Prediction l'etat suivant d'oiseau avec le dernier etat et le mouvement predit.
    X_est = A * X_est + B * u;
    predic_etat = [predic_etat; X_est(1)] ;
    %predict next covariance
    P = A * P * A' + cov;
    predic_var = [predic_var; P] ;
    % covariance de mesure observateur prevue
    % Gain de Kalman
    K = P*C'*inv(C*P*C'+var);
    % Mettez ? jour l'estimation de l'etat.
    X_est = X_est + K * (X_pos_obs(t) - C * X_est);
    % mettre a jour l'estimation de la covariance.
    P =  (eye(2)-K*C)*P;
    % Stoker pour le tracage
    X_pos_est = [X_pos_est; X_est(1)];
    v_est = [v_est; X_est(2)];
    P_amp_est = [P_amp_est; P(1)];
end

% Tracer les resultats
axes(handles.axes3);
tt = 0 : dt : duree;
plot(tt,X_pos,'-r.',tt,X_pos_obs,'-k.', tt,X_pos_est,'-g.');
 legend('position oiseau reelle','observation','estimation position[Kalman filter]');
axis([0 duree -30 80])


axes(handles.axes4);
for T = 1:length(X_pos_est)
    x = X_pos_est(T)-5:.01:X_pos_est(T)+5; % plage sur l'axe des x
     
    %prochaine position predite d'oiseau   
    m = predic_etat(T); % moyenne
    sigma = predic_var(T); % ecart-type
    y1 = normpdf(x,m,sigma); % pdf: fonction densite de probabilite.
    y1 = y1/(max(y1));
    %hl = line(x,y,'Color','m'); 
      
    %donn?es mesurees par l'observateur.
    m = X_pos_obs(T); % moyenne
    sigma = bruitObservation_amp; % ecart-type
    y2 = normpdf(x,m,sigma); % pdf
    y2 = y2/(max(y2));
    %hl = line(x,y,'Color','k ');
   
    %estimation de position combinee
    m = X_pos_est(T); % moyenne
    sigma = P_amp_est(T); % ecart-type
    y3 = normpdf(x,m,sigma); % pdf
    y3 = y3/(max(y3));
    %hl = line(x,y, 'Color','g');
    plot(x,y1,'m',x,y2,'k',x,y3,'g');
    axis([X_pos_est(T)-5 X_pos_est(T)+5 0 1]);    

   
    %position reelle d'oiseau
    hold on;
    plot(X_pos(T));
    ylim=get(gca,'ylim');
    line([X_pos(T);X_pos(T)],ylim.','linewidth',2,'color','b');
    legend('prediction etat','observation','estimation etat[kalman filter]','position oiseau reelle')
    pause(0.01);
    hold off;
end
