clear all
%% definir notre meta-variables (i.e. duree et decalage entre les observations)
duree = 10  %duree de vol d'oiseau
dt = .1;  %les observations presque continue,
%mais nous supposerons qu'on ?chantillonne ? plusieurs reprises au fil du temps ? un intervalle fixe

%% D?finir les equations de mise a jour (matrices de coefficients): un modele base sur la physique pour savoir ou nous nous attendons ? ce que l'oiseau soit [transition d'etat (etat + vitesse)] + [controle d'entree (acceleration)]
A = [1 dt; 0 1] ; % matrice de transition d'etat: vol attendu de la caille (prediction d'etat)
B = [dt^2/2; dt]; % matrice de controle d'entree: effet attendu de l'acceleration d'entree sur l'etat.
C = [1 0]; % matrice de mesure: la mesure attendue compte tenu de l'?tat pr?vu
%comme nous ne mesurons que la position (trop difficile pour l'observateur de calculer la vitesse), nous r?glons la variable de vitesse sur zero.

%% d?finir les variables principales
u = 1.5; % definir l'amplitude d'acceleration
X= [0; 0]; %etat initial - il a deux composantes: [position; vitesse] d'oiseau.
X_est = X;  %x_est de l'estimation initiale de l'emplacement d'oiseau (ce que nous mettons a jour)
bruitAccelOiseau_amp = 2.05; %bruit de processus: la variabilite de la vitesse a laquelle l'oiseau accelere (ecart-type d'accel?ration: metres / sec ^ 2)
bruitObservation_amp = 3;  %bruit de mesure: incertitude l'observateur (ecart-type de l'emplacement, en m?tres)
var = bruitObservation_amp^2;% var: convertir le bruit de mesure (ecart-type) en matrice de covariance = variance
cov = bruitAccelOiseau_amp^2 * [dt^4/4 dt^3/2; dt^3/2 dt^2]; % cov: convertir le bruit de processus (ecart-type) en matrice de covariance
P = cov; % P : estimation de la variance de position initiale d'oiseau (matrice de covariance)


%% initialise les variables de r?sultat
% Initialiser pour la vitesse
X_pos = []; % trajectoire REELLE de vol d'oiseaux.
v = []; % Vitesse REELLE d'oiseaux.
X_pos_obs = []; % trajectoire observer d'oiseaux.



%% simulation ce que l'observateur voit au fil du temps
figure(2);clf
figure(1);clf
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
    plot(0:dt:t, X_pos, '-r.')
    plot(0:dt:t, X_pos_obs, '-k.')
    axis([0 duree -30 80])
    legend('observation','position oiseau reelle');
    hold on
    pause(0.01);
end

%tracer la vitesse, juste pour montrer qu'elle augmente constamment, en raison de l'acc?l?ration constante
figure(4);
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
figure(2);
tt = 0 : dt : duree;
plot(tt,X_pos,'-r.',tt,X_pos_obs,'-k.', tt,X_pos_est,'-g.');
 legend('position oiseau reelle','observation','estimation position[Kalman filter]');
axis([0 duree -30 80])


figure(3);clf
for T = 1:length(X_pos_est)
clf
    x = X_pos_est(T)-5:.01:X_pos_est(T)+5; % plage sur l'axe des x
     
    %prochaine position predite d'oiseau   
    hold on
    m = predic_etat(T); % moyenne
    sigma = predic_var(T); % ecart-type
    y = normpdf(x,m,sigma); % pdf: fonction densite de probabilite.
    y = y/(max(y));
    hl = line(x,y,'Color','m'); 
      
    %donn?es mesurees par l'observateur.
    m = X_pos_obs(T); % moyenne
    sigma = bruitObservation_amp; % ecart-type
    y = normpdf(x,m,sigma); % pdf
    y = y/(max(y));
    hl = line(x,y,'Color','k'); 
   
    %estimation de position combinee
    m = X_pos_est(T); % moyenne
    sigma = P_amp_est(T); % ecart-type
    y = normpdf(x,m,sigma); % pdf
    y = y/(max(y));
    hl = line(x,y, 'Color','g'); 
    axis([X_pos_est(T)-5 X_pos_est(T)+5 0 1]);    

   
    %position reelle d'oiseau
    plot(X_pos(T));
    ylim=get(gca,'ylim');
    line([X_pos(T);X_pos(T)],ylim.','linewidth',2,'color','b');
    legend('prediction etat','observation','estimation etat[kalman filter]','position oiseau reelle')
    pause(0.01);
end
