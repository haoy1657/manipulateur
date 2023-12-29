function Ex1
clear all;
close all;
clc;
    
 
% Ralliement de points de passage

% fonction d'état : x étant l'état et u le vecteur de commande
    
    function xpoint=f(x,u)
        theta=x(3);
        xpoint=[u(1)*cos(theta);u(1)*sin(theta);u(2)];
    end


% point but final et orientation final désirée
D=[25;6]; betad=0;

% x,y,theta (conditions initiales en position x,y et orientation theta)
X=[-10;-10;-pi/2];  

% pas de discrétisation du temps
dt=0.1;

% Temps final de simulation
Tfinal = 20;

% les gains de commande
krho = 2;
kalpha= 7;
kbeta= -10;
% décommenter ces deux lignes si vous voulez superposer les poses du
% robot
clf(); 
figure(1);
hold on; axis([-30 30 -30 30]); axis square;
title('le trajectoire du robot avec le point départ[-20,-10] et le point final[25,6]')

i=1;
for t=0:dt:Tfinal
  
    % commenter ces deux lignes si vous voulez superposer les poses du
    % robot
%     clf(); 
%     hold on; axis([-30 30 -30 30]); axis square;

    % Tracer la nouvelle position du robot
    trace_robot(X,'blue'); 
    
    % Tracer la pose finale désirée du robot    
    plot(D(1),D(2),'ro'); 
    
    drawnow();
    
    %pause(0.1);

    % Calcul des paramètres d'erreurs
    rho = sqrt((D(1)-X(1))^2 + (D(2)-X(2))^2);
    beta= atan2(D(2)-X(2),D(1)-X(1));
    alpha=beta-X(3);
    
    % arrêt de la boucle si le point D est atteint
    if (rho < 0.1) break; end
    
    % calcul des commandes u et omega
    U(1)=krho*rho;%Vitesse linéaire
    U(2)=kalpha*alpha+kbeta*(betad-beta);%vitesse angulaire
    
    %intégrer l'équation d'état (déterminer le nouveau état) par un simple
    %schéma d'Euler explicite 
   
    X=X+f(X,U)*dt;%(x,y,theta)+(u(1)*cos(theta);u(1)*sin(theta);u(2))*dt
        
    %les vitesses des roues
    r= 1;
    w=2;
    phi(i,1)=(1/r)*(U(1)+w*U(2));
    phi(i,2)=(1/r)*(U(1)-w*U(2));

    i=i+1; 
    Tfinal=t;
  
end

% évolution des vitesses des roues 

figure(2)
plot(0:dt:Tfinal,phi(:, 1))
hold on;
plot(0:dt:Tfinal,phi(:, 2))
title('la vitesses des roues')
legend('vitesse roue droite','vitesse roue gauche')


%décommenter les lignes suivantes 
%création d'une série de points de passage avec angles d'orientation associés

D=transpose([[-10;-5],[0;0],[10;7],[25;6]]);
betad=[pi/2,pi/4,pi/6,pi/4];
X=[-20;-10;-pi/2];  
figure(3)
hold on; axis([-30 30 -30 30]); axis square;
title('le trajectoire du robot avec 4 point passage')

%4 points de passage

for i=1:4
    for t=0:dt:Tfinal
        trace_robot(X,'blue');
        hold on 
        plot(D(i,1),D(i,2),'ro'); 
        drawnow();
        rho = sqrt((D(i,1)-X(1))^2 + (D(i,2)-X(2))^2);
        beta= atan2(D(i,2)-X(2),D(i,1)-X(1));
        alpha=beta-X(3);    
        if (rho < 0.1) break; end
        U(1)=krho*rho;
        U(2)=kalpha*alpha+kbeta*(betad(i)-beta);
        X=X+f(X,U)*dt;
    end 
end  
    
end