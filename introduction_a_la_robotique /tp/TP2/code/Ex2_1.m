function Ex2_1
clear all;
close all;
clc; 
%exo2 suivi de tracjectoire
%Première contrôleur pour suivre la trajectoire donnée
% Pour cette partie, il existe deux contrôleurs, on suppose qu’il y a point qui bouge dans la
% trajectoire référence en fonction du temps, et on utilise le contrôleur qu’on a appliqué dans
% l’exercice 1.
% xpoint=[u(1)*cos(theta)-lx*u(2)*sin(theta);u(1)*sin(theta)+lx*u(2)*cos(theta);u(2)];
    function xpoint=f(x,u)
        theta=x(3);
        xpoint=[u(1)*cos(theta);u(1)*sin(theta);u(2)];
    end
%exo2 suivi de tracjectoire
%choix du trajectoire référentielle
%xrt=t;yrt=sin(t);
% les gains de commande
krho = 1  ;
kalpha= 5;
kbeta= -10;
Tfinal=45;dt=0.1;
X=[-2.5;2.5;0];
xr=0:dt:Tfinal;
i=1;
%creer le trajectoire suivi,on ajoute xr,yr et thetar pour chqaue point du
%trajectoire 
for t=0:dt:(Tfinal/3)
    yr(1,i)=0.5*t;
    thetar(1,i)=0.5;  
    i=i+1;
end
i=151;
for t=(Tfinal/3):dt:(2*Tfinal/3)
    yr(1,i)=t-7.5;
    thetar(1,i)=1;
    i=i+1;
end
i=301;
for t=(2*Tfinal/3):dt:Tfinal
    yr(1,i)=-t+52.5;
    thetar(1,i)=-1;
    i=i+1;
end

figure(1);
title("suivi du trajectoire en utilisant le controleur qu'on a fait dans l'exo1 avec le point départ[-2.5,2.5]")
xlabel("x")
ylabel("y")
hold on;
set(gcf,'position',[300 0 1000 1500 ]);
hold on;
axis([-5 50 -5 50]); axis square;
plot(xr,yr(1,:))
hold on;
plot(xr,thetar(1,:));


i=1;
point1=[];  
point2=[];
point1=[point1,X(1)];
point2=[point2,X(2)];

for t=0:dt:Tfinal
    
    D=[xr(1,i),yr(1,i)];betad=thetar(1,i);%La différence avec l’exercice 1 est que les deux positions varient au cours du temps. De
                                          %cette façon, on a réalisé un point mobile sur le chemin de suivi.
    trace_robot(X,'blue');
    drawnow();  
    rho = sqrt((D(1)-X(1))^2 + (D(2)-X(2))^2);%définir le controleur
    beta= atan2(D(2)-X(2),D(1)-X(1));
    alpha=beta-X(3);
    %if (rho < 0.1) break; end
    U(1)=krho*rho;%Vitesse linéaire
    U(2)=kalpha*alpha+kbeta*(betad-beta);%vitesse angulaire
    X=X+f(X,U)*dt;
    point1(1,i+1)=X(1);
    point2(1,i+1)=X(2);
    disp(X)
    i=i+1;
    
end



%
figure(2);
set(gcf,'position',[300 0 1000 1500 ]);
hold on;
axis([-5 50 -5 50]); axis square;
hold on;
plot(xr,yr(1,:))
hold on;
plot(xr,thetar(1,:));
hold on;
plot(point1,point2,"-k","LineWidth",1.5)
hold on;
title("le trajectoire du centre du robot avec le point départ[-2.5,2.5]")
xlabel("x")
ylabel("y")
legend("le trajectoire suivi","thetar or betad","trajectoire du centre du robot")


end
