function Ex_bonus1
% Bonus : Suivi d’un robot réel ou virtuel
clear all;
close all;
clc;
    function xpoint=f(x,u)
        theta=x(3);
        xpoint=[u(1)*cos(theta);u(1)*sin(theta);u(2)];
    end

Ur=[100,0];%la vitesse de la ligne et la vitesse angulaire
omegar=0;
%les gains de commande
k1=1e-4;
k2=1e-4;
k3=0.1;

%initialisation
Tfinal=30;
dt=0.1;
X=[0,2000,0];
Xr=[0,0,0];
%décrire la position et orientation du robot rouge 
i=1;
for t=0:dt:Tfinal
    xr(1,i)=Ur(1)*t;
    yr(1,i)=0;
    thetar(1,i)=0;
    omegar(1,i)=0;
    i=i+1;
end
figure(1)
set(gcf,'position',[300 0 1000 1500 ]);
hold on; axis([-100 3500 -100 100]); axis square;
grid()
title(' Suivi d"un robot réel ou virtuel')
i=1;
E1=[];
E2=[];
E3=[];
for t=0:dt:Tfinal
    trace_robot(X,'b');
    trace_robot(Xr,'r'); 
    drawnow();
    e=[X(1)-xr(1,i),X(2)-yr(1,i),X(3)-thetar(1,i)];%erreur de suivi
    E1=[E1,e(1)];E2=[E2,e(2)];E3=[E3,e(3)];
    %définir le controleur 
    % les 3 parametres d'erreur exprimés dans le repére du véhicule de référence
    z1=e(1)*cos(thetar(1,i))+e(2)*sin(thetar(1,i));
    z2=-e(1)*sin(thetar(1,i))+e(2)*cos(thetar(1,i));
    z3=tan(e(3));
    omega1=-k1*abs(Ur(1))*(z1+z2*z3);
    omega2=-k2*Ur(1)*z2-k3*abs(Ur(1))*z3;
    %La loi de commande
    u=(omega1+Ur(1))/cos(e(3));%vitesse linéaire
    omega=omega2*(cos(e(3)))^2+omegar(1,i);%vitesse angulaire
    U=[u,omega];
    % calcul des vitesses des roues
    %paramètre du robot
    r= 1;
    w=2;
    phi(1,i)=(1/r)*(U(1)+w*U(2));
    phi(2,i)=(1/r)*(U(1)-w*U(2));
    %mis a jour la position et orientation des robot
    X=X+f(X,U)*dt;
    Xr=Xr+f(Xr,Ur)*dt;
    i=i+1;
end
t=0:dt:Tfinal;
figure(2)
set(gcf,'position',[300 0 1000 1500 ]);
hold on
title("e1 en fonction du temps")
xlabel('t')
ylabel('e1')
hold on
plot(t,E1)
figure(3)
set(gcf,'position',[300 0 1000 1500 ]);
hold on
title("e2 en fonction du temps")
xlabel('t')
hold on
ylabel('e2')
plot(t,E2)
figure(4)
set(gcf,'position',[300 0 1000 1500 ]);
hold on
title("e3 en fonction du temps")
xlabel('t')
hold on
ylabel('e3')
plot(t,E3)
figure(5)
set(gcf,'position',[300 0 1000 1500 ]);
title("vitesse des roues en fonction du temps")
hold on
xlabel('t')
hold on
ylabel('v')
plot(t,phi(1,:),"r")
hold on
set(gcf,'position',[300 0 1000 1500 ]);
plot(t,phi(2,:),"b")
legend('roue1','roue2')

end








