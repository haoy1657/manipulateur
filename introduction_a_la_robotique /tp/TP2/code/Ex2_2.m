function Ex2_2
clear all;
close all;
clc; 
Tfinal=30;
dt=0.1;
lx=2;
Xp=[-5,0,pi/2];
X=Xp-[lx,0,0];
xr=0:dt:Tfinal;
%creer le trajectoire suivi,on ajoute xr,yr et thetar xrpoint, yrpoint, thetarpoint
%pour chqaue point du trajectoire 
i=1;    
for t=0:dt:Tfinal
    xrpoint(1,i)=1;
    yrpoint(1,i)=-5*sin(0.5*t);
    thetarpoint(1,i)=-2.5*cos(0.5*t);
    yr(1,i)=10*cos(0.5*t);
    thetar(1,i)=-5*sin(0.5*t);  
    i=i+1;
end

%les gains de commande
kx=0.5;
ky=0.5;
komega=1;

figure(1);
title("suivi du trajectoire en stabilisant autour zero l'erreur de suivi")
xlabel("x")
ylabel("y")
hold on;
set(gcf,'position',[300 0 1000 1500 ]);
hold on;
axis([-5 35 -20 20]); axis square;
plot(xr,yr(1,:))
hold on;
plot(xr,thetar(1,:));
i=1;
point1=[];
point2=[];
point3=[];
point4=[];
point1=[point1,Xp(1)];
point2=[point2,Xp(2)];
point3=[point3,X(1)];
point4=[point4,X(2)];
for t=0:dt:Tfinal
    X=Xp-[lx,0,0];
    point3(i,i+1)=X(1);
    point4(i,i+1)=X(2);
    e=[xr(1,i)-Xp(1),yr(1,i)-Xp(2),thetar(1,i)-Xp(3)];%erreur pour controler
    E(1,i)=e(1);
    E(2,i)=e(2);
    E(3,i)=e(3);
    disp("erreur de suivi:")
    disp(e)
    trace_robot(X,'blue');
    drawnow();
    %définir le controleur
    v1=xrpoint(1,i)+kx*e(1);
    v2=yrpoint(1,i)+ky*e(2);
    omega=thetarpoint(1,i)+komega*e(3);
    Xp=Xp+[v1,v2,omega]*dt;
    point1(1,i+1)=Xp(1);
    point2(1,i+1)=Xp(2);
    i=i+1;
end

%
figure(2);
set(gcf,'position',[300 0 1000 1500 ]);
hold on;
axis([-5 35 -20 20]); axis square;
grid();
hold on;
plot(xr,yr(1,:),"LineWidth",4)
hold on;
plot(xr,thetar(1,:),"LineWidth",4);
hold on;
plot(point1,point2,".m","LineWidth",1)
hold on;
plot(point3,point4,".b","LineWidth",1)
hold on;
title("le trajectoire du centre du robot")
xlabel("x")
ylabel("y")
legend("le trajectoire suivi","thetar or betad","trajectoire du point reférentiel du robot","trajectoire du centre du robot")



end