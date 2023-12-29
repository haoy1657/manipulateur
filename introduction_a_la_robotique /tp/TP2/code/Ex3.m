function Ex3
clear all;
close all;
clc;
    function point_intersection=linecross(k1,b1,k2,b2)%function pour calculer le coordonnée du point des deux droite dans l'espace
 
    if k1==k2 && b1==b2
        disp('les deux droits confondu');
    elseif k1==k2 && b1~=b2
        disp('pas de point intersection')
    else 
        x=(b2-b1)/(k1-k2);
        y=k1*x+b1;
        point_intersection=[x,y];
    end
    end

    function d=disntace(x1,y1,x2,y2)%distance entre deux point donnés
        d=sqrt((x1-x2)^2+(y1-y2)^2);
    end

    function xi=Xi(x,y) %cette fonction est pour calculer la distance le plus proche entre le point P 
                        %qui est par rapport a le repere du robot.
                        %la distance du point P a la courbe C(trajectoire)
        if y>=-a/b*x-c/b %la signe du xi départ sa position par rapport au trajectoire
            xi=abs(a*x+b*y+c)/(sqrt(a^2+b^2));
        elseif y<-a/b*x-c/b
            xi=-abs(a*x+b*y+c)/(sqrt(a^2+b^2));
        end
    end
    function s=S(x,y)%cette function est pour calculer la valeur de s quand on projet un point dans un courbe C
        point_projection=linecross(-a/b,-c/b,b/a,y-(b/a)*x);
        if point_projection(1)<=0%la signe de s est négatif quand le point de projection se situe a gauche de la droite
            s=-distance(point_projection(1),point_projection(2),point_depart(1),point_depart(2));
        elseif point_projection(1)>0
            s=distance(point_projection(1),point_projection(2),point_depart(1),point_depart(2));
        end
    end
%définir l'équation cartésienne, on prends un segment de cette droite x-->[0,30]    
a=1;b=-1;c=1;
lx=2;ly=-2;
u=1;
k=1.5;%si on augmente dt, on doit reduire la valeur de k pour rendre la valeur de xi autour zero
dt=0.3;
X=[-5,-10,pi/6];
point_depart=[0,-c/b];
point_final=[30,(-a/b)*30-c/b];
s_total=distance(point_depart(1),point_depart(2),point_final(1),point_final(2));

%tracer le trajectoire reference
i=1;
for x1=0:0.1:30
    y1(1,i)=-a/b*x1-c/b;
    i=i+1;
end
figure(1)
title("suivi du chein ")
xlabel("x")
ylabel("y")
hold on;
set(gcf,'position',[300 0 1000 1500 ]);
hold on;
axis([-10 35 -10 35]); axis square;
grid()
x1=0:0.1:30;
plot(x1,y1(1,:),'-b',"LineWidth",3)
hold on;
s=1;
xi=1;
t=0;
tab_s=[];
tab_xi=[];
tab_t=[];
while abs(s-s_total)>0.5 || xi>0.5
    trace_robot(X,'blue');
    drawnow();
    Xp=X-[lx,ly,0];
    xi=Xi(Xp(1),Xp(2));%calculer xi
    point_projection=linecross(-a/b,-c/b,b/a,Xp(2)-(b/a)*Xp(1));
    s=S(Xp(1),Xp(2));%calculer s
    tab_s=[tab_s,s];
    tab_xi=[tab_xi,xi];
    t=t+dt;%le temps passé total
    tab_t=[tab_t,t];
    thetae=X(3)-atan(-a/b);
    %définir le controleur
    omega=-u*sin(thetae)/(lx*cos(thetae))-u*k*xi;
    theta=X(3);
    xpoint=[u*cos(theta);u*sin(theta);omega];
    X=X+xpoint*dt;
    

end
figure(2)
plot(tab_s,tab_xi,'-b')
title('xi en fonction de s')
xlabel("s")
ylabel("xi")
grid()
figure(3)
plot(tab_t,tab_s,'-b')
title('s en fonction de t')
xlabel("t")
ylabel("s")
grid()
figure(4)
plot(tab_t,tab_xi,'b')
title('xi en fonction de t')
xlabel("t")
ylabel("xi")
grid()


 end







