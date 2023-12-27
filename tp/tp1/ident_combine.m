%% CE SCRIPT PERMET D'IDENTIFIER LES PARAMETRES DES
%% TERMES DE COUPLAGE POUR LES AXES 1 et 2
%% v1 G. MOREL - 2005/12/29
%% v2 V. PADOIS - 2007/10/31
%% v3 V. PADOIS - 2012/12/03

clear all;
load releve_mvts_combines;

%% constantes connues
kc1=0.0525;
N1=20.25;
kc2=0.0525;
N2=4.5;

%% Parametres identifies a vitesse constante
%% Pour l'axe 1 : 
   
p1=[0.885713295835873   0.156775736657458   0.011668376531522  -0.045244543135239   ];
alpha1=p1(1);
a1=p1(2);
b1=p1(3);
c1=p1(4);
%% Pour l'axe 2 :
p2=[0.084924593041250   0.070495234703875   0.000958000578040  -0.012077896410435];
alpha2=p2(1);
a2=p2(2);
b2=p2(3);
c2=p2(4);

%% identification a partir des donnees filtrees
for(i=1:length(t)) 
    Z(2*i-1:2*i,1:3)=[qppfil1(i) qppfil2(i)*cos(q2(i)-q1(i))-qpfil2(i)^2*sin(q2(i)-q1(i)) 0
        0 qppfil1(i)*cos(q2(i)-q1(i))+ qpfil1(i)^2*sin(q2(i)-q1(i)) qppfil2(i)];
    u(2*i-1,1)=N1*kc1*ifil1(i)-alpha1*cos(q1(i))-a1*sign(qpfil1(i))-b1*qpfil1(i)-c1;
    u(2*i,1)=N2*kc2*ifil2(i)-alpha2*cos(q2(i))-a2*sign(qpfil2(i))-b2*qpfil2(i)-c2;
end

p3=pinv(Z)*u;
format long
disp('Parametres estimes a partir des donnees filtrees :');
p3'

% reconstruction du modele complet
P1=p3(1);
P2=p3(2);
P3=p3(3);

for(i=1:length(t)) 
    %% couple d'inertie
    ciner(2*i-1,1)=P1*qppfil1(i)+P2*cos(q2(i)-q1(i))*qppfil2(i); %% AXE 1 
    ciner(2*i,1)= P2*cos(q2(i)-q1(i))*qppfil1(i)+P3*qppfil2(i);%%AXE 2
    %% couple centrifuge
    ccentri(2*i-1,1)=-P2*sin(q2(i)-q1(i))*(qpfil2(i)^2); %% AXE 1
    ccentri(2*i,1)=P2*sin(q2(i)-q1(i))*(qpfil1(i)^2); %% AXE 2
    %% couple de gravite    
    cgravi(2*i-1,1) =alpha1*cos(q1(i)) ; %% AXE 1
    cgravi(2*i,1) = alpha2*cos(q2(i)); %% AXE 2
    %% couple de frottements
    cfrott(2*i-1,1)=a1*sign(qpfil1(i))+b1*qpfil1(i)+c1; %% AXE 1
    cfrott(2*i,1)=a2*sign(qpfil2(i))+b2*qpfil2(i)+c2; %% AXE 2
    %% couple total
    ctotal(2*i-1:2*i,1)=ciner(2*i-1:2*i,1)+ccentri(2*i-1:2*i,1)+cgravi(2*i-1:2*i,1)+cfrott(2*i-1:2*i,1);
end

%% Affichage des commandes.
figure(1) %% pour l'axe 1
clf
hold on
grid on
h=plot(t,N1*kc1*i1,'y');
h=plot(t,N1*kc1*ifil1,'b');
set(h,'LineWidth',1.5);
h=plot(t,ciner(1:2:length(ctotal)),'r');
set(h,'LineWidth',1.5);
h=plot(t,cgravi(1:2:length(ctotal)),'m');
set(h,'LineWidth',1.5);
h=plot(t,ccentri(1:2:length(ctotal)),'k');
set(h,'LineWidth',1.);
h=plot(t,cfrott(1:2:length(ctotal)),'g');
set(h,'LineWidth',1.);
h=plot(t,ctotal(1:2:length(ctotal)),'c--');
set(h,'LineWidth',1.5);
legend('\Gamma_1 mesure','\Gamma_1 filtre','inertie','gravite','centrifuge','frottements','modele total');
title('Resultats axe 1 ; identification a partir de donnees filtrees');

figure(2)
clf
hold on
grid on
h=plot(t,N2*kc2*i2,'y');
h=plot(t,N2*kc2*ifil2,'b');
set(h,'LineWidth',1.5);
h=plot(t,ciner(2:2:length(ctotal)),'r');
set(h,'LineWidth',1.5);
h=plot(t,cgravi(2:2:length(ctotal)),'m');
set(h,'LineWidth',1.5);
h=plot(t,ccentri(2:2:length(ctotal)),'k--');
set(h,'LineWidth',1);
h=plot(t,cfrott(2:2:length(ctotal)),'g');
set(h,'LineWidth',1);
h=plot(t,ctotal(2:2:length(ctotal)),'c--');
set(h,'LineWidth',1.5);
legend('\Gamma_2 mesure','\Gamma_2 filtre','inertie','gravite','centrifuge','frottements','modele total');
title('Resultats axe 2 ; identification a partir de donnees filtrees');

figure(3)
clf;
hold on;
grid on;
h=plot(t,N1*kc1*i1,'b');
set(h,'LineWidth',1.5);
h=plot(t,ctotal(1:2:length(ctotal)),'c--');
set(h,'LineWidth',1.5);
legend('couple mesuré','couple estimé');
title('Resultats axe 1 ; compaison entre couple estimé et couple mesuré ');

figure(4)
clf;
hold on;
grid on;
h=plot(t,N2*kc2*i2,'b');
set(h,'LineWidth',1.5);
h=plot(t,ctotal(2:2:length(ctotal)),'c--');
set(h,'LineWidth',1.5);
legend('couple mesuré','couple estimé');
title('Resultats axe 2 ; compaison entre couple estimé et couple mesuré');

figure(5)
clf;
hold on;
grid on;
h=plot(t,N1*kc1*ifil1,'b');
set(h,'LineWidth',1.5);
h=plot(t,ctotal(1:2:length(ctotal)),'c--');
set(h,'LineWidth',1.5);
legend('couple filtré','couple estimé');
title('Resultats axe 1 ; compaison entre couple estimé et couple filtré');

figure(6)
clf;
hold on;
grid on;
h=plot(t,N2*kc2*ifil2,'b');
set(h,'LineWidth',1.5);
h=plot(t,ctotal(2:2:length(ctotal)),'c--');
set(h,'LineWidth',1.5);
legend('couple filtré','couple estimé');
title('Resultats axe 2 ; compaison entre couple estimé et couple filté');

