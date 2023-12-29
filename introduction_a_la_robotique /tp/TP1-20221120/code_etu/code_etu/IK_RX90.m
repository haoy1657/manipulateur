function [conf] = IK_RX90(p,R,q_k,IndSolLoc)

    % p : End-effector position
    % R : End-effector orientation
    % q_k : Current configuration
    % IndSolLoc : Solution number
    
    % Get RX90 data (load length and DH params)
    [L2,L3,L6,dh] = RX90data;
    digits(8);%relgler la précion du calcul

    % There are 8 solutions to the IK of an RX90
    % We will store them in 8x6 array (one solution per line) called q
    Q=[0.0000000 0.0000000 0.0000000 0.0000000 0.0000000 0.0000000
       0.0000000 0.0000000 0.0000000 0.0000000 0.0000000 0.0000000
       0.0000000 0.0000000 0.0000000 0.0000000 0.0000000 0.0000000
       0.0000000 0.0000000 0.0000000 0.0000000 0.0000000 0.0000000
       0.0000000 0.0000000 0.0000000 0.0000000 0.0000000 0.0000000
       0.0000000 0.0000000 0.0000000 0.0000000 0.0000000 0.0000000
       0.0000000 0.0000000 0.0000000 0.0000000 0.0000000 0.0000000
       0.0000000 0.0000000 0.0000000 0.0000000 0.0000000 0.0000000];%ici on peut aussi utiliser zeros(8,6)pour l'tablir 

    % Compute 04 position in R0
     p4 = p - R*[0 ;  0 ;  L6]; 
     conf = Q(IndSolLoc,:)';
     norme_carre_PE=sqrt(p(1)^2+p(2)^2+p(3)^2);%la norme du vecteur OP_E
     if norme_carre_PE<=(L2+L3+L6) && abs(p(3))<=1%l'espace atteignable en supossant la distance entre l'origine et la base est un et la base est assez grande
        %theta1
        theta1= vpa(atan2(p4(2),p4(1)));%on utilise vpa pour calculer plus précisement, c'est pour obtenir une correcte signe pour atan(x) quand x est tent vers zero 
        theta1prime= theta1 + pi;%deux solutions pout theta1
        for i=1:4
            Q(i,1)=theta1;
            Q(i+4,1)=theta1prime;
        end
        %theta2 et theta3
        for i=1:8
            lambda=p4(1)*cos(Q(i,1)) + p4(2)*sin(Q(i,1)) ; %on a les formule pour lambda eps A B C d'apres l'énocé, et puis on peut utilisr ces paramètre pour les deux solution du theta2
            epsilon=(-1)^i ; 
            A=2*L2*lambda;
            B = 2*L2*p4(3) ; 
            C = (lambda)^2 + (p4(3))^2 + (L2)^2 - (L3)^2;
            Q(i,2)=vpa(atan2(B*C-epsilon*A*sqrt(A^2+B^2-C^2),A*C+epsilon*B*sqrt(A^2+B^2-C^2)));
            Q(i,3)=vpa(atan2(lambda*cos(Q(i,2))+p4(3)*sin(Q(i,2))-L2 ,lambda*sin(Q(i,2))-p4(3)*cos(Q(i,2)) )) ;
        end
        %theta5 et puis theta4 et theta6
        zeroT6 = [R p ; 0 0 0 1];
        %theta5
        for i=1:8
            zeroT1=TH(Q(i,1),dh(1,:));%on utilse la fonction TH pour calculer matrice homogène directement
            unT2=TH(Q(i,2),dh(2,:));
            deuxT3=TH(Q(i,3),dh(3,:));

            zeroT3=(zeroT1)*(unT2)*(deuxT3);%multiplication des matrice homogène
            troisT6=(inv(zeroT3))*(zeroT6) ;
            Q(i,5)=vpa(atan2(sqrt((troisT6(1,3))^2+(troisT6(2,3))^2),troisT6(3,3)));
            %theta4
            if abs(sin(Q(i,5)))>=10^-6%c'est pour éviter le cas quand sin(Q(i,5))n'est parfaitement nul, si il y assez petit, on le considère nul 
                Q(i,4)=vpa(atan2(troisT6(2,4),troisT6(1,4)));
            else
                Q(i,4)=input("veuillez choisir une valeur arbitraire pour theta4 ici theta5 est nul:");%valeur arbitraire du theta4 quand sin(theta5)nul
            end
            %theta6
            troisT4=TH(Q(i,4),dh(4,:));
            quatreT5=TH(Q(i,5),dh(5,:));
            zeroT5=(zeroT3)*(troisT4)*(quatreT5);
            cinqT6=(inv(zeroT5))*(zeroT6) ;
            Q(i,6)=vpa(atan2(cinqT6(2,1),cinqT6(1,1)));
        end
        Q(3,5)=-Q(1,5);Q(4,5)=-Q(2,5);Q(7,5)=-Q(5,5);Q(8,5)=-Q(6,5);%theta5prime=-theta5 
        Q(3,4)=pi+Q(1,4);Q(4,4)=pi+Q(2,4);Q(7,4)=pi+Q(5,4);Q(8,4)=pi+Q(6,4);%theta4prime=theta4 plus pi
        Q(3,6)=pi+Q(1,6);Q(4,6)=pi+Q(2,6);Q(7,6)=pi+Q(5,6);Q(8,6)=pi+Q(6,6);%theta6prime=theta6 plus pi
    disp("Q sans la correction angulaire:")%première affichage du Q
    disp(Q)
        %minimiser les variation angulaire
        for i = 1:8  
            for j = 1:6  
                if abs( Q(i,j) ) > 3.14159%on choisit l'ordre précision du pi comme dix puissance moins cinq, parce que la précison du calcul général est dix puissance moins sept
                    if Q(i,j)>=0% on le divise en deux cas parceque la signe du fix()inverse quand Q est négatif
                        Q(i,j) = Q(i,j)-sign(Q(i,j))*fix(Q(i,j)/3.14159)*3.14159;
                    end
                    if Q(i,j)<0
                        Q(i,j) = Q(i,j)+sign(Q(i,j))*fix(Q(i,j)/3.14159)*3.14159;
                    end
                end
            end
        end
    disp("Q avec la correction angulaire:")%deuxième affichage du Q
    disp(Q)

   else
       disp("la pose donnée n'est pas atteignable")
   end
end
% For example :
% For p_E = [-0.5;0;0.6] and R_E = [-1 0 0;0 -1 0;0 0 1];
% 
% q =
% 
%     3.1416    1.3495         0   -0.0000    1.7921   -3.1416
%     3.1416   -0.0000   -3.1416   -1.5708    0.0000   -1.5708
%     3.1416    1.3495         0    3.1416   -1.7921         0
%     3.1416   -0.0000   -3.1416    1.5708   -0.0000    1.5708
%     6.2832   -3.1416   -0.0000    1.5708    0.0000   -1.5708
%     6.2832    1.7921    3.1416    3.1416    1.7921   -3.1416
%     6.2832   -3.1416   -0.0000    4.7124   -0.0000    1.5708
%     6.2832    1.7921    3.1416    6.2832   -1.7921         0
 



