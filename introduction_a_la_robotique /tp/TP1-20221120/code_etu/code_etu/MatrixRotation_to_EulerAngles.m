function EA=MatrixRotation_to_EulerAngles(R)

% This function computes Euler Angles xy'z" from a given rotation matrix
% R : Matrix Rotation 3x3 
% EA : Euler Angles 3x1
% Theta parameter will be choosed always between -pi/2 and pi/2

if abs(abs(R(1,3)) -1) > 10e-4
    theta=asin(R(1,3)); %solution entre -pi/2 et +pi/2 pour theta
    phi=atan2(-R(2,3)/cos(theta),R(3,3)/cos(theta));
    psi=atan2(-R(1,2)/cos(theta),R(1,1)/cos(theta));
else
    phi=0;
    theta=sign(R(1,3))*pi/2;
    psi=-R(1,3)*phi+atan2(R(2,1),R(2,2));
end

EA=[phi,theta,psi];

end
    
    
    
    
    
    