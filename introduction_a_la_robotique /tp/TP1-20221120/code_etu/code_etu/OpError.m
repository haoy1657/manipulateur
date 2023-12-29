function [dP,dO,T] = OpError(EEp,EEo,q,dh)

% Returns the position error vector and the orientation error dP and dO
% also returns the EE (EndEffector) current pose
% EEp : EE goal position
% EEo : EE goal orientation (given by Rotation matrix specified in the GUI)
% q : current configuration
% dh : DH parameters table


% Computes the current EE pose from the current configuration
T = modele_geom(dh,q);

%Position error : goal minus current
dP = EEp - T(1:3,4);

%Current rotation Matrix 
Rc=T(1:3,1:3);

%Compute Euler Angles from Rotation Matrix
EAc=MatrixRotation_to_EulerAngles(Rc);%the current one
EAg=MatrixRotation_to_EulerAngles(EEo);% and the goal one

%compute the Error orientation (Euler Angles Error Vector) : goal minus current
dO=(EAg-EAc)';

end

