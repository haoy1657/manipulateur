function R=EulerAngles_to_MatrixRotation(EA)

% This function computes the rotation matrix from a given Euler Angles
% xy'z" vector

phi=EA(1);
theta=EA(2);
psi=EA(3);

R= [                              cos(psi)*cos(theta),                             -cos(theta)*sin(psi),           sin(theta);
            cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta), cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta), -cos(theta)*sin(phi);
            sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta), cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta),  cos(phi)*cos(theta)];
end