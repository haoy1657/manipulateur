function dX = OpCorr(init_dP,init_dO,dP,dO,Kp,t);

% Return the operational velocity set point (consigne en français)
% init_dP : initial EE position error
% init_dO : initial EE orientation error
% Kp : Proportional gain
% t : nb. of steps to travel from the home pose to the goal pose

%The first term defines a constant velocity in the operational space and the
%second one add a term as function of the current error

dX = diag(sign([dP;dO]))*diag(sign([init_dP;init_dO]))*[init_dP;init_dO]/t + Kp*[dP;dO];

%dX = [init_dP;init_dO]/t + Kp*[dP;dO];

end