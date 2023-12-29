function IKGoCallback(hObject, eventdata)

global STOP_IT

EEoGoalLoc = evalin('base','EEoGoal');
EEpGoalLoc = evalin('base','EEpGoal');

qLoc = evalin('base','q');
dhLoc = evalin('base','dh');
hLoc = evalin('base','h');
h_qLoc = evalin('base','h_q');
h_TLoc = evalin('base','h_T');

% Set precision reuqirements 
epsp = 0.001; % 1 mm precision for the position
epso = 0.1; % 0.1 rad precision for the angle error (see Axis/Angle representation)

% Set the Correction Gains and parameters
t =  500; % nb. of steps to travel from here to goal
Kp = 0.01*eye(6);
%Kp = zeros(6,6);

% Initial conditions for the errors
dP = inf;
dR = inf;

% inverse velocity kinematics param
mode = 0; % No secondary constraint : 0

% Some trick
count = 0;
STOP_IT = 0;

while( (norm(dP) > epsp || norm(dR) > epso) && ~STOP_IT)
    
    % Compute the operational error
    [dP,dR,T] = OpError(EEpGoalLoc,EEoGoalLoc,qLoc,dhLoc);

    if (count == 0)
        init_dP = dP;
        init_dO = dR;
        count = 1;
    end
    
    % Compute the operational velocity set point
    dX = OpCorr(init_dP,init_dO,dP,dR,Kp,t);

    % inverse velocity kinematics
    dq = IVK_RX90(dX,T,qLoc,dhLoc,mode);
    
    % Integrate (simplified simulation)
    qLoc = simul(qLoc,dq);
    
    MvtRobot(dhLoc,hLoc,qLoc);
    
    % Update the display of the current config and current EE pose
    TLoc = modele_geom(dhLoc,qLoc);    
    updateVal(qLoc,h_qLoc,TLoc,h_TLoc);

    assignin('base','q',qLoc);
    pause(0.01);
end

end
