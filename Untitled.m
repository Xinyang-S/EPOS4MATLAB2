%%
%Emergency Stop - Run Section (may need to click Pause and Quit Debugging first)
delete(Motor1);
disp('estop')
%Rules - Never test new code with hand strapped in
%Todo: 
%output trigger signal to brain recorder
% 8 channel cable 520853 - red - digital output 1 - general purpose a
% ... yellow - digital input 1 - general purpose D

%create reliable motor position, current, torque controllers
%record brain recorder signal in matlab in real time

%%
%Left Hand, Extension, 'Verity' Pulse
clc
clear

%variables 
wait = 0.1;
waitBetweenStims = 4.5;
maxCurrent = 3000;
maxPosShiftP = 1000;
maxPosShiftN = -1000;
velocityP = 300;
velocityN = -300;
trials=20; % change to 5 if testing, or 50 for eeg experiment
trialNum=0;

%initialize
Motor1 = Epos4(0,0);
Motor1.ClearErrorState;
Motor1.DisableNode;
Motor1.SetOperationMode( OperationModes.ProfileVelocityMode );
Motor1.EnableNode;
Motor1.ClearErrorState;
c = clock;
clockStart = c(4)*3600+c(5)*60+c(6);
clockStartPrev = clockStart - 4.5;
startPositionE = Motor1.ActualPosition;
while (true)
    c = clock;
    clockStart = c(4)*3600+c(5)*60+c(6);
    encTomV(startPositionE, Motor1.ActualPosition)
    Motor1.SetWithAnalog(1,encTomV(startPositionE, Motor1.ActualPosition)); % degrees*10, analog output pin, analog mV value, max 4000
    Motor1.SetWithAnalog(2,encTomV(startPositionE, Motor1.ActualPosition)); % degrees*10, analog output pin, analog mV value, max 4000
end
delete(Motor1);
disp('Verity Pulse Extension Complete')
%encTomV(startPositionE, Motor1.ActualPosition)

