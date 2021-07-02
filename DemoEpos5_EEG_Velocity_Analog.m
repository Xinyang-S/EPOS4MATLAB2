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
trials=5; % change to 5 if testing, or 50 for eeg experiment
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

Motor1.SetWithAnalog(1,3500); % untrigger, analog output pin, analog mV value, max 4000
%move motor
while (trialNum<trials)
    Motor1.SetWithAnalog(2,encTomV(startPositionE, Motor1.ActualPosition)); % degrees*10, analog output pin, analog mV value, max 4000
    c = clock;
    clockStart = c(4)*3600+c(5)*60+c(6);
    if (clockStart > clockStartPrev + waitBetweenStims)
        trialNum = trialNum + 1;
        clockStartPrev = clockStart;
        Motor1.EnableNode;
        startPosition = Motor1.ActualPosition;
        c = clock;
        clockNew = c(4)*3600+c(5)*60+c(6);
        clockPrev = clockNew; 
        Motor1.SetWithAnalog(1,0); % trigger, analog output pin, analog mV value, max 4000
        Motor1.SetWithAnalog(1,3500); % untrigger, analog output pin, analog mV value, max 4000
        
        while(clockNew < (clockPrev + wait) && Motor1.ActualPosition < startPosition + maxPosShiftP && Motor1.ActualCurrent < maxCurrent)    
            Motor1.SetWithAnalog(2,encTomV(startPositionE, Motor1.ActualPosition)); % degrees*10, analog output pin, analog mV value, max 4000
            c = clock;
            clockNew = c(4)*3600+c(5)*60+c(6);
            Motor1.MotionInVelocity(velocityP);
        end
        Motor1.MotionInVelocity(0);

        c = clock;
        clockNew = c(4)*3600+c(5)*60+c(6);
        clockPrev = clockNew; 
        while(clockNew < (clockPrev + wait) && Motor1.ActualPosition > startPosition + maxPosShiftN && Motor1.ActualCurrent < maxCurrent)    
            Motor1.SetWithAnalog(2,encTomV(startPositionE, Motor1.ActualPosition)); % degrees*10, analog output pin, analog mV value, max 4000
            c = clock;
            clockNew = c(4)*3600+c(5)*60+c(6);
            Motor1.MotionInVelocity(velocityN);
        end
        Motor1.DisableNode;
        disp(trialNum)
    end
end
delete(Motor1);
disp('Verity Pulse Extension Complete')

%%
%Left Hand, Flexion, 'Verity' Pulse
clc
clear

%variables
wait = 0.1;
maxCurrent = 3000;
maxPosShiftP = 1000;
maxPosShiftN = -1000;
velocityP = 300;
velocityN = -300;
trials=50; % change to 5 if testing, or 50 for eeg experiment
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

%move motor
while (trialNum<trials)
    Motor1.SetWithAnalog(2,encTomV(startPositionE, Motor1.ActualPosition)); % degrees*10, analog output pin, analog mV value, max 4000
    c = clock;
    clockStart = c(4)*3600+c(5)*60+c(6);
    if (clockStart > clockStartPrev + waitBetweenStims)
        trialNum = trialNum + 1;
        clockStartPrev = clockStart;
        Motor1.EnableNode;
        startPosition = Motor1.ActualPosition;
        c = clock;
        clockNew = c(4)*3600+c(5)*60+c(6);
        clockPrev = clockNew; 
        Motor1.SetWithAnalog(1,0); % trigger, analog output pin, analog mV value, max 4000
        Motor1.SetWithAnalog(1,3500); % untrigger, analog output pin, analog mV value, max 4000
        while(clockNew < (clockPrev + wait) && Motor1.ActualPosition > startPosition + maxPosShiftN && Motor1.ActualCurrent < maxCurrent)    
            Motor1.SetWithAnalog(2,encTomV(startPositionE, Motor1.ActualPosition)); % degrees*10, analog output pin, analog mV value, max 4000
            c = clock;
            clockNew = c(4)*3600+c(5)*60+c(6);
            Motor1.MotionInVelocity(velocityN);
        end
        Motor1.MotionInVelocity(0);

        c = clock;
        clockNew = c(4)*3600+c(5)*60+c(6);
        clockPrev = clockNew;
        while(clockNew < (clockPrev + wait) && Motor1.ActualPosition < startPosition + maxPosShiftP && Motor1.ActualCurrent < maxCurrent)    
            Motor1.SetWithAnalog(2,encTomV(startPositionE, Motor1.ActualPosition)); % degrees*10, analog output pin, analog mV value, max 4000
            c = clock;
            clockNew = c(4)*3600+c(5)*60+c(6);
            Motor1.MotionInVelocity(velocityP);
        end
        Motor1.DisableNode;
        disp(trialNum)
    end
end

delete(Motor1);
disp('Verity Pulse Flexion Complete')

%%
%read encoder

Motor1 = Epos4(0,0);
Motor1.ClearErrorState;
Motor1.DisableNode;
Motor1.SetOperationMode( OperationModes.ProfileVelocityMode );
Motor1.EnableNode;
Motor1.ClearErrorState;
startPositionE=Motor1.ActualPosition;
while(true)
%     positionNow = (startPositionE-Motor1.ActualPosition)*90/6400 + 100; %degrees, neutral = ~180
%     if (positionNow > 399)
%         positionNow = 399; %max based on analog pin
%     end
%     if (positionNow < 0)
%         positionNow = 0; %min based on analog pin
%     end
    c = clock;
    clockNew = c(4)*3600+c(5)*60+c(6);
    Motor1.SetWithAnalog(2, (sin(clockNew)+1)*2000);
    disp((sin(clockNew)+1)*2000);
    %Motor1.SetWithAnalog(2,encTomV(startPositionE, Motor1.ActualPosition)); % degrees*10, analog output pin, analog mV value, max 4000
end
