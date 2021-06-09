m%%
%Emergency Stop - Run Section (may need to click Pause and Quit Debugging first)
delete(Motor1);
disp('estop')

%%
% Haptic wall (still a bit bouncy)
% Don't print anything continuously during runtime
clc
clear

%variables
hold = 10;
currentMaxP = 10000;
currentMaxN = -10000;
KW = 4; %8 //spring
DW = 10; %1.1 //damper
MW = 0; %0.001 //wall

%initialize
Motor1 = Epos4(0,0);
Motor1.ClearErrorState;
Motor1.DisableNode;
Motor1.SetOperationMode( OperationModes.CurrentMode );
Motor1.EnableNode;
Motor1.ClearErrorState;

c = clock;
clockStart = c(4)*3600+c(5)*60+c(6);
clockStartPrev = clockStart;
posClockPrev = clockStart;
startPosition = Motor1.ActualPosition;
posError = 0;
posErrorPrev = 0;
posErrorDiff = 0;
posErrorDiffPrev = 0;
posErrorDiffDifff = 0;
    
while (clockStart < clockStartPrev + hold)
    c = clock;
    clockStart = c(4)*3600+c(5)*60+c(6);
    
    %calculate distance into wall
    %disp([Motor1.ActualPosition startPosition])
    posError = Motor1.ActualPosition - (startPosition+100);
    disp(posError/6400*90);
%    
%     calculate error velocity
%     posErrorDiff = posError - posErrorPrev;
%     posClockDiff = clockStart - posClockPrev;
%    
%     if inside wall - current = KW*distance + DW*Velocity
%     if (Motor1.ActualPosition > startPosition + 100)
%         current = -1*(KW*posError + DW*posErrorDiff);        
%         if (current > currentMaxP)
%            current = currentMaxP;
%            disp('Wall Too Strong!')
%         elseif (current < currentMaxN)
%            current = currentMaxN;
%            disp('Wall Broke!')
%         else
%            Motor1.MotionWithCurrent(current);
%         end
%     else
%         Motor1.MotionWithCurrent(0);
%     end
%     
%     posErrorPrev = posError;
%     posErrorDiffPrev = posErrorDiff;
%     posClockPrev = clockStart;
end

delete(Motor1);
disp('Complete!')
