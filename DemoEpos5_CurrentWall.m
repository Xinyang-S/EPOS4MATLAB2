%%
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
KW = 7; %8 //spring
DW = 0; %1.1 //damper
MW = 0.01; %0.001 //wall

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
    %if (clockStart > posClockPrev + 0.001)
        posError = (startPosition+100) - Motor1.ActualPosition;
        posErrorDiff = (posError - posErrorPrev);
        posErrorDiffDiff = (posErrorDiff - posErrorDiffPrev);
        %velocity = Motor1.ActualVelocity;
        posClockDiff = clockStart - posClockPrev;
        if (Motor1.ActualPosition > startPosition + 100)
            currentK = KW*posError;
            currentD = DW*posErrorDiff;
            currentM = MW*posErrorDiffDiff;
            %current = KW*posError + DW*velocity;
            if (currentK > 0)
               currentK = 0;
            end
            if (currentD > 0)
               currentD = 0;
            end
            if (currentM > 0)
               currentM = 0;
            end
            current = currentK + currentD + currentM;
            if (current > 0)
               current = 0;
            elseif (current < currentMaxN)
               current = currentMaxN;
               disp('Wall Broke!')
            else
               Motor1.MotionWithCurrent(current);
            end
        else
            Motor1.MotionWithCurrent(0);
        end
        posErrorPrev = posError;
        posErrorDiffPrev = posErrorDiff;
        posClockPrev = clockStart;
    %end
end

delete(Motor1);
disp('Complete!')