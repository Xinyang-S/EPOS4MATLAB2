clc
clear
%variables
moveDistance = 4000;
wait = 4.0;
Motor1 = Epos4(0,0);
Motor1.ClearErrorState;
Motor1.EnableNode;
Motor1.DisableNode;
Motor1.EnableNode;

%Motor1.SetOperationMode( OperationModes.CurrentMode );
Motor1.SetOperationMode( OperationModes.ProfilePositionMode );
Motor1.MotionWithCurrent( 10000 );
Motor1.EnableNode;
Motor1.ClearErrorState;
startPosition = Motor1.ActualPosition;
disp(Motor1.ActualPosition) 
%Motor1.MotionWithCurrent( 4000 );
disp(Motor1.GetOperationMode)
    c = clock;
    clockNew = c(4)*3600+c(5)*60+c(6);
    clockPrev = clockNew; 
while(clockNew < clockPrev + wait)
    if (Motor1.IsInErrorState)
    Motor1.ClearErrorState
    end
    %Motor1.MotionWithCurrent( 4000 );
    actualPosition = Motor1.ActualPosition;
    desiredPosition = startPosition+(clockNew-clockPrev)*moveDistance/wait;
    commandPosition = (desiredPosition - actualPosition)*1 + desiredPosition; %gGain
    Motor1.MotionInPosition(commandPosition ,4000,4000,1); %Motor1.MotionInPosition( DesiredPosition, ProfileVelocity, ProfileAcceleration, AbsoluteOrRelativePosition  ) 
    c = clock;
    clockNew = c(4)*3600+c(5)*60+c(6);
    dispA = [desiredPosition actualPosition];
    disp(dispA)
end

disp(Motor1.ActualPosition)
    c = clock;
    clockNew = c(4)*3600+c(5)*60+c(6);
    clockPrev = clockNew; 
while(clockNew < clockPrev + wait)
    if (Motor1.IsInErrorState)
    Motor1.ClearErrorState
    end
    %Motor1.MotionWithCurrent( 4000 );
    actualPosition = Motor1.ActualPosition;
    desiredPosition = startPosition+moveDistance-(clockNew-clockPrev)*moveDistance/wait;
    commandPosition = (desiredPosition - actualPosition)*1 + desiredPosition; %gGain
    Motor1.MotionInPosition(commandPosition ,4000,4000,1); %Motor1.MotionInPosition( DesiredPosition, ProfileVelocity, ProfileAcceleration, AbsoluteOrRelativePosition  ) 
    c = clock;
    clockNew = c(4)*3600+c(5)*60+c(6);
    dispA = [desiredPosition actualPosition];
    disp(dispA)
end
disp(Motor1.ActualPosition)

%Motor1.WaitUntilDone(3000);
%Motor1.MotionInPosition(30,2,20,1);
%Motor1.WaitUntilDone(10000);
delete(Motor1);
disp('done')
