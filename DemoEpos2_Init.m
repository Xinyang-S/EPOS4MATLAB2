clc
clear

Motor1 = Epos4(0,0);
a = 'hi'
Motor1.EnableNode;
a = 'hi2'
Motor1.ClearErrorState;
a= 'hi3'
Motor1.MotionInPosition(0,2,20,1);
a= 'hi4'
Motor1.WaitUntilDone(10000);

Motor1.MotionInPosition(30,2,20,1);
Motor1.WaitUntilDone(10000);

Motor1.MotionInPosition(-30,2,20,1);
Motor1.WaitUntilDone(10000);

Motor1.MotionInPosition(0,2,20,1);
Motor1.WaitUntilDone(10000);

delete(Motor1);