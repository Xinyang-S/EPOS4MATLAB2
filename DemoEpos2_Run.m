function y = DemoEpos2_Run(u)
Motor1.MotionInPosition(0,2,20,1);
Motor1.WaitUntilDone(10000);

Motor1.MotionInPosition(30000,2,20,1);
Motor1.WaitUntilDone(10000);

Motor1.MotionInPosition(-30000,2,20,1);
Motor1.WaitUntilDone(10000);

Motor1.MotionInPosition(0,2,20,1);
Motor1.WaitUntilDone(10000);
y=u;
%delete(Motor1);