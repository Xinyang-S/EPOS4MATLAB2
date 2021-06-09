clear
close all
% load('targetTrack.mat')
% figure
% 
% subplot(3,1,1)
% plot(targetTrack.wristPos.trial1)
% hold on
% plot(-targetTrack.targetPos.trial1)
% 
% subplot(3,1,2)
% plot(targetTrack.wristPos.trial2)
% hold on
% plot(-targetTrack.targetPos.trial2)
% hold on
% subplot(3,1,3)
% plot(targetTrack.wristPos.trial3)
% hold on
% plot(-targetTrack.targetPos.trial3)
% legend('wrist','target')
% 
% sgtitle('Target tracking') 
% load('positionTrack.mat')
% figure
% 
% subplot(3,1,1)
% plot(positionTrack.wristPos.trial1)
% hold on
% plot(-positionTrack.targetPos.trial1)
% 
% subplot(3,1,2)
% plot(positionTrack.wristPos.trial2)
% hold on
% plot(-positionTrack.targetPos.trial2)
% hold on
% subplot(3,1,3)
% plot(positionTrack.wristPos.trial3)
% hold on
% plot(-positionTrack.targetPos.trial3)
% legend('wrist','target')
% 
% sgtitle('Position match') 

% load('torque.mat')
% torque.level.trial11=torque.level.trial11/1000;
% torque.level.trial25=torque.level.trial25/1000;
% torque.level.trial40=torque.level.trial40/1000;
% figure
% 
% subplot(3,1,1)
% plot(torque.wristPos.trial11)
% hold on
% plot(torque.level.trial11)
% 
% subplot(3,1,2)
% plot(torque.wristPos.trial25)
% hold on
% plot(torque.level.trial25)
% hold on
% subplot(3,1,3)
% plot(torque.wristPos.trial40)
% hold on
% plot(torque.level.trial40)
% legend('wrist','target')
% 
% sgtitle('Torque stabilisation') 
% 
% load('velocity.mat')
% figure
% 
% subplot(3,1,1)
% plot(velocity.trial1)
% subplot(3,1,2)
% plot(velocity.trial2)
% subplot(3,1,3)
% plot(velocity.trial3)
% 
% 
% 
% sgtitle('Velocity') 


% 
% load('velocity.mat')
% legendary=[];
% figure
% for i=1:1:8
% name=strcat('trial',num2str(i));
% 
% plot(smoothdata(velocity.(name),'gaussian',45),'DisplayName',name)
% hold on
% 
% end
%  legend show
%  
 
 
 
% figure
% plot(sin([1:0.01:10]))
% hold on
% plot(sin([1:0.005:10]))

A=[11 12 13 14 15];

sample=randi(size(A));
strength=A(sample)
A(sample)=[];



