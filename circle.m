% x = 0:1:60;
% 
% y=sqrt(900-x*x');
% figure
% plot(x,y)
clear
close
r=350;
x=0;
y=0;
th = 0:pi/800:pi;
xunit = r * cos(th) + x;
yunit = r * sin(th) + y;
% figure( 'Position', [10 10 640 480])
% plot(xunit, yunit);
% xlim=([-300 300]);
% ylim=([-300 300]);
% axis square
figure( 'Position', [10 10 640 480])
xunit=xunit(201:601);
yunit=yunit(201:601);
plot(xunit, yunit,'color','black');
axis equal