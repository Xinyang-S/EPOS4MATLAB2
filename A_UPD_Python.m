clear u_force
clear u_py
%addpath C:\Users\96387\AppData\Roaming\MathWorks\MATLAB Add-Ons\Functions\Matlab Python UDP Socket
%u4 = udpport("LocalPort",1000,'TimeOut',100);
u_force = udpport('LocalPort',1262);
%u_py = pyUDPsocket('LocalPort',1250)
%u_force = udp('127.0.0.1',1249,'LocalPort',1250);

%fopen(u_force);
A_array = [];
c = clock;
starttime = c(4)*3600 + c(5)*60 + c(6);
Currenttime = starttime;
while Currenttime < starttime+10
    c = clock;
    Currenttime = c(4)*3600 + c(5)*60 + c(6);
    %flush(u_force)
    %app_data = read(u4,10,'string');
    tic
    A = read(u_force,1,'single')
    %A = fscanf(u_force)
    toc
    A_array = [A_array A];
    %A = uint8(A)5
    %A = swapbytes(typecast(uint8([65 227 216 168]),'single'))
end
%fclose(u_force)