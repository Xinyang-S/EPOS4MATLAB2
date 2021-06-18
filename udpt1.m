clear u1 u2 u3 u4
u3 = udpport("LocalPort",2000);
u1 = udpport("LocalPort",4000); % open udp for FES pw from simulink, clear port if error
f2 = [];
f1 = 0;

while(true)
    
    write(u3,[1234.1231],"double","LocalHost",1000);
%     f1 = read(u1,1,"double");  % ensure buffer is multiple of number of electrodes used 
%     f2 = [f2 f1];
end

    