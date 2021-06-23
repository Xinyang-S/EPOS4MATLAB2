clear u1 u2 u3 u4
u3 = udpport("LocalPort",2000);
u1 = udpport("LocalPort",1250,'timeout',100); % open udp for FES pw from simulink, clear port if error
f2 = [];
f1 = 0;
%write(u3, strcat(num2str(6), '0','0',num2str(1),'0','0','2'), 'string','LocalHost', 1000)
        
while(true)
    flush(u1);
    tic
    script_data = read(u1,1,"single")  % ensure buffer is multiple of number of electrodes used 
    toc
end

    