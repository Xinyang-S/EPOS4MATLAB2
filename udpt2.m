clear u1 u2 u3 u4 u_force
u4 = udpport("LocalPort",1000,'TimeOut',100);
u2 = udp("LocalHost",3000);
 
%u = udp('127.0.0.1','localport', 6000, 'remoteport',8000);
fopen(u2);
while 1

    fwrite(u2, 2.12321, 'single')
    tic
    fread(u2,1)
    toc
end
fclose(u2);
echoudp('off')