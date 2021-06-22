clear u1 u2 u3 u4 u_force
% u4 = udpport("LocalPort",1000,'TimeOut',100);
% u2 = udpport("LocalPort",3000);
% echoudp('on',3000)
u1 = udp("127.0.0.1",4000);
fopen(u1);
while 1
    tic
%     flush(u2)
    fread(u1,1)
    toc
end
% echoudp('off')