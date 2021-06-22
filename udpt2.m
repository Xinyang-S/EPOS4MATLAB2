clear u1 u2 u3 u4 u_force
u4 = udpport("LocalPort",1000,'TimeOut',100);
u2 = udpport("LocalPort",3000);
echoudp('on',3000)
while 1
    tic
    flush(u2)
    write(u2, 'asdfasdfasdfasdfasdf', 'string','LocalHost', 4000)
    toc
end
echoudp('off')