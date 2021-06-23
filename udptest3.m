%u2 = udp("LocalHost",1250);
% u2 = udp('127.0.0.1','localport', 1263);

u2 = udp("LocalHost",3000);
fopen(u2);
    c = clock();
    clockNew = (c(4)*3600 + c(5)*60 + c(6));
    clockStart = clockNew;
while 1
    tic
    c = clock();
    clockNew = c(4)*3600 + c(5)*60 + c(6);
    new = single(clockNew-clockStart);
    newV = typecast(new, 'int8');
    fwrite(u2, newV, 'int8')
    pause(0.001);
    toc
end
fclose(u2);
echou32p('off')