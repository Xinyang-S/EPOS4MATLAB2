u=udp('127.0.0.1', 3000);
fopen(u);
while(true)
    f=GetForce();
    if(~isempty(f))
        fwrite(u, f)   
    end
end
