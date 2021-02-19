
u = udp('129.127.29.176',2390);

fopen(u);
while 1
    
data = input('data = ');
 


fwrite(u,data);
end