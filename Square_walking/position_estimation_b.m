function POS=position_estimation_b(vel,p_init,Pvm)

%
% forward velocity
len=length(vel);
pos=zeros(len,3);
dt=1/400;
Ppm=zeros(len,1);

%
for i=len:-1:1
    if(i==len)
        pos(len,:)=p_init;
        Ppm(len)=0;
    else
        pos(i,:)=pos(i+1,:)- vel(i,:)*dt;
        Ppm(i)=Ppm(i+1)+dt^2*Pvm(i);
    end
    %
end


POS.pos=pos;
POS.Ppm=Ppm;

end