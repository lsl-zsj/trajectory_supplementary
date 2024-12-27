function POS=position_estimation(vel,p_init,Pvm)

%
% forward velocity
len=length(vel);
pos=zeros(len,3);
dt=1/400;
Ppm=zeros(len,1);
for i=1:len
    if(i==1)
        pos(i,:)=p_init;
        Ppm(i)=0;
    else
        pos(i,:)=pos(i-1,:)+ vel(i-1,:)*dt;
        Ppm(i)=Ppm(i-1)+dt^2*Pvm(i-1);
    end
    %
end

POS.pos=pos;
POS.Ppm=Ppm;

end