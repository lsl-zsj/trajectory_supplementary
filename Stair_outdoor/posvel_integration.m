function [pos,vel]=posvel_integration(p_init,v_init,acc,fs)
% numerical integrate the acceleration to obtian the velocity and position
len=length(acc);
pos=zeros(len,3);
vel=zeros(len,3);
dt=1/fs;
for i=1:len
    if(i==1)
        pos(i,:)=p_init;
        vel(i,:)=v_init;
    else
        vel(i,:)=vel(i-1,:)+ acc(i-1,:)*dt;
        pos(i,:)=pos(i-1,:)+ vel(i-1,:)*dt + 0.5*acc(i-1,:)*dt^2;
    end

end

end