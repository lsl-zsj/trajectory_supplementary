function VEL=velocity_estimation(acc,v_init,stdAcc,index)
%
% forward velocity
len=length(acc);
vel=zeros(len,3);
dt=1/400;
Pv=zeros(len,1);
for i=1:len
    if(i==1)
        vel(i,:)=v_init;
    else
        vel(i,:)=vel(i-1,:)+ acc(i-1,:)*dt;
    end
    %
    Pv(i)=(i-1)*(len-i)/(len-1)*dt^2*stdAcc^2;
end
% velocity correction
velEnd=vel(end,:);
velStep=velEnd/(len-1);
velCorx=0:velStep(1):velStep(1)*(len-1);
velCory=0:velStep(2):velStep(2)*(len-1);
velCorz=0:velStep(3):velStep(3)*(len-1);
velCor=[velCorx',velCory',velCorz'];
velZUPT=vel-velCor;

VEL.vel=velZUPT;
VEL.Pv=Pv;
VEL.index=index;
end