function [P]=gyroscope_norm_loop(mydata,freeAcc,fs,count)

%
lshift=12;
rshift=12;
swduring=40;
if(fs==100)
   lshift=lshift/4;
   rshift=rshift/4; 
   swduring=swduring/4;
end

acc=mydata(:,2:4);
gyro=mydata(:,5:7);
mag=mydata(:,8:10);
len=length(gyro);
gyroNorm=zeros(len,1);
gyroNormFlag=zeros(len,1);
for i=1:len
    gyroNorm(i)=norm(gyro(i,:));
    accNorm(i)=norm(acc(i,:));
    magNorm(i)=norm(mag(i,:));
end
% threshold
Thes1=0.4;
for i=2:len
    if(gyroNorm(i)<=Thes1)
        % static
        gyroNormFlag(i)=0;
    else
        % move
        gyroNormFlag(i)=1;
    end
end

% correction
indone=find(gyroNormFlag==1);
gyroNormFlagc=gyroNormFlag;
for i=2:length(indone)
    if(indone(i)-indone(i-1)<swduring)
        gyroNormFlagc(indone(i-1):indone(i))=1;
    end
end


if(1)
    %filter interval: exculde the impulsive move
    gyroNormFlagcc=gyroNormFlagc;
    inv=0;
    gyroinv=[];
    for i=2:len
        if(gyroNormFlagcc(i-1)==0&&gyroNormFlagcc(i)==1)
            inv=inv+1;
            gyroinv(inv,1)=i; % start moving
        elseif(gyroNormFlagcc(i-1)==1&&gyroNormFlagcc(i)==0)
            gyroinv(inv,2)=i; % stop moving
        end
    end
    % if move intervel is less than 20, setting it as moveless
    % for m=1:length(gyroinv)
    %     if((gyroinv(m,2)-gyroinv(m,1))<20)
    %     gyroNormFlagcc(gyroinv(m,1):gyroinv(m,2))=0;
    %     end
    % end
    gyroNormFlagc=gyroNormFlagcc;
end

% P gives the segmentations of moveless. The first column stores the start
% of moveless while the second stores to end of moveless.
n=length(gyroNormFlagc);
j=1;
for i=1:n
    if i==1
        % moveless
        if gyroNormFlagc(i)==0
            P(j,1)=i;
        end
    else
        % moveless to moving
        if gyroNormFlagc(i-1)==0&&gyroNormFlagc(i)==1
            P(j,2)=i-lshift; % start move , 10 is specified shift region
            j=j+1;
        end
        % moving to moveless
        if gyroNormFlagc(i-1)==1&&gyroNormFlagc(i)==0
            P(j,1)=i+rshift;  % end of move
        end
  
    end
end

% recover the swing stance judgement
[seg,col]=size(P);
gyroNormFlagcc=gyroNormFlagc;
gyroNormFlagcc(1:P(1,2))=0;
gyroNormFlagcc(P(end,1):end)=0;
for i=1:seg-1
    swind=P(i,2):P(i+1,1); % swing index
    stind=P(i+1,1):P(i+1,2); % stance index
    gyroNormFlagcc(swind)=1;
    gyroNormFlagcc(stind)=0;    
end

fs=400;
t=0:1/fs:(len-1)*1/fs;

%% 
if(count==1)
figure
hold on
plot(freeAcc(:,1),'blue','lineWidth',0.5)
plot(freeAcc(:,2),'black','lineWidth',0.5)
plot(freeAcc(:,3),'m','lineWidth',0.5)
plot(gyroNormFlagcc(:)*8,'--red','lineWidth',1)
plot(gyroNorm(:),'--g','lineWidth',2)
xlabel('time (s)','interpreter','latex')
ylabel('acc (m/s$^{2}$) or gyr (rad/s)','interpreter','latex')
legend('Free Acc X','Free Acc Y','Free Acc Z','Flag','Gyr Norm','interpreter','latex')
set(gca,'fontSize',16)
ylim([-20 30])
set(gcf,'position',[100 100 750 600])
box on
end

end
