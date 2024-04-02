function ekfb=SAB_New_MKMCS(ekfb,acc,mag)

%Q=ekfb.Q;
g=ekfb.g;
h=ekfb.h;
FMmatrix=ekfb.F;
statef_=ekfb.statef_;
statef=ekfb.statef;
covf_=ekfb.covf_;
covf=ekfb.covf;
% stored data 
n=4;
len=length(ekfb.statef_);
covb=zeros(n,n,len);
stateb=zeros(n,len);
for i=len:-1:1
    if(i==len)
        x=statef(:,i);
        P=covf(:,:,i);
    else
    % 
    F=FMmatrix(:,:,i);
    K=covf(:,:,i)*F'/(covf_(:,:,i+1));
    x=statef(:,i)+K*(x-statef_(:,i+1));
    P=covf(:,:,i)-K*(covf_(:,:,i+1)-P)*K';
    end

    %% innovation
    % Accelerometer data
    ax = -acc(:,1); ay = -acc(:,2); az = -acc(:,3);
    % Magnetometer data
    hx = mag(:,1); hy = mag(:,2); hz = mag(:,3);
    q1=x(1);
    q2=x(2);
    q3=x(3);
    q4=x(4);
    C=[q1^2-q2^2-q3^2+q4^2 2*(q1*q2+q3*q4) 2*(q1*q3-q2*q4)
        2*(q1*q2-q3*q4) -q1^2+q2^2-q3^2+q4^2 2*(q2*q3+q1*q4)
        2*(q1*q3+q2*q4) 2*(q2*q3-q1*q4) -q1^2-q2^2+q3^2+q4^2]; % eq.24
    z_predict=[C zeros(3); zeros(3) C]*[0;0;g;h];
    z=[ax(i);ay(i);az(i);hx(i);hy(i);hz(i)]; %measurement vector
    ze=z-z_predict; % innovation
    %% store
    ZE(:,i)=ze;
    stateb(:,i)=x';
    covb(:,:,i)=P;
end

ekfb.stateb=stateb;
ekfb.covb=covb;
ekfb.ZE=ZE;
end