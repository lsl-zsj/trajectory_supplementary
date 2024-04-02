function kfb=kf_backward(kf,kff)

% init
Q=kf.Q;
R=kf.R;
F=kf.F;
H=kf.H;
len=kf.len;
n=kf.n;

statef_=kff.statef_;
statef=kff.statef;
covf_=kff.covf_;
covf=kff.covf;

% stored data 
covb=zeros(n,n,len);
stateb=zeros(n,len);

for i=len:-1:1
    if(i==len)
        x=statef(:,i);
        P=covf(:,:,i);
    else
    % 
    K=covf(:,:,i)*F'/(covf_(:,:,i+1));
    x=statef(:,i)+K*(x-statef_(:,i+1));
    P=covf(:,:,i)-K*(covf_(:,:,i+1)-P)*K';
    end
    
    stateb(:,i)=x';
    covb(:,:,i)=P;

end

kfb.stateb=stateb;
kfb.covb=covb;


end