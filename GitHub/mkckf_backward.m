function kfb=mkckf_backward(kf,kff)


% init
Q=kf.Q;
R=kf.R;
F=kf.F;
H=kf.H;
len=kf.len;
n=kf.n;
sigma_pb=kf.sigma_pb;
sigma_qb=kf.sigma_qb;

statef_=kff.statef_;
statef=kff.statef;
covf_=kff.covf_;
covf=kff.covf;

% stored data 
covb=zeros(n,n,len);
stateb=zeros(n,len);

for i=len:-1:1
    if(i==len)
        x=statef(:,i); % measurement
        P=covf(:,:,i); % covariance 
    else
    % 
    % chol decomposition
    bp = chol(kff.covf(:,:,i),'lower') ;
    bq = chol(Q,'lower') ;
    %bp=sqrt(kff.covf(:,:,i));
    %bq=sqrt(Q);
    cnt=3;  
    num=3;
    z=x;    % x_{k+1|k+1}^{b} 
    while(num>0)
        if(num==cnt)
          x_tlast=statef(:,i); % prediction 
        else  
          x_tlast=x_t; 
        end
        num=num-1;               %
        dp= bp\statef(:,i);
        wp= bp\x_tlast;
        ep=dp-wp;
        % 
        dq= bq\z;
        wq= bq\F*x_tlast;
        eq=dq-wq;
        %  P_ and R
        Cp=diag(exp(-ep.*ep./(2*sigma_pb.*sigma_pb)));
        Cq=diag(exp(-eq.*eq./(2*sigma_qb.*sigma_qb)));
        for kk=1:n
            if(Cp(kk,kk)<0.0001)
                Cp(kk,kk)=0.0001;
            end
        end
        for kk=1:n
            if(Cq(kk,kk)<0.0001)
                Cq(kk,kk)=0.0001;
            end   
        end
        P_1=bp/Cp*bp';
        Q_1=bq/Cq*bq';

        P_1_=F*P_1*F'+Q_1;
        K_1= P_1*F'/P_1_;
        x_t=statef(:,i) + K_1 *(z-statef_(:,i+1));
        %
        % stored data
        if(num==cnt-1)
        eq_matrix(:,i)=eq;
        end
        if(num==cnt-2)
        ep_matrix(:,i)=ep;
        end
        xe(cnt-num,i)=norm(x_t-x_tlast)/(norm(x_tlast)+0.0001);
        if(xe(cnt-num,i)<0.00001)
            break
        end
        threshold(cnt-num,i)=xe(cnt-num,i);
    end 
    x=x_t;
    %K=covf(:,:,i)*F'/(covf_(:,:,i+1));
    %x=statef(:,i)+K*(x-statef_(:,i+1));
    P=covf(:,:,i)-K_1*(covf_(:,:,i+1)-P)*K_1';
    end
    
    stateb(:,i)=x';
    covb(:,:,i)=P;

end

kfb.stateb=stateb;
kfb.covb=covb;
kfb.eq_matrix=eq_matrix;
kfb.ep_matrix=ep_matrix;

end