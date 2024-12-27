function smoother_impulsive_measurement()

clear all
% linear tracking example with unknown input
dt=0.1;
R=0.1*[10 0;0,2];
G=[0.5 * dt ^2;
     dt];
Q=G*G';
Q=diag(diag(Q));
F=[1 dt;
     0 1];
H=[1 0; 0 1];
tsim = 300;
len=tsim/dt;
x    = zeros(2, len);
u   = zeros(1, len);          % control input
u_n   = zeros(1, len);          % control input
z    = zeros(2, len);         % measurement at hz
t    = 0:dt:tsim-dt;          % time
x0 = [0;0];
u0=0;
%% state and measurement generation
for i = 1:len
    % process
    u(:,i) = 0;    
    u_n(:,i) = u(:,i)+ randn(1); % this actually is the sequence of 0 to N-1
    if(i==1)
    x(:,i) =F*x0+G*u0;
    else
    x(:,i) = F * x(:,i-1) + G * u(:,i);   % Generate truth  u(:,i) actually is u(:,i-1)
    end
    % measurement
    % erf(1.645/sqrt(2))=0.9 erf(1.2815/sqrt(2))=0.8 erf(1.0365/sqrt(2))=0.7
    %  erf(0.8416/sqrt(2)) = 0.6

    if(abs(randn(1))<1.645)
        noise=randn(2,1);
    else
        noise=20*randn(2,1);
    end
    z(:,i) = H*x(:,i) + sqrt(R) * noise;      
end
%% kalman filter
x0=[0;0];
P0 = diag([1;1]);
kf.Q=Q;
kf.R=R;
kf.F=F;
kf.H=H;
kf.G=G;
kf.x0=x0;
kf.P0=P0;
kf.len=len;
kf.n=2;
kf.m=2;
mkc_sigma=100;
c_sigma=10;
kf=kfInstanceInit(mkc_sigma,c_sigma,kf,'measurement');
% kf forward pass
u_n=1*randn(1, len); 

kff=kf_forward(kf,u_n,z);
% kf backward pass
kfb=kf_backward(kf,kff);
% ukf forward pass
ukff=ukf_forward(kf,u_n,z);
% ukf backward pass
ukfb=kf_backward(kf,ukff);
% ckf forward pass 
ckff=ckf_forward(kf,u_n,z);
% ckf backward pass
ckfb=ckf_backward(kf,ckff);
% forward pass
mkckff=mkckf_forward(kf,u_n,z);
% backward pass
mkckfb=mkckf_backward(kf,mkckff);

%% plot and display the error
len=kf.len;
t=0:dt:(len-1)*dt;
state=x;
for i=1:len
    Pf_trace(i)=trace(mkckff.covf(:,:,i));
    Pb_trace(i)=trace(mkckfb.covb(:,:,i));
end


%% backward
figure
x1=subplot(2,1,1);
hold on
plot(t,state(1,:)-kff.statef(1,:),'color','red','LineWidth',1.0,'LineStyle','--')
plot(t,state(1,:)-ckff.statef(1,:),'color','blue','LineWidth',1.0,'LineStyle','--')
plot(t,state(1,:)-mkckff.statef(1,:),'color','black','LineWidth',1.0,'LineStyle','--')
plot(t,state(1,:)-kfb.stateb(1,:),'color','red','LineWidth',1.0,'LineStyle','-')
plot(t,state(1,:)-ckfb.stateb(1,:),'color','blue','LineWidth',1.0,'LineStyle','-')
plot(t,state(1,:)-mkckfb.stateb(1,:),'color','black','LineWidth',1.0,'LineStyle','-','Marker','diamond','MarkerIndices',1:20:length(t))
legend('kf','ckf','mkckf','kfs','ckfs','mkckfs','interpreter','latex')
ylabel('pos error','interpreter','latex')
set(gca,'fontsize',16)
x2=subplot(2,1,2);
hold on
plot(t,state(2,:)-kff.statef(2,:),'color','red','LineWidth',1.0,'LineStyle','--')
plot(t,state(2,:)-ckff.statef(2,:),'color','blue','LineWidth',1.0,'LineStyle','--')
plot(t,state(2,:)-mkckff.statef(2,:),'color','black','LineWidth',1.0,'LineStyle','--')
plot(t,state(2,:)-kfb.stateb(2,:),'color','red','LineWidth',1.0,'LineStyle','-')
plot(t,state(2,:)-ckfb.stateb(2,:),'color','blue','LineWidth',1.0,'LineStyle','-')
plot(t,state(2,:)-mkckfb.stateb(2,:),'color','black','LineWidth',1.0,'LineStyle','-','Marker','diamond','MarkerIndices',1:20:length(t))
legend('kf','ckf','mkckf','kfs','ckfs','mkckfs','interpreter','latex')
ylabel('vel error','interpreter','latex')
set(gca,'fontsize',16)
linkaxes([x1,x2],'x')
xlim([100,200]);
linkaxes([x1,x2],'x')

%% optimize the kernel bandwidth
%z(:,i) = H*x(:,i) + sqrt(R) * noise; 
mn=R^(-1/2)*(z-F*mkckfb.stateb);
mn_vec=reshape(mn,1,2*len);

global w
w=mn_vec;
% iterated optimization
SIGMA=mkc_sigma;
emiter=0;
emitermax=5;
while (emiter<emitermax)
emiter=emiter+1;  
%% E step
fun=@ko_value_mat_sigma;
x0=5;
options = optimoptions(@fminunc,'Display','iter','Algorithm','quasi-newton');
[x,fval,exitflag,output] = fminunc(fun,x0,options);
fprintf('sigma:%.3f\r\n',x);
%
mkc_sigma=abs(x);
c_sigma=10;
%% M step
kf=kfInstanceInit(mkc_sigma,c_sigma,kf,'measurement');
% forward pass
mkckff_opt=mkckf_forward(kf,u_n,z);
% backward pass
mkckfb_optb=mkckf_backward(kf,mkckff_opt);
%  
mn=R^(-1/2)*(z-F*mkckfb_optb.stateb);
mn_vec=reshape(mn,1,2*len);
w=mn_vec;
%% store the data
SIGMA=[SIGMA,mkc_sigma];
MKCKFF(emiter)=mkckff_opt;
MKCKFB(emiter)=mkckfb_optb;
end


% plot the error
figure
x1=subplot(2,1,1);
box on
hold on
plot(t,state(1,:)-kfb.stateb(1,:),'color','b','LineWidth',1.0,'LineStyle','--','Marker','diamond','MarkerIndices',1:20:length(t))
plot(t,state(1,:)-mkckfb.stateb(1,:),'color','black','LineWidth',1.0,'LineStyle','-','Marker','none','MarkerIndices',1:20:length(t))
plot(t,state(1,:)-MKCKFB(3).stateb(1,:),'color','red','LineWidth',1.0,'LineStyle','-','Marker','none','MarkerIndices',1:20:length(t))
legend('KS','MKCRTS ($\sigma_r=100$)','opt MKCRTS ($\sigma_r=1.7$)','interpreter','latex','Orientation','horizontal')
xticks([])
ylabel('$x_1-\bar{x}_1$','interpreter','latex')
set(gca,'fontsize',16)
x2=subplot(2,1,2);
hold on
box on
plot(t,state(2,:)-kfb.stateb(2,:),'color','b','LineWidth',1.0,'LineStyle','--','Marker','diamond','MarkerIndices',1:20:length(t))
plot(t,state(2,:)-mkckfb.stateb(2,:),'color','black','LineWidth',1.0,'LineStyle','-','Marker','none','MarkerIndices',1:20:length(t))
plot(t,state(2,:)-MKCKFB(3).stateb(2,:),'color','red','LineWidth',1.0,'LineStyle','-','Marker','none','MarkerIndices',1:20:length(t))
%legend('MKCKS','opt MKCKS','interpreter','latex')
ylabel('$x_2-\bar{x}_2$','interpreter','latex')
set(gca,'fontsize',16)
linkaxes([x1,x2],'x')
xlim([100,200]);
linkaxes([x1,x2],'x')
set(gcf,'position',[100 100 750 600])

%% noise distribution



%% final error plot 
figure
x1=subplot(2,1,1);
hold on
plot(t,state(1,:)-kff.statef(1,:),'color','red','LineWidth',1.0,'LineStyle','--')
plot(t,state(1,:)-ckff.statef(1,:),'color','blue','LineWidth',1.0,'LineStyle','--')
plot(t,state(1,:)-MKCKFF(5).statef(1,:),'color','black','LineWidth',1.0,'LineStyle','--')
plot(t,state(1,:)-kfb.stateb(1,:),'color','red','LineWidth',1.0,'LineStyle','-')
plot(t,state(1,:)-ckfb.stateb(1,:),'color','blue','LineWidth',1.0,'LineStyle','-')
plot(t,state(1,:)-MKCKFB(5).stateb(1,:),'color','black','LineWidth',1.0,'LineStyle','-','Marker','diamond','MarkerIndices',1:20:length(t))
legend('kf','ckf','mkckf','kfs','ckfs','mkckfs','interpreter','latex')
ylabel('pos error','interpreter','latex')
set(gca,'fontsize',16)
x2=subplot(2,1,2);
hold on
plot(t,state(2,:)-kff.statef(2,:),'color','red','LineWidth',1.0,'LineStyle','--')
plot(t,state(2,:)-ckff.statef(2,:),'color','blue','LineWidth',1.0,'LineStyle','--')
plot(t,state(2,:)-MKCKFF(5).statef(2,:),'color','black','LineWidth',1.0,'LineStyle','--')
plot(t,state(2,:)-kfb.stateb(2,:),'color','red','LineWidth',1.0,'LineStyle','-')
plot(t,state(2,:)-ckfb.stateb(2,:),'color','blue','LineWidth',1.0,'LineStyle','-')
plot(t,state(2,:)-MKCKFB(5).stateb(2,:),'color','black','LineWidth',1.0,'LineStyle','-','Marker','diamond','MarkerIndices',1:20:length(t))
legend('kf','ckf','mkckf','kfs','ckfs','mkckfs','interpreter','latex')
ylabel('vel error','interpreter','latex')
set(gca,'fontsize',16)
linkaxes([x1,x2],'x')
xlim([100,200]);
linkaxes([x1,x2],'x')
set(gcf,'position',[100 100 750 600])


%% RMSE Calculation
error.state=state;
error.kff=kff.statef;
error.kfb=kfb.stateb;
error.ckff=ckff.statef;
error.ckfb=ckfb.stateb;
error.ukff=ukff.statef;
error.ukfb=ukfb.stateb;
error.mkckff=mkckff.statef;
error.mkckfb=mkckfb.stateb;
error.mkckff_opt=MKCKFF(end).statef;
error.mkckfb_opt=MKCKFB(end).stateb;
%
error.ekff=error.state-error.kff;
error.ekfb=error.state-error.kfb;
error.eckff=error.state-error.ckff;
error.eckfb=error.state-error.ckfb;
error.eukff=error.state-error.ukff;
error.eukfb=error.state-error.ukfb;
error.emkckff=error.state-error.mkckff;
error.emkckfb=error.state-error.mkckfb;
error.emkckff_opt=error.state-MKCKFF(end).statef;
error.emkckfb_opt=error.state-MKCKFB(end).stateb;
% RMSE
error.ekff_rms=rms(error.ekff,2);
error.ekfb_rms=rms(error.ekfb,2);
error.eckff_rms=rms(error.eckff,2);
error.eckfb_rms=rms(error.eckfb,2);
error.eukff_rms=rms(error.eukff,2);
error.eukfb_rms=rms(error.eukfb,2);
error.emkckff_rms=rms(error.emkckff,2);
error.emkckfb_rms=rms(error.emkckfb,2);
error.emkckff_opt_rms=rms(error.emkckff_opt,2);
error.emkckfb_opt_rms=rms(error.emkckfb_opt,2);
% ME
error.ekff_max=max(abs(error.ekff),[],2);
error.ekfb_max=max(abs(error.ekfb),[],2);
error.eckff_max=max(abs(error.eckff),[],2);
error.eckfb_max=max(abs(error.eckfb),[],2);
error.eukff_max=max(abs(error.eukff),[],2);
error.eukfb_max=max(abs(error.eukfb),[],2);
error.emkckff_max=max(abs(error.emkckff),[],2);
error.emkckfb_max=max(abs(error.emkckfb),[],2);
error.emkckff_opt_max=max(abs(error.emkckff_opt),[],2);
error.emkckfb_opt_max=max(abs(error.emkckfb_opt),[],2);
%

% fprintf('EKF: %.3f \r',error.ekff_rms);
% fprintf('EKS: %.3f \r',error.ekfb_rms);
% fprintf('CEKF: %.3f \r',error.eckff_rms);
% fprintf('CEKS: %.3f \r',error.eckfb_rms);
% fprintf('UKF: %.3f \r',error.eukff_rms);
% fprintf('UKS: %.3f \r',error.eukfb_rms);
% fprintf('MKCKF: %.3f \r',error.emkckff_opt_rms);
% fprintf('MKCKS: %.3f \r',error.emkckfb_opt_rms);

%% smoother error plot
figure
x1=subplot(2,1,1);
box on
hold on
%plot(t,state(1,:)-kff.statef(1,:),'color','red','LineWidth',1.0,'LineStyle','--')
%plot(t,state(1,:)-ckff.statef(1,:),'color','blue','LineWidth',1.0,'LineStyle','--')
%plot(t,state(1,:)-MKCKFF(5).statef(1,:),'color','black','LineWidth',1.0,'LineStyle','--')
plot(t,state(1,:)-kfb.stateb(1,:),'color','red','LineWidth',1.0,'LineStyle','--')
plot(t,state(1,:)-ukfb.stateb(1,:),'color','m','LineWidth',1.0,'LineStyle','-')
plot(t,state(1,:)-ckfb.stateb(1,:),'color','blue','LineWidth',1.0,'LineStyle','--')
plot(t,state(1,:)-MKCKFB(5).stateb(1,:),'color','black','LineWidth',1.2,'LineStyle','-','Marker','diamond','MarkerIndices',1:20:length(t))
legend('KS','MCKS','UKS','MKCRTS','interpreter','latex','orientation','horizontal')
xticks([])
ylabel('$x_1-\bar{x}_1$','interpreter','latex')
set(gca,'fontsize',16)
x2=subplot(2,1,2);
hold on
box on
%plot(t,state(2,:)-kff.statef(2,:),'color','red','LineWidth',1.0,'LineStyle','--')
%plot(t,state(2,:)-ckff.statef(2,:),'color','blue','LineWidth',1.0,'LineStyle','--')
%plot(t,state(2,:)-MKCKFF(5).statef(2,:),'color','black','LineWidth',1.0,'LineStyle','--')
plot(t,state(2,:)-kfb.stateb(2,:),'color','red','LineWidth',1.0,'LineStyle','--')
plot(t,state(2,:)-ukfb.stateb(2,:),'color','m','LineWidth',1.0,'LineStyle','-')
plot(t,state(2,:)-ckfb.stateb(2,:),'color','blue','LineWidth',1.0,'LineStyle','--')
plot(t,state(2,:)-MKCKFB(5).stateb(2,:),'color','black','LineWidth',1.2,'LineStyle','-','Marker','diamond','MarkerIndices',1:20:length(t))
%legend('KS','CKFS','UKS','MKCFS','interpreter','latex','orientation','horizontal')
ylabel('$x_2-\bar{x}_2$','interpreter','latex')
set(gca,'fontsize',16)
linkaxes([x1,x2],'x')
xlim([100,200]);
linkaxes([x1,x2],'x')
set(gcf,'position',[100 100 750 600])



fprintf('KF& %.3f & %.3f & %.3f & %.3f\r',error.ekff_rms(1),error.ekff_rms(2),error.ekff_max(1),error.ekff_max(2));
fprintf('KS& %.3f & %.3f & %.3f & %.3f\r',error.ekfb_rms(1),error.ekfb_rms(2),error.ekfb_max(1),error.ekfb_max(2));
fprintf('CKF& %.3f & %.3f & %.3f & %.3f\r',error.eckff_rms(1),error.eckff_rms(2),error.eckff_max(1),error.eckff_max(2));
fprintf('CKS& %.3f & %.3f & %.3f & %.3f\r',error.eckfb_rms(1),error.eckfb_rms(2),error.eckfb_max(1),error.eckfb_max(2));
fprintf('UKF& %.3f & %.3f & %.3f & %.3f\r',error.eukff_rms(1),error.eukff_rms(2),error.eukff_max(1),error.eukff_max(2));
fprintf('UKS& %.3f & %.3f & %.3f & %.3f\r',error.eukfb_rms(1),error.eukfb_rms(2),error.eukfb_max(1),error.eukfb_max(2));
fprintf('MKCKF& %.3f & %.3f & %.3f & %.3f\r',error.emkckff_opt_rms(1),error.emkckff_opt_rms(2),error.emkckff_opt_max(1),error.emkckff_opt_max(2));
fprintf('MKCKS& %.3f & %.3f & %.3f & %.3f\r',error.emkckfb_opt_rms(1),error.emkckfb_opt_rms(2),error.emkckfb_opt_max(1),error.emkckfb_opt_max(2));



end