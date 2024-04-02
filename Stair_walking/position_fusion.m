function [pos,K,P]=position_fusion(posf,posb,Pposf,Pposb)

Pposf=Pposf+10^(-15);
Pposb=Pposb+10^(-15);

len=length(Pposf);
pos=zeros(len,3);
K=zeros(len,1);
P=zeros(len,1);

for i=1:len
    % 
    K(i)=Pposb(i)/(Pposf(i)+Pposb(i));
    pos(i,:)=K(i).*posf(i,:)+(1-K(i)).*posb(i,:);
    P(i)=1/(1/(Pposf(i))+1/(Pposb(i)));
end


end