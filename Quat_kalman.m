clc
clear all
close  all

m1=466.5;
m2=49.8;
k1=5700;
k2=135000;
b1=290;
b2=1400;
k1=5.51;
k2=10;

%define the matrices for MIMO system
A=[-b1/m1 b1/m1 -k1/m1 k1/m1; 
    b1/m2 -(b1+b2)/m2 k1/m2 -(k1+k2)/m2; 
    1 0 0 0; 
    0 1 0 0];
B=[0 0;
    b2/m2 k2/m2;
    0 0;
    0 0];
C=[1 0 0 0];
D=zeros(1,2);

%State space model
sys=ss(A,B,C,D)
 
%Discrete state space model
sysd=c2d(sys,0.002,'zoh');
[F,G,H,D]=ssdata(sysd)
 
%Initialisation
Q=10;
R=10;
u=[10;10];
P=eye(4);
w=10*randn(4,1);
v=6*randn(1);
X=[100;1000;100;10000];
 
%KALMAN ALGORITHM
 
for i=1:10000
 
%PREDICTION
 
%Predicted state
     X=F*X+G*u;  
 
%Predicted covariance
     P=F*P*F'+Q;
 
%Noisy measurement
     x=X+w;
     Z=H*x+v; %measurement equation
 
%Predicted Values
% Xpred1(i,1)=x(1); 1st state
% Xpred2(i,1)=x(2); 2nd state
% Xpred3(i,1)=x(3); 3rd state
% Xpred4(i,1)=x(4); 4th state
 
%ESTIMATION
 
%Innovation covariance matrix
     S=(H*P*H'+R);
 
%Kalman gain
     W=P*H'*inv(S);
 
%Update State
%       T=(eye(4)-W*H)*X+W*Z;
       T=X+W*(Z-H*X);
%Update Covariance
     P=(eye(4)-W*H)*P;
     X1=X';% estimation
     x1=x' %true value
     E=x1-X1; %residue
     t(i)=i*0.01; %???
     true(i,:)=x;
     esti(i,:)=X1;
     error(i,:)=E;
%Estimated Values
% Xest1(i,1)=x(1);
% Xest2(i,1)=x(2);
% Xest3(i,1)=x(3);
% Xest4(i,1)=x(4);
end
% figure(1);
% plot(t,u);
 
%Comparision between predicted values and true values
 
%State X1
figure(2);
% plot(t,Xpred1,'-k.',t,Xest1,'-r'),legend('Xpred1','Xest1'),title('Comparision between predicted & estimated values of state X1'),xlabel('time'),ylabel('State X1');
plot(t,true(:,1),'-k.',t,esti(:,1),'-r'),legend('pred','est'),title('Comparision between predicted & estimated values of state X1'),xlabel('time'),ylabel('State X1');
 
 
%Error between predicted value and estimated value
% error1=Xpred1-Xest1;
figure(3)
% plot(t,error1),,title('Error between predicted & estimated values of state X1'),xlabel('time'),ylabel('Error');
plot(t,error(:,1),'k'),title('Error between predicted & estimated values of state X1'),xlabel('time'),ylabel('Error');
 
%State X2
figure(4)
% plot(t,Xpred2,'-k.',t,Xest2,'-r'),legend('Xpred2','Xest2'),title('Comparision between predicted & estimated values of state X2'),xlabel('time'),ylabel('State X2');
plot(t,true(:,2),'-k.',t,esti(:,2),'-r'),legend('pred','est'),title('Comparision between predicted & estimated values of state X2'),xlabel('time'),ylabel('State X2');
 
 
%Error between predicted value and estimated value
% error2=Xpred2-Xest2;
figure(5)
% plot(t,error2),title('Error between predicted & estimated values of state X2'),xlabel('time'),ylabel('Error');
plot(t,error(:,2),'k'),title('Error between predicted & estimated values of state X2'),xlabel('time'),ylabel('Error');
 
%State X3
figure(6)
% plot(t,Xpred3,'-k.',t,Xest3,'-r'),legend('Xpred3','Xest3'),title('Comparision between predicted & estimated values of state X3'),xlabel('time'),ylabel('State X3');
plot(t,true(:,3),'-k.',t,esti(:,3),'-r'),legend('pred','est'),title('Comparision between predicted & estimated values of state X3'),xlabel('time'),ylabel('State X3');
 
 
%Error between predicted value and estimated value
% error3=Xpred3-Xest3;
figure(7)
% plot(t,error3),title('Error between predicted & estimated values of state X3'),xlabel('time'),ylabel('Error');
plot(t,error(:,3),'k'),title('Error between predicted & estimated values of state X3'),xlabel('time'),ylabel('Error');
 
%State X4
figure(8)
% plot(t,Xpred4,'-k.',t,Xest4,'-r'),legend('Xpred4','Xest4'),title('Comparision between predicted & estimated values of state X4'),xlabel('time'),ylabel('State X4');
plot(t,true(:,4),'-k.',t,esti(:,4),'-r'),legend('pred','est'),title('Comparision between predicted & estimated values of state X4'),xlabel('time'),ylabel('State X4');
 
%Error between predicted value and estimated value
% error4=Xpred4-Xest4;
figure(9)
%plot(t,error4),title('Error between predicted & estimated values of state X4'),xlabel('time'),ylabel('Error');
plot(t,error(:,4),'k'),title('Error between predicted & estimated values of state X4'),xlabel('time'),ylabel('Error');