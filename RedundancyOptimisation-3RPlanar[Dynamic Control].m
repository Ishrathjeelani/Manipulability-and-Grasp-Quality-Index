clc; clear all; close all;

global l1 l2 l3 m1 m2 m3 g ts qdi dqdi ddqdi taus i
l1=1; l2=1; l3=1;
m1=0.5; m2=0.5; m3=0.5; g=-9.81; taus=[];
% 
% %Assuming masses at the end of each joint
% syms l1 l2 l3 m1 m2 m3 g q1(t) q2(t) q3(t) q1_dot(t) q2_dot(t) q3_dot(t) q1_ddot(t) q2_ddot(t) q3_ddot(t) t t1 t2 t3
% x1=l1*cos(q1); x2=l1*cos(q1)+l2*cos(q1+q2); x3=l1*cos(q1)+l2*cos(q1+q2)+l3*cos(q1+q2+q3);
% y1=l1*sin(q1); y2=l1*sin(q1)+l2*sin(q1+q2); y3=l1*sin(q1)+l2*sin(q1+q2)+l3*sin(q1+q2+q3);
% 
% x1_dot=diff(x1,t); y1_dot=diff(y1,t); v1_sq=x1_dot^2+y1_dot^2;
% x2_dot=diff(x2,t); y2_dot=diff(y2,t); v2_sq=x2_dot^2+y2_dot^2;
% x3_dot=diff(x3,t); y3_dot=diff(y3,t); v3_sq=x3_dot^2+y3_dot^2;
% 
% KE1=0.5*m1*v1_sq;
% KE2=0.5*m2*v2_sq;
% KE3=0.5*m3*v3_sq;
% KE=KE1+KE2+KE3;
% 
% PE1=m1*g*y1;
% PE2=m2*g*y2;
% PE3=m3*g*y3;
% PE=PE1+PE2+PE3;
% 
% L=KE-PE;
% L=subs(L,[diff(q1,t),diff(q2,t),diff(q3,t)],[q1_dot,q2_dot,q3_dot]);
% tau1=diff(diff(L,q1_dot),t)-diff(L,q1);
% tau1=subs(tau1,[diff(q1_dot(t), t),diff(q2_dot(t), t),diff(q3_dot(t), t),diff(q1,t),diff(q2,t),diff(q3,t)],[q1_ddot,q2_ddot,q3_ddot,q1_dot,q2_dot,q3_dot]);
% tau1=simplify(tau1);
% tau2=diff(diff(L,q2_dot),t)-diff(L,q2);
% tau2=subs(tau2,[diff(q1_dot(t), t),diff(q2_dot(t), t),diff(q3_dot(t), t),diff(q1,t),diff(q2,t),diff(q3,t)],[q1_ddot,q2_ddot,q3_ddot,q1_dot,q2_dot,q3_dot]);
% tau2=simplify(tau2);
% tau3=diff(diff(L,q3_dot),t)-diff(L,q3);
% tau3=subs(tau3,[diff(q1_dot(t), t),diff(q2_dot(t), t),diff(q3_dot(t), t),diff(q1,t),diff(q2,t),diff(q3,t)],[q1_ddot,q2_ddot,q3_ddot,q1_dot,q2_dot,q3_dot]);
% tau3=simplify(tau3);
% collect(tau2,[q1_ddot,q2_ddot,q3_ddot])
% vars = [q1_ddot(t) q2_ddot(t) q3_ddot(t)];
% eqns = [tau1==t1, tau2==t2, tau3==t3];
% [A,b] = equationsToMatrix(eqns,vars);
% 
% ddots = inv(A)*b;
% %collect(ddots(1),[q1_ddot,q2_ddot,q3_ddot])
% %simplify(ddots(1))

%Trajectory Planning
syms t
t0=0; tf=2; n=30;
ts=tf/n;
tspan = linspace(t0,tf,n);
c=[0,2]; r=0.6;
%line
[xd1t,dxd1t,ddxd1t]= trajectory_planner(t0,tf,c(1)-r,0,c(1)+r,0);
yd1=2;

xd1=[]; dxd1=[]; yd2=[]; dyd2=[]; const=pi/2-pi/10; 
dq10=0; dq20=0; dq30=0;  yd0=c(2); q10=3.0886; q20=-1.6075; q30=-0.2245;
q1ds=[]; q2ds=[]; q3ds=[]; dq1ds=[]; dq2ds=[]; dq3ds=[]; ddq1ds=[]; ddq2ds=[]; ddq3ds=[];
q1as=[]; q2as=[]; q3as=[]; dq1as=[]; dq2as=[]; dq3as=[];
fig=figure();

for i=1:n
    i
    xd1(end+1)=subs(xd1t,t,tspan(i));
    dxd1(end+1)=subs(dxd1t,t,tspan(i));
    xd2=xd1(i)-c(1); phi=acos(xd2/r); yd2(end+1)=c(2)+r*sin(phi); dyd2(end+1)=(yd2(i)-yd0);
    %Trajectory1
    %[q1,q2,q3,dq1,dq2,dq3]=invKin_constQ(xd1(i),yd1,dxd1(i),0,const);
    %[q1,q2,q3,dq1,dq2,dq3]=invKin_norm(q10,q20,q30,dxd1(i),0);
    %[q1,q2,q3,dq1,dq2,dq3]=invKin_velNorm(xd1(i),yd1,dxd1(i),0);
    %[q1,q2,q3,dq1,dq2,dq3]=invKin_velMan(xd1(i),yd1,dxd1(i),0);

    %Trajectory2
    %[q1,q2,q3,dq1,dq2,dq3]=invKin_constQ(xd1(i),yd2(i),dxd1(i),dyd2(i),const);
    %[q1,q2,q3,dq1,dq2,dq3]=invKin_norm(q10,q20,q30,dxd1(i),dyd2(i));
    %[q1,q2,q3,dq1,dq2,dq3]=invKin_velNorm(xd1(i),yd2(i),dxd1(i),dyd2(i));
    [q1,q2,q3,dq1,dq2,dq3]=invKin_velMan(xd1(i),yd2(i),dxd1(i),dyd2(i));
    

    q1=wrapToPi(q1); q2=wrapToPi(q2); q3=wrapToPi(q3);
    q1ds(end+1)=wrapToPi(q1); q2ds(end+1)=wrapToPi(q2); q3ds(end+1)=wrapToPi(q3);
    dq1ds(end+1)=dq1; dq2ds(end+1)=dq2; dq3ds(end+1)=dq3;
    ddq1ds(end+1)=(dq1-dq10)/ts; ddq2ds(end+1)=(dq2-dq20)/ts; ddq3ds(end+1)=(dq3-dq30)/ts;
    dy0=yd2(i);

    qdi=[q1 q2 q3]'; dqdi=[dq1 dq2 dq3]'; ddqdi=[ddq1ds(i) ddq2ds(i) ddq3ds(i)]';
    x0 =[q1;q2;q3;dq1;dq2;dq3];%[q10;q20;q30;dq10;dq20;dq30];%
    yi = ode4(@(t,x)odefun_tau(t,x),tspan(i),ts,x0);
    q1=yi(1); q2=yi(2); q3=yi(3); dq1=yi(4); dq2=yi(5); dq3=yi(6);
    q1as(end+1)=yi(1); q2as(end+1)=yi(2); q3as(end+1)=yi(3);
    dq1as(end+1)=yi(4); dq2as(end+1)=yi(5); dq3as(end+1)=yi(6);

    dq10=dq1; dq20=dq2; dq30=dq3; q10=q1; q20=q2; q30=q3;
    
    plot([0 cos(q1)],[0 sin(q1)],'k');
    title("Tracking Semi-circle with Max.Vel. Ellipse");
    axis([-2 2 -1 3]);
    hold on;
    plot([cos(q1) cos(q1)+cos(q1+q2)],[sin(q1) sin(q1)+sin(q1+q2)],'k');
    plot([cos(q1)+cos(q1+q2) cos(q1)+cos(q1+q2)+cos(q1+q2+q3)],[sin(q1)+sin(q1+q2) sin(q1)+sin(q1+q2)+sin(q1+q2+q3)],'b');
    plot(xd1(i),yd2(i),'r.');
    drawnow
end
q1ds(1)
q2ds(1)
q3ds(1)
hold off;
fig1=figure();
plot(q1ds); hold on; plot(q2ds); plot(q3ds);
title("Desired Joint Positions"); 
xlabel("Time(s)"); ylabel("Angle(rad)");
legend("Joint 1","Joint 2","Joint 3");
hold off;

fig2=figure();
plot(dq1ds); hold on; plot(dq2ds); plot(dq3ds); 
title("Desired Joint Velocities");
xlabel("Time(s)"); ylabel("Angular Velocity(rad/s)");
legend("Joint 1","Joint 2","Joint 3");
hold off;

fig3=figure();
plot(ddq1ds); hold on; plot(ddq2ds); plot(ddq3ds); 
title("Desired Joint Accelerations");
xlabel("Time(s)"); ylabel("Angular Acceleration(rad/s2)");
legend("Joint 1","Joint 2","Joint 3");
hold off;

fig4=figure();
plot(taus(1,:)); hold on; plot(taus(2,:)); plot(taus(3,:)); 
title("Control Input - Torques at Joints");
xlabel("Time(s)"); ylabel("Torque(Nm)");
legend("Joint 1","Joint 2","Joint 3");
hold off;


% 
% [t,y] = ode45(fun, tspan, x0);
% fig1=figure();
% plot(t,wrapToPi(y(:,1)))
% hold on
% plot(t,wrapToPi(y(:,2)))
% plot(t,wrapToPi(y(:,3)))
% hold off;




function dxdt= odefun(t,x)
l1=1; l2=1; l3=1;
m1=1; m2=1; m3=1; g=-9.81;
m11=(m1+m2+m3)*l1^2+(m2+m3)*l2^2+m3*l3^2+2*l1*l3*m3*cos(x(2)+x(3))+2*l1*l2*cos(x(2))*(m2+m3)+2*l2*l3*m3*cos(x(3));
m12=(m2+m3)*l2^2+m3*l3^2+l1*l3*m3*cos(x(2)+x(3))+l1*l2*cos(x(2))*(m2+m3)+2*l1*l3*m3*cos(x(3));
m13=m3*l3^2+l1*l3*m3*cos(x(2)+x(3))+l2*l3*m3*cos(x(3));
m21=(m2+m3)*l2^2+m3*l3^2+l1*l3*m3*cos(x(2)+x(3))+l1*l2*cos(x(2))*(m2+m3)+2*l2*l3*m3*cos(x(3));
m22=(m2+m3)*l2^2+m3*l3^2+2*l2*l3*m3*cos(x(3));
m23=m3*l3^2+l3*l2*m3*cos(x(3));
m31=m13; m32=m23; m33=l3^2*m3;
M=[m11 m12 m13;
    m21 m22 m23;
    m31 m32 m33];
c11=(-l1*l3*m3*sin(x(2)+x(3))-l1*l2*sin(x(2))*(m2+m3))*x(5)^2 +(-l1*l3*m3*sin(x(2)+x(3))-l2*l3*m3*sin(x(3)))*x(6)^2;
c12=2*(-l1*l3*m3*sin(x(2)+x(3))-l1*l2*(m1+m3)*sin(x(2)))*x(4)*x(5)+2*(-l1*l3*m3*sin(x(2)+x(3))-l2*l3*m3*sin(x(3)))*x(4)*x(6)+2*(-l1*l3*m3*sin(x(2)+x(3))-l2*l3*m3*sin(x(3)))*x(5)*x(6);
c21=(l1*l3*m3*sin(x(2)+x(3))+l1*l2*sin(x(2))*(m2+m3))*x(4)^2+(-l3*l2*m3*sin(x(3)))*x(6)^2;
c22=2*(-l1*l3*m3*sin(x(2)+x(3))-l2*l3*m3*sin(x(3)))*x(4)*x(6)+2*(-l2*l3*m3*sin(x(3)))*x(5)*x(6);
c31=(l1*l3*m3*sin(x(2)+x(3))+l2*l3*m3*sin(x(3)))*x(4)^2+(l2*l3*m3*sin(x(3)))*x(5)^2;
c32=2*(l2*l3*m3*sin(x(3)))*x(4)*x(5);
C=[c11+c12;
    c21+c22;
    c31+c32];
g11=m1*g*l1*cos(x(1))+m2*g*(l1*cos(x(1))+l2*cos(x(1)+x(2)))+m3*g*(l1*cos(x(1))+l2*cos(x(1)+x(2))+l3*cos(x(1)+x(2)+x(3)));
g22=m2*g*(l2*cos(x(1)+x(2)))+m3*g*(l2*cos(x(1)+x(2))+l3*cos(x(1)+x(2)+x(3)));
g33=m3*g*(l3*cos(x(1)+x(2)+x(3)));
G=[g11; g22; g33];
eqns=inv(M)*(-C-G);

dxdt = zeros(6,1);
dxdt(1) = x(4);
dxdt(2) = x(5);
dxdt(3) = x(6);
dxdt(4) = eqns(1,1);
dxdt(5) = eqns(2,1);
dxdt(6) = eqns(3,1);
end

function dxdt= odefun_tau(t,x)
global l1 l2 l3 m1 m2 m3 g ts qdi dqdi ddqdi taus i
l1=1; l2=1; l3=1;
m1=1; m2=1; m3=1; g=-9.81;
%Control Params
kp=10; kd=10;

m11=(m1+m2+m3)*l1^2+(m2+m3)*l2^2+m3*l3^2+2*l1*l3*m3*cos(x(2)+x(3))+2*l1*l2*cos(x(2))*(m2+m3)+2*l2*l3*m3*cos(x(3));
m12=(m2+m3)*l2^2+m3*l3^2+l1*l3*m3*cos(x(2)+x(3))+l1*l2*cos(x(2))*(m2+m3)+2*l1*l3*m3*cos(x(3));
m13=m3*l3^2+l1*l3*m3*cos(x(2)+x(3))+l2*l3*m3*cos(x(3));
m21=(m2+m3)*l2^2+m3*l3^2+l1*l3*m3*cos(x(2)+x(3))+l1*l2*cos(x(2))*(m2+m3)+2*l2*l3*m3*cos(x(3));
m22=(m2+m3)*l2^2+m3*l3^2+2*l2*l3*m3*cos(x(3));
m23=m3*l3^2+l3*l2*m3*cos(x(3));
m31=m13; m32=m23; m33=l3^2*m3;
M=[m11 m12 m13;
    m21 m22 m23;
    m31 m32 m33];
c11=(-l1*l3*m3*sin(x(2)+x(3))-l1*l2*sin(x(2))*(m2+m3))*x(5)^2 +(-l1*l3*m3*sin(x(2)+x(3))-l2*l3*m3*sin(x(3)))*x(6)^2;
c12=2*(-l1*l3*m3*sin(x(2)+x(3))-l1*l2*(m1+m3)*sin(x(2)))*x(4)*x(5)+2*(-l1*l3*m3*sin(x(2)+x(3))-l2*l3*m3*sin(x(3)))*x(4)*x(6)+2*(-l1*l3*m3*sin(x(2)+x(3))-l2*l3*m3*sin(x(3)))*x(5)*x(6);
c21=(l1*l3*m3*sin(x(2)+x(3))+l1*l2*sin(x(2))*(m2+m3))*x(4)^2+(-l3*l2*m3*sin(x(3)))*x(6)^2;
c22=2*(-l1*l3*m3*sin(x(2)+x(3))-l2*l3*m3*sin(x(3)))*x(4)*x(6)+2*(-l2*l3*m3*sin(x(3)))*x(5)*x(6);
c31=(l1*l3*m3*sin(x(2)+x(3))+l2*l3*m3*sin(x(3)))*x(4)^2+(l2*l3*m3*sin(x(3)))*x(5)^2;
c32=2*(l2*l3*m3*sin(x(3)))*x(4)*x(5);
C=[c11+c12;
    c21+c22;
    c31+c32];
g11=m1*g*l1*cos(x(1))+m2*g*(l1*cos(x(1))+l2*cos(x(1)+x(2)))+m3*g*(l1*cos(x(1))+l2*cos(x(1)+x(2))+l3*cos(x(1)+x(2)+x(3)));
g22=m2*g*(l2*cos(x(1)+x(2)))+m3*g*(l2*cos(x(1)+x(2))+l3*cos(x(1)+x(2)+x(3)));
g33=m3*g*(l3*cos(x(1)+x(2)+x(3)));
G=[g11; g22; g33];

x1=qdi(1); x2=qdi(2); x3=qdi(3); x4=dqdi(1); x5=dqdi(2); x6=dqdi(3);
c11d=(-l1*l3*m3*sin(x2+x3)-l1*l2*sin(x2)*(m2+m3))*x5^2 +(-l1*l3*m3*sin(x2+x3)-l2*l3*m3*sin(x3))*x6^2;
c12d=2*(-l1*l3*m3*sin(x2+x3)-l1*l2*(m1+m3)*sin(x2))*x4*x5+2*(-l1*l3*m3*sin(x2+x3)-l2*l3*m3*sin(x3))*x4*x6+2*(-l1*l3*m3*sin(x2+x3)-l2*l3*m3*sin(x3))*x5*x6;
c21d=(l1*l3*m3*sin(x2+x3)+l1*l2*sin(x2)*(m2+m3))*x4^2+(-l3*l2*m3*sin(x3))*x6^2;
c22d=2*(-l1*l3*m3*sin(x2+x3)-l2*l3*m3*sin(x3))*x4*x6+2*(-l2*l3*m3*sin(x3))*x5*x6;
c31d=(l1*l3*m3*sin(x2+x3)+l2*l3*m3*sin(x3))*x4^2+(l2*l3*m3*sin(x3))*x5^2;
c32d=2*(l2*l3*m3*sin(x3))*x4*x5;
Cd=[c11d+c12d;
    c21d+c22d;
    c31d+c32d];
g11d=m1*g*l1*cos(x1)+m2*g*(l1*cos(x1)+l2*cos(x1+x2))+m3*g*(l1*cos(x1)+l2*cos(x1+x2)+l3*cos(x1+x2+x3));
g22d=m2*g*(l2*cos(x1+x2))+m3*g*(l2*cos(x1+x2)+l3*cos(x1+x2+x3));
g33d=m3*g*(l3*cos(x1+x2+x3));
Gd=[g11d; g22d; g33d];
T=Cd+Gd+M*(ddqdi+kp*(qdi-[x(1); x(2); x(3)])+kd*(dqdi-[x(4); x(5); x(6)]));
taus(:,end+1)=T;
eqns=inv(M)*(-C-G+T);

dxdt = zeros(6,1);
dxdt(1) = x(4);
dxdt(2) = x(5);
dxdt(3) = x(6);
dxdt(4) = eqns(1,1);
dxdt(5) = eqns(2,1);
dxdt(6) = eqns(3,1);
end

function [theta_d,thetadot_d,thetaddot_d]= trajectory_planner(to,tf,thetai,thetadi,thetaf,thetadf)
syms t
Q = [thetai;thetadi;thetaf;thetadf];
t0=to;
B = [1 t0 t0*t0 t0*t0*t0;
    0 1 2*t0 3*t0*t0;
    1 tf tf*tf tf*tf*tf;
    0 1 2*tf 3*tf*tf];
Binv1= inv(B);
A1= Binv1*Q;
a0=A1(1,1);
a1=A1(2,1);
a2=A1(3,1);
a3=A1(4,1);
theta_d=a0+(a1*t)+(a2*t*t)+(a3*t*t*t); % Thita(t)
thetadot_d=(a1)+(2*a2*t)+(3*a3*t*t);
thetaddot_d=2*a2+ (6*a3*t);
end

function [q1_val,q2_val,q3_val,dq1,dq2,dq3]=invKin_constQ(xd,yd,dxd,dyd,const)
global l1 l2 l3 m1 m2 m3 g
syms q1 q2 q3
q=[0; 0; 0]; dq=[0; 0; 0];
J=[-l1*sin(q1)-l2*sin(q1+q2)-l3*sin(q1+q2+q3) -l2*sin(q1+q2)-l3*sin(q1+q2+q3) -l3*sin(q1+q2+q3);
    l1*cos(q1)+l2*cos(q1+q2)+l3*cos(q1+q2+q3) l2*cos(q1+q2)+l3*cos(q1+q2+q3) l3*cos(q1+q2+q3)];

eq1=l1*cos(q1)+l2*cos(q1+q2)+l3*cos(q1+q2+q3)==xd;
eq2=l1*sin(q1)+l2*sin(q1+q2)+l3*sin(q1+q2+q3)==yd;
eq3=q1+q2+q3==const;
%soln=solve([eq1,eq2,eq3],[q1, q2, q3]);
soln=vpasolve([eq1,eq2,eq3],[q1, q2, q3],[-pi pi; -pi pi; -pi pi]);
q1_val=soln.q1; q2_val=soln.q2; q3_val=soln.q3; 
J_val=subs(J,[q1,q2,q3],[q1_val,q2_val,q3_val]);
dq=pinv(J_val)*[dxd; dyd];
dq1=dq(1); dq2=dq(2); dq3=dq(3);
end

function [q1_val,q2_val,q3_val,dq1,dq2,dq3]=invKin_norm(q1,q2,q3,dxd,dyd)
global l1 l2 l3 m1 m2 m3 g ts

%syms q1 q2 q3
J=[-l1*sin(q1)-l2*sin(q1+q2)-l3*sin(q1+q2+q3) -l2*sin(q1+q2)-l3*sin(q1+q2+q3) -l3*sin(q1+q2+q3);
    l1*cos(q1)+l2*cos(q1+q2)+l3*cos(q1+q2+q3) l2*cos(q1+q2)+l3*cos(q1+q2+q3) l3*cos(q1+q2+q3)];
dq=pinv(J)*[dxd; dyd]; dq1=dq(1); dq2=dq(2); dq3=dq(3);
q1_val=q1+dq(1)*ts; q2_val=q2+dq(2)*ts; q3_val=q3+dq(3)*ts;

% eqn=pinv(J)*[dxd;dyd];
% eq1=l1*cos(q1)+l2*cos(q1+q2)+l3*cos(q1+q2+q3)==xd;
% eq2=l1*sin(q1)+l2*sin(q1+q2)+l3*sin(q1+q2+q3)==yd;
% eq3=eqn(1)+eqn(2)+eqn(3)==0;
% soln=solve([eq1 eq2 eq3],[q1 q2 q3],"Real",true)
% q1_val=soln.q1(1); q2_val=soln.q2(1); q3_val=soln.q3(1);
% J_val=subs(J,[q1 q2 q3],[q1_val,q2_val,q3_val])
% dq=pinv(J_val)*[dxd;dyd];
% dq1=dq(1); dq2=dq(2); dq3=dq(3);
end

function [q1,q2,q3,dq1,dq2,dq3]=invKin_velNorm(xd,yd,dxd,dyd)
global l1 l2 l3 m1 m2 m3 g ts
q1 = optimvar('q1');
q2 = optimvar('q2');
q3 = optimvar('q3');
dq1 = optimvar('dq1');
dq2 = optimvar('dq2');
dq3 = optimvar('dq3');
J=[-l1*sin(q1)-l2*sin(q1+q2)-l3*sin(q1+q2+q3) -l2*sin(q1+q2)-l3*sin(q1+q2+q3) -l3*sin(q1+q2+q3);
    l1*cos(q1)+l2*cos(q1+q2)+l3*cos(q1+q2+q3) l2*cos(q1+q2)+l3*cos(q1+q2+q3) l3*cos(q1+q2+q3)];
dx=J*[dq1; dq2; dq3];
prob = optimproblem('ObjectiveSense','minimize');
prob.Constraints.cons1=l1*cos(q1)+l2*cos(q1+q2)+l3*cos(q1+q2+q3)==xd;
prob.Constraints.cons2=l1*sin(q1)+l2*sin(q1+q2)+l3*sin(q1+q2+q3)==yd;
prob.Constraints.cons3=q1>=-2*pi;
prob.Constraints.cons4=q2>=-2*pi;
prob.Constraints.cons5=q3>=-2*pi;
prob.Constraints.cons6=q1<=2*pi;
prob.Constraints.cons7=q2<=2*pi;
prob.Constraints.cons8=q3<=2*pi;
prob.Constraints.cons9=dx(1)==dxd;
prob.Constraints.cons10=dx(2)==dyd;
%prob.Constraints.cons10=cos(q1 + q2 + q3)*sin((q1))*cos((q1) + (q2))*sin(q1) - cos(q1 + q2 + q3)*cos((q1))*sin((q1) + (q2))*sin(q1) + sin(q1 + q2 + q3)*cos((q1))*sin((q1) + (q2))*cos(q1) - sin(q1 + q2 + q3)*sin((q1))*cos((q1) + (q2))*cos(q1) + cos(q1 + q2 + q3)*cos((q1) + (q2) + (q3))*sin((q1) + (q2))*sin(q1) - cos(q1 + q2 + q3)*sin((q1) + (q2) + (q3))*cos((q1) + (q2))*sin(q1) - sin(q1 + q2 + q3)*cos((q1) + (q2) + (q3))*sin((q1) + (q2))*cos(q1) + sin(q1 + q2 + q3)*sin((q1) + (q2) + (q3))*cos((q1) + (q2))*cos(q1) - cos((q1))*sin((q1) + (q2) + (q3))*cos(q1 + q2)*sin(q1) + cos((q1))*sin((q1) + (q2) + (q3))*sin(q1 + q2)*cos(q1) + cos((q1) + (q2) + (q3))*sin((q1))*cos(q1 + q2)*sin(q1) - cos((q1) + (q2) + (q3))*sin((q1))*sin(q1 + q2)*cos(q1) - cos((q1))*sin((q1) + (q2))*cos(q1 + q2)*sin(q1) + cos((q1))*sin((q1) + (q2))*sin(q1 + q2)*cos(q1) + sin((q1))*cos((q1) + (q2))*cos(q1 + q2)*sin(q1) - sin((q1))*cos((q1) + (q2))*sin(q1 + q2)*cos(q1) - cos(q1 + q2 + q3)*cos((q1))*sin((q1) + (q2) + (q3))*sin(q1 + q2) + cos(q1 + q2 + q3)*cos((q1) + (q2) + (q3))*sin((q1))*sin(q1 + q2) + sin(q1 + q2 + q3)*cos((q1))*sin((q1) + (q2) + (q3))*cos(q1 + q2) - sin(q1 + q2 + q3)*cos((q1) + (q2) + (q3))*sin((q1))*cos(q1 + q2) - 2*cos(q1 + q2 + q3)*cos((q1))*sin((q1) + (q2) + (q3))*sin(q1) + 2*cos(q1 + q2 + q3)*cos((q1) + (q2) + (q3))*sin((q1))*sin(q1) + 2*sin(q1 + q2 + q3)*cos((q1))*sin((q1) + (q2) + (q3))*cos(q1) - 2*sin(q1 + q2 + q3)*cos((q1) + (q2) + (q3))*sin((q1))*cos(q1) + 2*cos(q1 + q2 + q3)*cos((q1) + (q2) + (q3))*sin((q1) + (q2))*sin(q1 + q2) - 2*cos(q1 + q2 + q3)*sin((q1) + (q2) + (q3))*cos((q1) + (q2))*sin(q1 + q2) - 2*sin(q1 + q2 + q3)*cos((q1) + (q2) + (q3))*sin((q1) + (q2))*cos(q1 + q2) + 2*sin(q1 + q2 + q3)*sin((q1) + (q2) + (q3))*cos((q1) + (q2))*cos(q1 + q2)-0.002>=0;
%prob.Constraints.cons11=q3^2-0.002>=0;
prob.Objective = dq1^2+dq2^2+dq3^2;
x0.q1=0; x0.q2=0; x0.q3=0; x0.dq1=0; x0.dq2=0; x0.dq3=0;
sol = solve(prob,x0);
q1=sol.q1; q2=sol.q2; q3=sol.q3; dq1=sol.dq1; dq2=sol.dq2; dq3=sol.dq3;
end

function [q1,q2,q3,dq1,dq2,dq3]=invKin_velMan(xd,yd,dxd,dyd)
global l1 l2 l3 m1 m2 m3 g ts
q1 = optimvar('q1');
q2 = optimvar('q2');
q3 = optimvar('q3');
dq1 = optimvar('dq1');
dq2 = optimvar('dq2');
dq3 = optimvar('dq3');
J=[-l1*sin(q1)-l2*sin(q1+q2)-l3*sin(q1+q2+q3) -l2*sin(q1+q2)-l3*sin(q1+q2+q3) -l3*sin(q1+q2+q3);
    l1*cos(q1)+l2*cos(q1+q2)+l3*cos(q1+q2+q3) l2*cos(q1+q2)+l3*cos(q1+q2+q3) l3*cos(q1+q2+q3)];
dx=J*[dq1; dq2; dq3];
JJT=J*J';
prob = optimproblem('ObjectiveSense','minimize');
prob.Constraints.cons1=l1*cos(q1)+l2*cos(q1+q2)+l3*cos(q1+q2+q3)==xd;
prob.Constraints.cons2=l1*sin(q1)+l2*sin(q1+q2)+l3*sin(q1+q2+q3)==yd;
prob.Constraints.cons3=q1>=-2*pi;
prob.Constraints.cons4=q2>=-2*pi;
prob.Constraints.cons5=q3>=-2*pi;
prob.Constraints.cons6=q1<=2*pi;
prob.Constraints.cons7=q2<=2*pi;
prob.Constraints.cons8=q3<=2*pi;
prob.Constraints.cons9=dx(1)==dxd;
prob.Constraints.cons10=dx(2)==dyd;
prob.Constraints.cons11=dq1^2+dq2^2+dq3^2==1;
%prob.Constraints.cons9=dx(2)==dyd;
%prob.Constraints.cons10=cos(q1 + q2 + q3)*sin((q1))*cos((q1) + (q2))*sin(q1) - cos(q1 + q2 + q3)*cos((q1))*sin((q1) + (q2))*sin(q1) + sin(q1 + q2 + q3)*cos((q1))*sin((q1) + (q2))*cos(q1) - sin(q1 + q2 + q3)*sin((q1))*cos((q1) + (q2))*cos(q1) + cos(q1 + q2 + q3)*cos((q1) + (q2) + (q3))*sin((q1) + (q2))*sin(q1) - cos(q1 + q2 + q3)*sin((q1) + (q2) + (q3))*cos((q1) + (q2))*sin(q1) - sin(q1 + q2 + q3)*cos((q1) + (q2) + (q3))*sin((q1) + (q2))*cos(q1) + sin(q1 + q2 + q3)*sin((q1) + (q2) + (q3))*cos((q1) + (q2))*cos(q1) - cos((q1))*sin((q1) + (q2) + (q3))*cos(q1 + q2)*sin(q1) + cos((q1))*sin((q1) + (q2) + (q3))*sin(q1 + q2)*cos(q1) + cos((q1) + (q2) + (q3))*sin((q1))*cos(q1 + q2)*sin(q1) - cos((q1) + (q2) + (q3))*sin((q1))*sin(q1 + q2)*cos(q1) - cos((q1))*sin((q1) + (q2))*cos(q1 + q2)*sin(q1) + cos((q1))*sin((q1) + (q2))*sin(q1 + q2)*cos(q1) + sin((q1))*cos((q1) + (q2))*cos(q1 + q2)*sin(q1) - sin((q1))*cos((q1) + (q2))*sin(q1 + q2)*cos(q1) - cos(q1 + q2 + q3)*cos((q1))*sin((q1) + (q2) + (q3))*sin(q1 + q2) + cos(q1 + q2 + q3)*cos((q1) + (q2) + (q3))*sin((q1))*sin(q1 + q2) + sin(q1 + q2 + q3)*cos((q1))*sin((q1) + (q2) + (q3))*cos(q1 + q2) - sin(q1 + q2 + q3)*cos((q1) + (q2) + (q3))*sin((q1))*cos(q1 + q2) - 2*cos(q1 + q2 + q3)*cos((q1))*sin((q1) + (q2) + (q3))*sin(q1) + 2*cos(q1 + q2 + q3)*cos((q1) + (q2) + (q3))*sin((q1))*sin(q1) + 2*sin(q1 + q2 + q3)*cos((q1))*sin((q1) + (q2) + (q3))*cos(q1) - 2*sin(q1 + q2 + q3)*cos((q1) + (q2) + (q3))*sin((q1))*cos(q1) + 2*cos(q1 + q2 + q3)*cos((q1) + (q2) + (q3))*sin((q1) + (q2))*sin(q1 + q2) - 2*cos(q1 + q2 + q3)*sin((q1) + (q2) + (q3))*cos((q1) + (q2))*sin(q1 + q2) - 2*sin(q1 + q2 + q3)*cos((q1) + (q2) + (q3))*sin((q1) + (q2))*cos(q1 + q2) + 2*sin(q1 + q2 + q3)*sin((q1) + (q2) + (q3))*cos((q1) + (q2))*cos(q1 + q2)-0.002>=0;
%prob.Constraints.cons11=q3^2-0.002>=0;
prob.Objective = -(JJT(1,1)*JJT(2,2)+JJT(1,2)*JJT(2,1));
x0.q1=0; x0.q2=0; x0.q3=0; x0.dq1=0; x0.dq2=0; x0.dq3=0;
sol = solve(prob,x0);
q1=sol.q1; q2=sol.q2; q3=sol.q3; dq1=sol.dq1; dq2=sol.dq2; dq3=sol.dq3;
end

function yout = ode4(F,t,h,y0)
% ODE4  Classical Runge-Kutta ODE solver.
%   yout = ODE4(F,t0,h,tfinal,y0) uses the classical
%   Runge-Kutta method with fixed step size h on the interval
%      t0 <= t <= tfinal
%   to solve
%      dy/dt = F(t,y)
%   with y(t0) = y0.
%   Copyright 2014 - 2015 The MathWorks, Inc.
y = y0;
yout = y;
s1 = F(t,y);
s2 = F(t+h/2, y+h*s1/2);
s3 = F(t+h/2, y+h*s2/2);
s4 = F(t+h, y+h*s3);
y = y + h*(s1 + 2*s2 + 2*s3 + s4)/6;
yout = y; %#ok<AGROW>
end
