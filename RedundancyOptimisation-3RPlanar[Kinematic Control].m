clc; clear; close all;

global l1 l2 l3 b1 b2 b3 a1 a2 a3
l1=20; l2=30; l3=30; b1=[1/sqrt(3)*cos(pi/2), 1/sqrt(3)*sin(pi/2)]; b2=[1/sqrt(3)*cos(pi+pi/6), 1/sqrt(3)*sin(pi+pi/6)]; b3=[1/sqrt(3)*cos(-pi/6), 1/sqrt(3)*sin(-pi/6)];
a1=[sqrt(3)*cos(pi/2), sqrt(3)*sin(pi/2)]; a2=[sqrt(3)*cos(pi+pi/6), sqrt(3)*sin(pi+pi/6)]; a3=[sqrt(3)*cos(-pi/6), sqrt(3)*sin(-pi/6)];

syms q1 q2 q3

th=[0:0.05:2*pi];
th_s=[0:0.1:10*pi];
r1=2; c=2/(10*pi); r2=c*th_s;
t1=[1:1:length(th)];
t2=[1:1:length(th_s)];
pz=l1;
cx=0; cy=0;
traj1_x=cx+r1*cos(th);
traj1_y=cy+r1*sin(th);
traj2_x=cx+r2.*cos(th_s);
traj2_y=cy+r2.*sin(th_s);



J = [-l3*sin(q1)*cos(q2+q3)-l2*sin(q1)*cos(q2), -l3*cos(q1)*sin(q2+q3)-l2*cos(q1)*sin(q2), -l3*cos(q1)*sin(q2+q3);
 l3*cos(q1)*cos(q2+q3)+l2*cos(q1)*cos(q2), l3*sin(q1)*sin(q2+q3)-l2*sin(q1)*sin(q2), l3*sin(q1)*sin(q2+q3)];

[sol11,sol12,sol13]= inv_kin_parallel(traj1_x,traj1_y,0);
[sol21,sol22,sol23]= inv_kin(traj2_x,traj2_y,pz);
M1s =[];
for i=1:length(traj1_x)
  J_val = subs(J,[q1 q2 q3],[sol11(i) sol12(i) sol13(i)]);
  M1s(end+1)=sqrt(det(J_val*J_val'));
end
M2s =[];

for i=1:length(traj2_x)
 J_val = subs(J,[q1 q2 q3],[sol21(i) sol22(i) sol23(i)]);
 M2s(end+1)=sqrt(det(J_val*J_val'));
end

fig1 = figure();
plot(t1,sol11);
hold on;
plot(t1,sol12);
plot(t1,sol13);
title("Joint Angles vs Time [Trajectory 1]");
legend('th1','th2','th3');
xlabel("time(s)");
ylabel("theta (rad)");
hold off;

fig2 = figure();
plot(t2,sol21);
hold on;
plot(t2,sol22);
plot(t2,sol23);
title("Joint Angles vs Time [Trajectory 2]");
legend('th1','th2','th3');
xlabel("time(s)");
ylabel("theta (rad)");
hold off;

fig3 = figure();
plot(t1,M1s);
title("Manipulability Index vs Time [Trajectory 1]");
xlabel("time(s)");
ylabel("w (Manupilability Index)");

fig4 = figure();
plot(t2,M2s);
title("Manipulability Index vs Time [Trajectory 2]");
xlabel("time(s)");
ylabel("w (Manupilability Index)");

fig5 = figure();
plot(traj1_x,traj1_y);
title("Trajectory 1 - Circle R=20mm");
xlabel("x");
ylabel("y");

fig6 = figure();
plot(traj2_x,traj2_y);
title("Trajectory 2 - Spiral R=20mm");
xlabel("x");
ylabel("y");


function [s1s,s2s,s3s]=inv_kin_parallel(px,py,phi)
    syms s1 s2 s3 
    global b1 b2 b3 a1 a2 a3
    s1s=[]; s2s=[]; s3s=[];
    for i=1:length(px)
        i
        eq1=(px(i)+b1(1)*cos(phi)-b1(2)*sin(phi)-a1(1))^2+(py(i)+b1(1)*sin(phi)+b1(2)*cos(phi)-a1(2))^2==s1^2;
        eq2=(px(i)+b2(1)*cos(phi)-b2(2)*sin(phi)-a2(1))^2+(py(i)+b2(1)*sin(phi)+b2(2)*cos(phi)-a2(2))^2==s2^2;
        eq3=(px(i)+b3(1)*cos(phi)-b3(2)*sin(phi)-a3(1))^2+(py(i)+b3(1)*sin(phi)+b3(2)*cos(phi)-a3(2))^2==s3^2;
        soln = solve([eq1,eq2,eq3],[s1 s2 s3]);
        for j=1:length(soln.s1)
            if soln.s1(j)>0 && soln.s2(j)>0 && soln.s3(j)>0
            s1s(i)=soln.s1(j); s2s(i)=soln.s2(j); s3s(i)=soln.s3(j);
            end
        end    
    end
end

function [th1s,th2s,th3s]=inv_kin(px,py,pz)
    global l1 l2 l3
    syms s1 s2 s23 c1 c2 c23
    th1s=[]; th2s=[]; th3s=[];
    x0=0; y0=0; th10=0; th20=0; th30=0;
    for i=1:length(px)
        i
        eq1 = l3*s23+l2*s2+l1==pz;
        eq2 = l3*s1*c23+l2*s1*c2==py(i);
        eq3 = l3*c1*c23+l2*c1*c2==px(i);
        eq4 = s1^2+c1^2==1;
        eq5 = s2^2+c2^2==1;
        eq6 = s23^2+c23^2==1;
        soln = solve([eq1,eq2,eq3,eq4,eq5,eq6],[s1 s2 s23 c1 c2 c23]);
        nrm =100;
        idx = 0;
        for j=1:4
             aa1 = double(atan(soln.s1(j)/soln.c1(j)));%acos(soln.c1(1));%
             aa2 = double(atan(soln.s2(j)/soln.c2(j)));
             aa3 = double(atan(soln.s23(j)/soln.c23(j)))-aa2;
               JJ = [-l3*sin(aa1)*cos(aa2+aa3)-l2*sin(aa1)*cos(aa2), -l3*cos(aa1)*sin(aa2+aa3)-l2*cos(aa1)*sin(aa2), -l3*cos(aa1)*sin(aa2+aa3);
            l3*cos(aa1)*cos(aa2+aa3)+l2*cos(aa1)*cos(aa2), l3*sin(aa1)*sin(aa2+aa3)-l2*sin(aa1)*sin(aa2), l3*sin(aa1)*sin(aa2+aa3)];
            if norm(JJ*[aa1-th10; aa2-th20; aa3-th30]-[px(i)-x0;py(i)-y0],2)<=nrm
                nrm=norm(JJ*[aa1-th10; aa2-th20; aa3-th30]-[px(i)-x0;py(i)-y0])
                idx=j;
                th1=aa1; th2=aa2; th3=aa3;
            end
        end
        %th1 = atan(soln.s1(idx)/soln.c1(idx));%acos(soln.c1(1));%
        %th2 = atan(soln.s2(idx)/soln.c2(idx));
        %th3 = atan(soln.s23(idx)/soln.c23(idx))-th2;
        th1s(end+1)=th1;
        th2s(end+1)=th2;
        th3s(end+1)=th3;
        x0=px(i); y0=py(i); th10=th1; th20=th2; th30=th3;
    end
end