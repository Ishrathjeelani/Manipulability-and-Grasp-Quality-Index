clc; clear; close all;
syms a1 a2 a3;
l=1;
xj1 = l*cos(a1); xj2 = l*(cos(a1)+cos(a1+a2)); xj3 = l*(cos(a1)+cos(a1+a2)+cos(a1+a2+a3));
yj1 = l*sin(a1); yj2 = l*(sin(a1)+sin(a1+a2)); yj3 = l*(sin(a1)+sin(a1+a2)+sin(a1+a2+a3));
pj=[xj1,yj1,xj2,yj2,xj3,yj3];
J = [-sin(a1)-sin(a1+a2)-sin(a1+a2+a3) -sin(a1+a2)-sin(a1+a2+a3) -sin(a1+a2+a3);
    cos(a1)+cos(a1+a2)+cos(a1+a2+a3) cos(a1+a2)+cos(a1+a2+a3) cos(a1+a2+a3)];

%% Substituting values
J1_val = subs(J,[a1 a2 a3],[pi/4 -pi/4 -pi/2]);
p1_val = subs(pj,[a1 a2 a3],[pi/4 -pi/4 -pi/2]);
J2_val = subs(J,[a1 a2 a3],[pi/4 -pi/4 pi/2]);
p2_val = subs(pj,[a1 a2 a3],[pi/4 -pi/4 pi/2]);
J3_val = subs(J,[a1 a2 a3],[pi/4 0 -pi+0.1]);
p3_val = subs(pj,[a1 a2 a3],[pi/4 0 -pi+0.1]);
J4_val = subs(J,[a1 a2 a3],[pi/4 0 0.1]);
p4_val = subs(pj,[a1 a2 a3],[pi/4 0 0.1]);
 %% Null Space Visualisation
[u1,s1,v1]=svd(J1_val);
v1(:,3) =double(v1(:,3));
th1 = v1(1,3)*linspace(0,2*pi,50);
th2 = v1(2,3)*linspace(0,2*pi,50);
th3 = v1(3,3)*linspace(0,2*pi,50);
fig= figure();
plot3(th1,th2,th3);
title("Null Space of Planar 3R");
grid on
view([60 30]);
xlabel('x')
ylabel('y')
zlabel('z')

%% Manipulability and Force Ellipsoids
fig1=figure();
plot_ellipse(J1_val*J1_val',p1_val(5),p1_val(6));
hold on;
plot_ellipse(inv(J1_val*J1_val'),p1_val(5),p1_val(6));
plot([0,p1_val(1)],[0,p1_val(2)],'k',LineWidth=3);
plot([p1_val(1),p1_val(3)],[p1_val(2),p1_val(4)],'y',LineWidth=3);
plot([p1_val(3),p1_val(5)],[p1_val(4),p1_val(6)],'g',LineWidth=3);
title("TCase 1");
xlabel("Vx,Fx");
ylabel("Vy,Fy");
legend('Force Ellipsoid','Velocity Ellipsoid');
hold off;
fig2=figure();
plot_ellipse(J2_val*J2_val',p2_val(5),p2_val(6));
hold on;
plot_ellipse(inv(J2_val*J2_val'),p2_val(5),p2_val(6));
plot([0,p2_val(1)],[0,p2_val(2)],'k',LineWidth=3);
plot([p2_val(1),p2_val(3)],[p2_val(2),p2_val(4)],'y',LineWidth=3);
plot([p2_val(3),p2_val(5)],[p2_val(4),p2_val(6)],'g',LineWidth=3);
title("TCase 2");
xlabel("Vx,Fx");
ylabel("Vy,Fy");
legend('Force Ellipsoid','Velocity Ellipsoid');
hold off;
fig3=figure();
plot_ellipse(J3_val*J3_val',p3_val(5),p3_val(6));
hold on;
plot_ellipse(inv(J3_val*J3_val'),p3_val(5),p3_val(6));
plot([0,p3_val(1)],[0,p3_val(2)],'k',LineWidth=3);
plot([p3_val(1),p3_val(3)],[p3_val(2),p3_val(4)],'y',LineWidth=3);
plot([p3_val(3),p3_val(5)],[p3_val(4),p3_val(6)],'g',LineWidth=3);
title("TCase 3");
xlabel("Vx,Fx");
ylabel("Vy,Fy");
legend('Force Ellipsoid','Velocity Ellipsoid');
hold off;
fig4=figure();
plot_ellipse(J4_val*J4_val',p4_val(5),p4_val(6));
hold on;
plot_ellipse(inv(J4_val*J4_val'),p4_val(5),p4_val(6));
plot([0,p4_val(1)],[0,p4_val(2)],'k',LineWidth=3);
plot([p4_val(1),p4_val(3)],[p4_val(2),p4_val(4)],'y',LineWidth=3);
plot([p4_val(3),p4_val(5)],[p4_val(4),p4_val(6)],'g',LineWidth=3);
title("TCase 4");
xlabel("Vx,Fx");
ylabel("Vy,Fy");
legend('Force Ellipsoid','Velocity Ellipsoid');
hold off;

function plot_ellipse(E,px,py)
 % plots an ellipse of the form xEx = 1
 R = chol(E);
 t = linspace(0, 2*pi+0.05, 100); % or any high number to make curve smooth
 z = [cos(t); sin(t)];
 ellipse = inv(R) * z;
 plot(ellipse(1,:)+px, ellipse(2,:)+py)
end

