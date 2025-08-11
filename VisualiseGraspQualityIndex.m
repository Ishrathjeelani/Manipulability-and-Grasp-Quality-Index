clc; clear all; close all;

%% Define circle
r=20; th=[0:0.1:2*pi];
px=r*cos(th); py=r*sin(th);

%% Let a point of contact be at (0,20,0) (for finger 1)
phi=pi/25; minSV0=0; vol0=0; iso0=0; th_max=pi; gamma=pi/3; poly0=1;polyCG0=1;
angle_arr=[0:phi:2*pi]; configs_SV=[]; configs_Vol=[]; configs_Iso=[]; configs_Poly=[]; configs_PolyCG=[];
optSV_Config=[]; optVol_Config=[]; optIso_Config=[]; optPoly_Config=[]; optPolyCG_Config=[];
p0=[0 20 0];
for i=2:length(angle_arr)-1
    i
    p1=[r*cos(angle_arr(i)) r*sin(angle_arr(i)) 0];
    for j=i+1:length(angle_arr)-1
    p2=[r*cos(angle_arr(j)) r*sin(angle_arr(j)) 0];
    G=getGraspMatrix(p0,p1,p2,[0 angle_arr(i) angle_arr(j)]);
    [U S V]=svd(G);
    minSV = min( S(S>0));
    if minSV>minSV0
        minSV0=minSV;
       optSV_Config=[angle_arr(i) angle_arr(j) minSV];
    end
    configs_SV(i-1,j-2)=minSV;
    configs_SV(j-2,i-1)=minSV;
    vol=sqrt(det(S*S'));
    if vol>vol0
        vol0=vol;
        optVol_Config=[angle_arr(i) angle_arr(j) vol];
    end
    configs_Vol(i-1,j-2)=vol;
    configs_Vol(j-2,i-1)=vol;
    iso=min( S(S>0))/max( S(S>0));
    if (1-iso)<(1-iso0)
        iso0=iso;
        optIso_Config=[angle_arr(i) angle_arr(j) iso];
    end
    configs_Iso(i-1,j-2)=iso;
    configs_Iso(j-2,i-1)=iso;
    th1=abs(acos((p1-p0)*(p2-p0)'/(norm(p1-p0)*norm(p2-p0))));
    th2=abs(acos((p2-p1)*(p0-p1)'/(norm(p2-p1)*norm(p0-p1))));
    th3=abs(acos((p1-p2)*(p0-p2)'/(norm(p1-p2)*norm(p0-p2))));
    poly=(abs(th1-gamma)+abs(th2-gamma)+abs(th3-gamma))/th_max;
    if poly<poly0
        poly0=poly;
        optPoly_Config=[angle_arr(i) angle_arr(j) poly];
    end
    configs_Poly(i-1,j-2)=poly;
    configs_Poly(j-2,i-1)=poly;
    vx=[p2(1) p1(1) p0(1)];
    vy=[p2(2) p1(2) p0(2)];
    polyin = polyshape(vx,vy);
    pc = centroid(polyin);
    polyCG=norm(pc);
    if polyCG<=polyCG0
    polyCG0=polyCG;
    optPolyCG_Config=[angle_arr(i) angle_arr(j) polyCG];
    end
    configs_PolyCG(i-1,j-2)=polyCG;
    configs_PolyCG(j-2,i-1)=polyCG;
    end
end
fig1=figure();
surf(angle_arr(2:end-2),angle_arr(2:end-2),configs_SV);
title("Minimum Singular Value Index");
xlabel("Theta 2");
ylabel("Theta 3");
fig2=figure();
surf(angle_arr(2:end-2),angle_arr(2:end-2),configs_Vol);
title("Wrench Ellipsoid Volume Index");
xlabel("Theta 2");
ylabel("Theta 3");
fig3=figure();
surf(angle_arr(2:end-2),angle_arr(2:end-2),configs_Iso);
title("Grasp Isotropy Index");
xlabel("Theta 2");
ylabel("Theta 3");
fig4=figure();
surf(angle_arr(2:end-2),angle_arr(2:end-2),configs_Poly);
title("Grasp Polygon Regularity Index");
xlabel("Theta 2");
ylabel("Theta 3");
fig5=figure();
surf(angle_arr(2:end-2),angle_arr(2:end-2),configs_PolyCG);
title("Grasp Centroid displacement Index");
xlabel("Theta 2");
ylabel("Theta 3");
Optimal_MSV=optSV_Config
Optimal_Volume=optVol_Config
Optimal_Isotropy=optIso_Config
Optimal_PolygonRegularity=optPoly_Config
Optima_PolygonCGDistance=optPolyCG_Config



%Function to compute Grasp Matrix (Assuming Soft Contact)
function G=getGraspMatrix(p1,p2,p3,phi)
syms q3
trans = [cos(q3) -sin(q3) 0;
    sin(q3) cos(q3) 0;
    0 0 1];
p=[p1 0;p2 0;p3 0];
r=[p(:,1) p(:,2) p(:,3)];
n_cap=[]; t_cap=[]; s_cap=[];
G_Fric=[]; tau=[]; contact_frames=[];
for i=1:3
  frame=subs(trans,q3,phi(i));
  contact_frames(:,3*(i-1)+1:3*i)=frame;
  s(i,:)=frame(:,1)';
  t(i,:)=frame(:,2)';
  n(i,:)=frame(:,3)';
  %Cross-Product to compute moment
  skew_r = [0 -r(i,3) r(i,2);
      r(i,3) 0 -r(i,1);
      -r(i,2) r(i,1) 0];
  M = double(skew_r*frame);
  s_cap(end+1,:)= [s(i,:) M(:,3)'];
  t_cap(end+1,:)= [t(i,:) M(:,2)'];
  n_cap(end+1,:)= [n(i,:) M(:,1)'];
  tau(end+1,:)=[0 0 0 n(i,:)];
  G_Fric(:,end+1:end+4)= [s_cap(i,:)' t_cap(i,:)' n_cap(i,:)' tau(i,:)'];
end
G=G_Fric;
end
