% Made by Damien GUENERS
% Initialisation of the control model from simulink
% Sample time: 0.0001 seconds

clear all
close all

%% Body configuration function of the tall and the mass of the user (from Martijn Dekker)
lbody = 1.70;%meter
mbody = 60;%kilograms
mass=mbody;
f = 0;
kankle = mbody/55*50;
g = 9.81;
KFE = -5/180*pi;
[lThigh,lShank,hFoot,lFoot,lHeel,wPelvis,r1,r2,m1,m2,l,omega0_x,Xp_x,trunklbd,M_SM,mShank,mThigh]=segmentlengths_function(lbody,mbody,f,kankle,g,KFE,0);%???
[lThigh,lShank,hFoot,lFoot,lHeel,wPelvis,r1,r2,m1,m2,l,omega0_z,Xp_z,trunklbd,M_SM,mShank,mThigh]=segmentlengths_function(lbody,mbody,f,0,g,KFE,1);

%% Denavit Hartenberg matrix parameters (meter)
hfoot=hFoot;
lshank=lShank;
lthigh=lThigh;
lhip=0;%in function of the exoskeleton if hip abduction and flexion can be consider to be the same joint
lpelvis=wPelvis;

DH_Parameters=[hfoot 0;
    lshank 0;
    lthigh lhip;
    0 0;
    -lpelvis 0;
    0 0;
    -lthigh -lhip;
    -lshank 0
    -hfoot 0];

%% Position of the center of mass on the 4 joint repere (meter)

COM_Parameters=[-lpelvis/2;r2;0];



%% Initial swing leg (boolean)
initial_RorL=0;%Right swing = 1 / Left swing = 0

%% Number of step generate by the trajectory generator (3 steps min)
nb_step=4;
nb_step_pattern=20;

%% stride step lenght (meter)
stride_lenght=0.50;

%% stance width (meter)
stance_width=lpelvis;

%% Time configuration (second)

% time_stride=3.8;%best
time_stride=4;
time_stance=60/100*time_stride;
time_swing=40/100*time_stride;
time_DS=(time_stance-time_swing)/2;%2 DS by cycle
time_step=time_stride/2;

half_time_swing=time_swing/2;
end_time_swing=90/100*time_swing;
advanced_time=0.1;
delta_time=0.1;

%% State machine parameters
T_IT=time_DS;
T_DS=time_DS;
T_SS=time_swing;

%% Initial CoM velocity (m/s)
initial_CoM_velocity=[0;0;0];

%% Validity region of the joint angle (radian)

validity_angle=[-20*pi/180 20*pi/180;
    -120*pi/2 1.5*pi/180;
    -18*pi/180 110*pi/180;
    -19*pi/180 22*pi/180;
    -22*pi/180 19*pi/180;
    -18*pi/180 110*pi/180;
    -120*pi/180 1.5*pi/180;
    -20*pi/180 20*pi/180];%(min max)

validity_AEI=[-10*pi/180 10*pi/180;
    -10*pi/180 10*pi/180];


%% Martijn Dekker other parameter for simulink

dxFoot      = lFoot - lHeel;
dxTrunk     = 0.1;
HAA2HEE     = [0; 0; 0.001];
HEE2HFE     = 0.5*[0.2; 0; 0];

hTrunk      = trunklbd(1);
wTrunk      = trunklbd(2);
dTrunk      = trunklbd(3);
mTrunk      = M_SM;
Masses      = [mThigh; mShank; mTrunk];
lShankSM = lShank;
lThighSM = lThigh;
Lenght=[lShank;lThigh];
Pelvis_Parameters=[-lpelvis/2;hTrunk/2;0];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
LoadParametersSimMechanics;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

dxFootSM = dxFoot;
lFootSM = lFoot;

%% Position of the center of feet on Right feet repere (meter)
Foot_Parameters=[0;-(lFoot/2-lHeel);0];

%% Initial foot position
initial_left_foot=[0;0;-stance_width/2];
initial_right_foot=[0;0;stance_width/2];

%% Initial joint configurations (radian)
jointsinitial = [9;-15;6;0;0;6;-15;9]/180*pi;


theta0=0;
theta1=jointsinitial(1);
theta2=jointsinitial(2);
theta3=jointsinitial(3);
theta4=jointsinitial(4);
theta5=jointsinitial(5);
theta6=jointsinitial(6);
theta7=jointsinitial(7);
theta8=jointsinitial(8);
theta9=0;

a0=DH_Parameters(1,1);
d1=DH_Parameters(1,2);
a1=DH_Parameters(2,1);
d2=DH_Parameters(2,2);
a2=DH_Parameters(3,1);
d3=DH_Parameters(3,2);
a3=DH_Parameters(4,1);
d4=DH_Parameters(4,2);
a4=DH_Parameters(5,1);
d5=DH_Parameters(5,2);
a5=DH_Parameters(6,1);
d6=DH_Parameters(6,2);
a6=DH_Parameters(7,1);
d7=DH_Parameters(7,2);
a7=DH_Parameters(8,1);
d8=DH_Parameters(8,2);
a8=DH_Parameters(9,1);
d9=DH_Parameters(9,2);

xcom=COM_Parameters(1);
ycom=COM_Parameters(2);
zcom=COM_Parameters(3);
xfoot=Foot_Parameters(1);
yfoot=Foot_Parameters(2);
zfoot=Foot_Parameters(3);

xpelvis=Pelvis_Parameters(1);
ypelvis=Pelvis_Parameters(2);
zpelvis=Pelvis_Parameters(3);


XRshankR=[ a0 + xfoot + (lshank*cos(theta0)*cos(theta1))/2;
    yfoot + (lshank*sin(theta1))/2;
    zfoot - (lshank*cos(theta1)*sin(theta0))/2];

XRshankL=[ a0 + xfoot + d3*sin(theta0) + a2*cos(theta1 + theta2)*cos(theta0) + a1*cos(theta0)*cos(theta1) - a4*cos(theta4)*sin(theta0) - d7*cos(theta4)*cos(theta5)*sin(theta0) - d7*sin(theta0)*sin(theta4)*sin(theta5) + a4*cos(theta1 + theta2 + theta3)*cos(theta0)*sin(theta4) + a6*sin(theta1 + theta2 + theta3)*cos(theta0)*sin(theta6) - a6*cos(theta4)*cos(theta6)*sin(theta0)*sin(theta5) + a6*cos(theta5)*cos(theta6)*sin(theta0)*sin(theta4) - d7*cos(theta1 + theta2 + theta3)*cos(theta0)*cos(theta4)*sin(theta5) + d7*cos(theta1 + theta2 + theta3)*cos(theta0)*cos(theta5)*sin(theta4) - (lshank*sin(theta1 + theta2 + theta3)*cos(theta0)*cos(theta6)*sin(theta7))/2 - (lshank*sin(theta1 + theta2 + theta3)*cos(theta0)*cos(theta7)*sin(theta6))/2 + (lshank*cos(theta4)*cos(theta6)*cos(theta7)*sin(theta0)*sin(theta5))/2 - (lshank*cos(theta5)*cos(theta6)*cos(theta7)*sin(theta0)*sin(theta4))/2 + a6*cos(theta1 + theta2 + theta3)*cos(theta0)*cos(theta4)*cos(theta5)*cos(theta6) - (lshank*cos(theta4)*sin(theta0)*sin(theta5)*sin(theta6)*sin(theta7))/2 + (lshank*cos(theta5)*sin(theta0)*sin(theta4)*sin(theta6)*sin(theta7))/2 + a6*cos(theta1 + theta2 + theta3)*cos(theta0)*cos(theta6)*sin(theta4)*sin(theta5) - (lshank*cos(theta1 + theta2 + theta3)*cos(theta0)*cos(theta4)*cos(theta5)*cos(theta6)*cos(theta7))/2 + (lshank*cos(theta1 + theta2 + theta3)*cos(theta0)*cos(theta4)*cos(theta5)*sin(theta6)*sin(theta7))/2 - (lshank*cos(theta1 + theta2 + theta3)*cos(theta0)*cos(theta6)*cos(theta7)*sin(theta4)*sin(theta5))/2 + (lshank*cos(theta1 + theta2 + theta3)*cos(theta0)*sin(theta4)*sin(theta5)*sin(theta6)*sin(theta7))/2;
    yfoot + (a6*sin(theta1 + theta2 + theta3 + theta4 - theta5 - theta6))/4 + (a6*sin(theta1 + theta2 + theta3 - theta4 + theta5 - theta6))/4 - (d7*cos(theta1 + theta2 + theta3 + theta4 - theta5))/2 + (d7*cos(theta1 + theta2 + theta3 - theta4 + theta5))/2 - (lshank*sin(theta1 + theta2 + theta3 + theta4 - theta5 + theta6 + theta7))/8 - (lshank*sin(theta1 + theta2 + theta3 - theta4 + theta5 + theta6 + theta7))/8 - (a4*cos(theta1 + theta2 + theta3 + theta4))/2 - (a6*sin(theta1 + theta2 + theta3 + theta6))/2 + a2*sin(theta1 + theta2) - (lshank*sin(theta1 + theta2 + theta3 - theta6 - theta7))/4 + a1*sin(theta1) + (a6*sin(theta1 + theta2 + theta3 + theta4 - theta5 + theta6))/4 + (a6*sin(theta1 + theta2 + theta3 - theta4 + theta5 + theta6))/4 + (a4*cos(theta1 + theta2 + theta3 - theta4))/2 + (a6*sin(theta1 + theta2 + theta3 - theta6))/2 + (lshank*sin(theta1 + theta2 + theta3 + theta6 + theta7))/4 - (lshank*sin(theta1 + theta2 + theta3 + theta4 - theta5 - theta6 - theta7))/8 - (lshank*sin(theta1 + theta2 + theta3 - theta4 + theta5 - theta6 - theta7))/8;
    zfoot + d3*cos(theta0) - a2*cos(theta1 + theta2)*sin(theta0) - a4*cos(theta0)*cos(theta4) - a1*cos(theta1)*sin(theta0) - d7*cos(theta0)*cos(theta4)*cos(theta5) - d7*cos(theta0)*sin(theta4)*sin(theta5) - a4*cos(theta1 + theta2 + theta3)*sin(theta0)*sin(theta4) - a6*sin(theta1 + theta2 + theta3)*sin(theta0)*sin(theta6) - a6*cos(theta0)*cos(theta4)*cos(theta6)*sin(theta5) + a6*cos(theta0)*cos(theta5)*cos(theta6)*sin(theta4) + d7*cos(theta1 + theta2 + theta3)*cos(theta4)*sin(theta0)*sin(theta5) - d7*cos(theta1 + theta2 + theta3)*cos(theta5)*sin(theta0)*sin(theta4) + (lshank*sin(theta1 + theta2 + theta3)*cos(theta6)*sin(theta0)*sin(theta7))/2 + (lshank*sin(theta1 + theta2 + theta3)*cos(theta7)*sin(theta0)*sin(theta6))/2 + (lshank*cos(theta0)*cos(theta4)*cos(theta6)*cos(theta7)*sin(theta5))/2 - (lshank*cos(theta0)*cos(theta5)*cos(theta6)*cos(theta7)*sin(theta4))/2 - (lshank*cos(theta0)*cos(theta4)*sin(theta5)*sin(theta6)*sin(theta7))/2 + (lshank*cos(theta0)*cos(theta5)*sin(theta4)*sin(theta6)*sin(theta7))/2 - a6*cos(theta1 + theta2 + theta3)*cos(theta4)*cos(theta5)*cos(theta6)*sin(theta0) - a6*cos(theta1 + theta2 + theta3)*cos(theta6)*sin(theta0)*sin(theta4)*sin(theta5) + (lshank*cos(theta1 + theta2 + theta3)*cos(theta4)*cos(theta5)*cos(theta6)*cos(theta7)*sin(theta0))/2 - (lshank*cos(theta1 + theta2 + theta3)*cos(theta4)*cos(theta5)*sin(theta0)*sin(theta6)*sin(theta7))/2 + (lshank*cos(theta1 + theta2 + theta3)*cos(theta6)*cos(theta7)*sin(theta0)*sin(theta4)*sin(theta5))/2 - (lshank*cos(theta1 + theta2 + theta3)*sin(theta0)*sin(theta4)*sin(theta5)*sin(theta6)*sin(theta7))/2];
 
XRthighR=[a0 + xfoot - (lthigh*(cos(theta0)*sin(theta1)*sin(theta2) - cos(theta0)*cos(theta1)*cos(theta2)))/2 + a1*cos(theta0)*cos(theta1);
    yfoot + (lthigh*sin(theta1 + theta2))/2 + a1*sin(theta1);
    zfoot + (lthigh*(sin(theta0)*sin(theta1)*sin(theta2) - cos(theta1)*cos(theta2)*sin(theta0)))/2 - a1*cos(theta1)*sin(theta0)];

XRthighL=[a0 + xfoot + d3*sin(theta0) + a2*cos(theta1 + theta2)*cos(theta0) + a1*cos(theta0)*cos(theta1) - a4*cos(theta4)*sin(theta0) - d7*cos(theta4)*cos(theta5)*sin(theta0) - d7*sin(theta0)*sin(theta4)*sin(theta5) + a4*cos(theta1 + theta2 + theta3)*cos(theta0)*sin(theta4) - (lthigh*sin(theta1 + theta2 + theta3)*cos(theta0)*sin(theta6))/2 + (lthigh*cos(theta4)*cos(theta6)*sin(theta0)*sin(theta5))/2 - (lthigh*cos(theta5)*cos(theta6)*sin(theta0)*sin(theta4))/2 - d7*cos(theta1 + theta2 + theta3)*cos(theta0)*cos(theta4)*sin(theta5) + d7*cos(theta1 + theta2 + theta3)*cos(theta0)*cos(theta5)*sin(theta4) - (lthigh*cos(theta1 + theta2 + theta3)*cos(theta0)*cos(theta4)*cos(theta5)*cos(theta6))/2 - (lthigh*cos(theta1 + theta2 + theta3)*cos(theta0)*cos(theta6)*sin(theta4)*sin(theta5))/2;
    yfoot - (lthigh*sin(theta1 + theta2 + theta3 + theta4 - theta5 - theta6))/8 - (lthigh*sin(theta1 + theta2 + theta3 - theta4 + theta5 - theta6))/8 - (d7*cos(theta1 + theta2 + theta3 + theta4 - theta5))/2 + (d7*cos(theta1 + theta2 + theta3 - theta4 + theta5))/2 - (a4*cos(theta1 + theta2 + theta3 + theta4))/2 + (lthigh*sin(theta1 + theta2 + theta3 + theta6))/4 + a2*sin(theta1 + theta2) + a1*sin(theta1) + (a4*cos(theta1 + theta2 + theta3 - theta4))/2 - (lthigh*sin(theta1 + theta2 + theta3 + theta4 - theta5 + theta6))/8 - (lthigh*sin(theta1 + theta2 + theta3 - theta4 + theta5 + theta6))/8 - (lthigh*sin(theta1 + theta2 + theta3 - theta6))/4;
    zfoot + d3*cos(theta0) - a2*cos(theta1 + theta2)*sin(theta0) - a4*cos(theta0)*cos(theta4) - a1*cos(theta1)*sin(theta0) - d7*cos(theta0)*cos(theta4)*cos(theta5) - d7*cos(theta0)*sin(theta4)*sin(theta5) - a4*cos(theta1 + theta2 + theta3)*sin(theta0)*sin(theta4) + (lthigh*sin(theta1 + theta2 + theta3)*sin(theta0)*sin(theta6))/2 + (lthigh*cos(theta0)*cos(theta4)*cos(theta6)*sin(theta5))/2 - (lthigh*cos(theta0)*cos(theta5)*cos(theta6)*sin(theta4))/2 + d7*cos(theta1 + theta2 + theta3)*cos(theta4)*sin(theta0)*sin(theta5) - d7*cos(theta1 + theta2 + theta3)*cos(theta5)*sin(theta0)*sin(theta4) + (lthigh*cos(theta1 + theta2 + theta3)*cos(theta4)*cos(theta5)*cos(theta6)*sin(theta0))/2 + (lthigh*cos(theta1 + theta2 + theta3)*cos(theta6)*sin(theta0)*sin(theta4)*sin(theta5))/2];
 
XRtrunk=[a0 + xfoot + d3*sin(theta0) + a2*cos(theta1 + theta2)*cos(theta0) + a1*cos(theta0)*cos(theta1) - xpelvis*cos(theta4)*sin(theta0) + ypelvis*sin(theta0)*sin(theta4) + zpelvis*sin(theta1 + theta2 + theta3)*cos(theta0) + ypelvis*cos(theta1 + theta2 + theta3)*cos(theta0)*cos(theta4) + xpelvis*cos(theta1 + theta2 + theta3)*cos(theta0)*sin(theta4);
    yfoot - (xpelvis*cos(theta1 + theta2 + theta3 + theta4))/2 + a2*sin(theta1 + theta2) + (ypelvis*sin(theta1 + theta2 + theta3 + theta4))/2 + a1*sin(theta1) + (xpelvis*cos(theta1 + theta2 + theta3 - theta4))/2 + (ypelvis*sin(theta1 + theta2 + theta3 - theta4))/2 - zpelvis*cos(theta1 + theta2 + theta3);
    zfoot - xpelvis*(cos(theta0)*cos(theta4) + cos(theta1 + theta2 + theta3)*sin(theta0)*sin(theta4)) + ypelvis*(cos(theta0)*sin(theta4) - cos(theta1 + theta2 + theta3)*cos(theta4)*sin(theta0)) + d3*cos(theta0) - a2*cos(theta1 + theta2)*sin(theta0) - a1*cos(theta1)*sin(theta0) - zpelvis*sin(theta1 + theta2 + theta3)*sin(theta0)];

CoM_position=(mShank*XRshankR+mShank*XRshankL+mThigh*XRthighR+mThigh*XRthighL+mTrunk*XRtrunk)/(2*mShank+2*mThigh+mTrunk);
CoM_position(3)=0;
R=[0 1 0;
    1 0 0;
    0 0 -1];

initial_CoM_position=R*CoM_position;

XR4=[a0 + xfoot + d3*sin(theta0) + a1*cos(theta0)*cos(theta1) + a2*cos(theta0)*cos(theta1)*cos(theta2) - a2*cos(theta0)*sin(theta1)*sin(theta2);
    yfoot + a2*sin(theta1 + theta2) + a1*sin(theta1);
    zfoot + a2*(sin(theta0)*sin(theta1)*sin(theta2) - cos(theta1)*cos(theta2)*sin(theta0)) + d3*cos(theta0) - a1*cos(theta1)*sin(theta0)];

x4CoM=CoM_position(1)-XR4(1);

%% height vrp initial (meter)
%Corresponds to the height of the center of mass stand phase
height_vrp=CoM_position(1);


%% PID term for the swing foot control

%right

kP_right=mbody*[5.6 0 0 0;
    0 8.5 0 0;
    0 0 19 0;
    0 0 0 75];

kI_right=mbody*[38.2 0 0 0;
    0 34.5 0 0;
    0 0 35.5 0;
    0 0 0 35];

kD_right=mbody*[0.12 0 0 0;
    0 0.64 0 0;
    0 0 0.84 0;
    0 0 0 0.95];

%left
kP_left=mbody*[75 0 0 0;
    0 19 0 0;
    0 0 8.5 0;
    0 0 0 5.6];


kI_left=mbody*[35 0 0 0;
    0 35.5 0 0;
    0 0 34.5 0;
    0 0 0 38.2];

kD_left=mbody*[0.95 0 0 0;%
    0 0.84 0 0;
    0 0 0.64 0;
    0 0 0 0.12];

%% Proportional gain for the DCM control

K_DCM=4.3*[1 0 0;
    0 1 0;
    0 0 1];
