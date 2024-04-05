%% Tippy top graphs
% python3  mjtippytop.py -l  -r 10
tic
if (exist('ts','var')) angvel1=angvel;qpos1=qpos;qvel1=qvel; stem_height1=stem_height;ts1=ts;end

ts=csvread('Data/ts.csv');
angvel=csvread('Data/angvel.csv');
qvel=csvread('Data/qvel.csv');
qpos=csvread('Data/qpos.csv');
stem_height=csvread('Data/stem_height.csv');
toc

figure(31);plot(ts,angvel); title('Angular velocity')
grid on;xlabel('time (s)');ylabel('ang vel (rad/s)')
figure(32);plot(ts,stem_height);title('Stem height')
figure(33);plot(ts,qvel);title('CoM velocity')
figure(34);plot(ts,qpos(:,1:3));title('CoM position')