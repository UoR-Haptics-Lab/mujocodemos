%% Tippy top graphs
% python3  mjtippytop.py -m  -r 10
tic
if (exist('ts','var')) angvel1=angvel;qpos1=qpos;qvel1=qvel; stem_height1=stem_height;ts1=ts;end

fname='ttdata.mat';
f=dir(fname);
freshness=datetime('now')-datetime(f.date);
freshness.Format='s';
if seconds(freshness) > 300 % time in seconds > val then getting stale
  sprintf('File last updated %s ago',string(freshness))
end

load(fname)
% toc
figure(21);plot(ts,angvel); title('Angular velocity')
grid on;xlabel('time (s)');ylabel('ang vel (rad/s)')
figure(22);plot(ts,stem_height);title('Stem height')
figure(23);plot(ts,qvel);title('CoM velocity')
figure(24);plot(ts,qpos(:,1:3));title('CoM position')
%figure(25);plot(ts,qpos(:,1:3));title('CoM velocity')

