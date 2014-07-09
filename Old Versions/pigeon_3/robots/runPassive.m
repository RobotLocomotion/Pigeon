function xtraj = runPassive()

options.floating = true;
p = RigidBodyManipulator('pigeon_3.URDF', options);

tf = .5;
x0 = zeros(42,1); % initial state
x0(3) = 3; % initial height
x0(23) = 10; % initial forward velocity
x0(4) = .75; % initial pitch
xtraj = p.simulate([0 tf], x0);

% I think the model in solid works
% isn't in the right axis. Usually the x(22)
% should be the forward velocity and
% x(5) should be the pitch. Basically the pigeon
% is facing in the wrong direction in solidworks
% and this make for counterintuitive states in Drake.
% Could we fix that?

v = p.constructVisualizer();
v.playback(xtraj, struct('slider',true));

end

