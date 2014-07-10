function xtraj = runPassive()

options.floating = true;
p = RigidBodyManipulator('pigeon_8.URDF', options);

tf = .5;
x0 = zeros(42,1); % initial state
x0(3) = 3; % initial height
x0(22) = 10; % initial forward velocity
x0(5) = -.75; % initial pitch
xtraj = p.simulate([0 tf], x0);

v = p.constructVisualizer();
v.playback(xtraj, struct('slider',true));

end
