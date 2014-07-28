function xtraj = runPassive()

options.floating = true;
p = RigidBodyManipulator('pigeon_15.URDF', options);

tf = .5;

x0 = Point(p.getStateFrame());  % initial state
x0.base_z = 5;                  % initial height
x0.base_xdot = 10;              % initial forward velocity
x0.base_pitch = -.2;            % initial pitch
xtraj = p.simulate([0 tf], x0);

v = p.constructVisualizer();
v.playback(xtraj, struct('slider',true));

end
