function xtraj = runFlapping( )

tf = 1;
dt = .007;
pts = tf/dt;

options.floating = true;
p = TimeSteppingRigidBodyManipulator('pigeon_12.URDF',dt,options);
p = enableIdealizedPositionControl(p,true);
p = compile(p);
x0 = Point(p.getStateFrame());  % initial state
x0.base_z = 5;          % initial height
x0.base_xdot = 20;      % initial forward velocity
x0.base_pitch = -.2;    % initial pitch



u = zeros(16,pts);
u(1,:) = .3*sin(linspace(0,10*tf,pts))+.15;
u(3,:) = -sin(linspace(0,10*tf,pts));
u(7,:) = -.7*sin(linspace(0,10*tf,pts));
u(10,:) = sin(linspace(0,10*tf,pts));
u(11,:) = .7*sin(linspace(0,10*tf,pts));
utraj = PPTrajectory(foh(linspace(0,tf,pts),u));
utraj = setOutputFrame(utraj, p.getInputFrame);

sys = cascade(utraj,p);

xtraj = sys.simulate([0 tf],x0);

v = p.constructVisualizer();
v.playback(xtraj, struct('slider',true));

end
