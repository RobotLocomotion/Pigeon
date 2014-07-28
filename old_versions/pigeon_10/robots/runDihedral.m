function xtraj = runDihedral( )

tf = 7;
dt = .007;
pts = tf/dt;

options.floating = true;
p = TimeSteppingRigidBodyManipulator('pigeon_10.URDF',dt,options);
p = enableIdealizedPositionControl(p,true);
p = compile(p);
x0 = Point(p.getStateFrame());  % initial state
x0.base_z = 5;          % initial height
x0.base_xdot = 20;      % initial forward velocity
x0.base_pitch = -.2;    % initial pitch

x0.left_elbow_roll = -pi/6;
x0.right_elbow_roll = pi/6;



u = zeros(15,pts);
u(1,:) = .1*sin(linspace(0,5*tf,pts))+.05;
%u(5,:) = -sin(linspace(0,10*tf,pts));
u(5,:) = -pi/6;
%u(8,:) = -.7*sin(linspace(0,10*tf,pts));
%u(12,:) = sin(linspace(0,10*tf,pts));
u(12,:) = pi/6;
%u(15,:) = .7*sin(linspace(0,10*tf,pts));
utraj = PPTrajectory(foh(linspace(0,tf,pts),u));
utraj = setOutputFrame(utraj, p.getInputFrame);

sys = cascade(utraj,p);

xtraj = sys.simulate([0 tf],x0);

v = p.constructVisualizer();
v.playback(xtraj, struct('slider',true));

end
