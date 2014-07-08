function xtraj = runFlapping( )

tf = 1.5;
dt = .001;
pts = tf/dt;

options.floating = true;
p = TimeSteppingRigidBodyManipulator('pigeon_8.URDF',dt,options);
p = enableIdealizedPositionControl(p,true);
p = compile(p);
x0 = zeros(42,1); % initial state
x0(3) = 5; % initial height
x0(22) = 20; % initial forward velocity
x0(5) = .2; % initial pitch

u = zeros(15,pts);
u(1,:) = sin(linspace(0,tf,pts));
u(5,:) = sin(linspace(0,10*tf,pts));
u(8,:) = .7*sin(linspace(0,10*tf,pts));
u(12,:) = -sin(linspace(0,10*tf,pts));
u(15,:) = -.7*sin(linspace(0,10*tf,pts));
utraj = PPTrajectory(foh(linspace(0,tf,pts),u));
utraj = setOutputFrame(utraj, p.getInputFrame);

sys = cascade(utraj,p);

xtraj = sys.simulate([0 tf],x0);

v = p.constructVisualizer();
v.playback(xtraj, struct('slider',true));

end
