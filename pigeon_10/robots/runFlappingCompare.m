function xtraj = runFlapping( )

tf = 1;
dt = .007;
pts = tf/dt;

options.floating = true;
p = TimeSteppingRigidBodyManipulator('pigeon_10.URDF',dt,options);
p = enableIdealizedPositionControl(p,true);
p = compile(p);
x0 = Point(p.getStateFrame());  % initial state
x0.base_z = 0;          % initial height
x0.base_xdot = 20;      % initial forward velocity
x0.base_pitch = -.2;    % initial pitch

xn = update(p,0,double(x0),zeros(15,1));

p_old = TimeSteppingRigidBodyManipulator('../../old_versions/pigeon_3/robots/pigeon_3.URDF',dt,options);
p_old = enableIdealizedPositionControl(p_old,true);
p_old = compile(p_old);

x0_old = Point(p_old.getStateFrame());  % initial state
x0_old.base_z = 0;          % initial height
x0_old.base_ydot = 20;      % initial forward velocity
x0_old.base_pitch = .2;    % initial pitch

xn_old = update(p_old,0,double(x0_old),zeros(15,1));

valuecheck(xn,xn_old);


u = zeros(15,pts);
u(1,:) = sin(linspace(0,tf,pts));
u(5,:) = -sin(linspace(0,10*tf,pts));
u(8,:) = -.7*sin(linspace(0,10*tf,pts));
u(12,:) = sin(linspace(0,10*tf,pts));
u(15,:) = .7*sin(linspace(0,10*tf,pts));
utraj = PPTrajectory(foh(linspace(0,tf,pts),u));
utraj = setOutputFrame(utraj, p.getInputFrame);

sys = p ;% cascade(utraj,p);

if (1)
  v = p.constructVisualizer();
  sys = cascade(sys,v);
  simulate(sys,[0 tf],x0);
else  
  xtraj = sys.simulate([0 tf],x0);

  v = p.constructVisualizer();
  v.playback(xtraj, struct('slider',true));
end

end

