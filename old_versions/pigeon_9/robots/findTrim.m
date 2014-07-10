function [ xsol,usol,exitflag ] = findTrim( )

options.floating = true;
p = RigidBodyManipulator('pigeon_8.URDF', options);

x0 = zeros(42,1);
x0(1) = 5;
u0 = zeros(15,1);

X = [x0;u0];
options.MaxFunEvals = 10000;
options.Algorithm = 'active-set';
A = zeros(57);
b = zeros(57,1);
tol = 1E-4;
lb = [1 1 1 -1.5 -1.5 -1.5 -1.5*ones(1,15) 40 0 -tol 0 0 0 zeros(1,15) -Inf*ones(1,15)]';
ub = [1 1 1 1.5 1.5 1.5 1.5*ones(1,15) 50 0 tol 0 0 0 zeros(1,15) Inf*ones(1,15)]';
[sol,~,exitflag] = fmincon(@minfun, X, A, b, [], [], lb, ub, [], options);
    
function y = minfun(inX)
    x = inX(1:42);
    u = inX(43:57);
    xdot = p.dynamics(0,x,u);
    y = xdot(4:6)'*xdot(4:6);
    y = y+100*xdot(3)^2;
    y = y+100*xdot(24)^2;
end

xsol = sol(1:42);
usol = sol(43:57);

tf = 1;
dt = .01;
pts = tf/dt;

options.floating = true;
p = TimeSteppingRigidBodyManipulator('pigeon_8.URDF',dt,options);
p = enableIdealizedPositionControl(p,true);
p = compile(p);

u = xsol(7:21);
utraj = PPTrajectory(foh(linspace(0,tf,pts),repmat(u,1,pts)));
utraj = setOutputFrame(utraj, p.getInputFrame);

sys = cascade(utraj,p);

xtraj = sys.simulate([0 tf],xsol);

v = p.constructVisualizer();
v.playback(xtraj, struct('slider',true));

end

