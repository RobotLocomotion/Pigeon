function [ xsol,usol,exitflag ] = findTrim( )

options.floating = true;
p = RigidBodyManipulator('pigeon_10.URDF', options);

prog = FixedPointProgram(p,[1;3]);  % ignores xdot and zdot constraints

lb = [0 0 0 -1.5 -1.5 -1.5 -1.5*ones(1,15) ...  % positions
    20 0 -2 0 0 0 zeros(1,15) ...               % velocities
    -Inf*ones(1,15)]';                          % torques
ub = [0 0 0 1.5 0 1.5 1.5*ones(1,15) ...
    30 0 0 0 0 0 zeros(1,15) ...
    Inf*ones(1,15)]';

prog = addBoundingBoxConstraint(prog,BoundingBoxConstraint(lb,ub));

x0 = Point(getStateFrame(p),zeros(42,1));
x0.base_xdot = 20;
u0 = zeros(15,1);

w = prog.solve([double(x0);u0]); 
xstar = w(1:42)
ustar = w(43:end)


sys = cascade(setOutputFrame(ConstantTrajectory(ustar),getInputFrame(p)),p);
v = p.constructVisualizer();
simulate(cascade(sys,v),[0 1],xstar);

%xtraj = sys.simulate([0 1],xstar);
%v.playback(xtraj, struct('slider',true));


return;

X = [x0;u0];
options.MaxFunEvals = 10000;
options.Algorithm = 'active-set';

[sol,~,exitflag] = fmincon(@minfun, X, [], [], [], [], lb, ub, [], options);
    
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
p = TimeSteppingRigidBodyManipulator('pigeon_10.URDF',dt,options);
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

