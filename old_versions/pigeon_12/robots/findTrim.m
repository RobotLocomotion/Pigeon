function [xstar, ustar] = findTrim(p)

if (nargin<1)
  options.floating = true;
  p = RigidBodyManipulator('pigeon_12.URDF', options);
end

prog = FixedPointProgram(p,[1;3]);  % ignores xdot and zdot constraints

lb = [0 0 5 -1.5 -1.5 -1.5 -1.5*ones(1,16) ...  % positions
    20 0 -2 0 0 0 zeros(1,16) ...               % velocities
    -Inf*ones(1,16)]';                          % torques
ub = [0 0 5 1.5 0 1.5 1.5*ones(1,16) ...
    30 0 0 0 0 0 zeros(1,16) ...
    Inf*ones(1,16)]';

prog = addConstraint(prog,BoundingBoxConstraint(lb,ub));
prog = addConstraint(prog,FunctionHandleObjective(60,@cost));

x0 = Point(getStateFrame(p),zeros(44,1));
x0.base_xdot = 20;
x0.base_z = 5;
u0 = zeros(16,1);

w = prog.solve([double(x0);u0]); 
xstar = w(1:44);
ustar = w(45:end);

if (nargout<1)
  utraj = ConstantTrajectory(ustar);
  utraj = setOutputFrame(utraj,getInputFrame(p));
  sys = cascade(utraj,p);
  xtraj = simulate(sys,[0 .1],xstar);
  v = p.constructVisualizer(); 
  v.playback(xtraj, struct('slider',true));
end

end

function [c,dc] = cost(x)
    xdot = x(23);
    zdot = x(25);
    c = zdot/xdot;
    dc = [zeros(1,22) -zdot/xdot^2 0 1/xdot zeros(1,19) zeros(1,16)];
end
