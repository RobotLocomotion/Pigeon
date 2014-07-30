function [xstar, ustar] = findTrim(p)

if (nargin<1)
  options.floating = true;
  p = RigidBodyManipulator('pigeon_17.URDF', options);
  
end

prog = FixedPointProgram(p,[1;3]);  % ignores xdot and zdot constraints

lb = [0 0 5 0 -1.5 0 -1.5*ones(1,29) ...  % positions
    20 0 -2 0 0 0 zeros(1,29) ...               % velocities
    -Inf*ones(1,29)]';                          % torques
ub = [0 0 5 0 0 0 1.5*ones(1,29) ...
    30 0 0 0 0 0 zeros(1,29) ...
    Inf*ones(1,29)]';

A = [zeros(1,9) 1 zeros(1,6) -1 zeros(1,82);    % constraints for symmetry
     zeros(1,10) 1 zeros(1,6) -1 zeros(1,81);
     zeros(1,11) 1 zeros(1,6) -1 zeros(1,80);
     zeros(1,12) 1 zeros(1,6) -1 zeros(1,79);
     zeros(1,13) 1 zeros(1,6) -1 zeros(1,78);
     zeros(1,14) 1 zeros(1,6) -1 zeros(1,77);
     zeros(1,15) 1 zeros(1,6) -1 zeros(1,76);
     zeros(1,23) 1 zeros(1,5) -1 zeros(1,69);
     zeros(1,24) 1 zeros(1,5) -1 zeros(1,68);
     zeros(1,25) 1 zeros(1,5) -1 zeros(1,67);
     zeros(1,26) 1 zeros(1,5) -1 zeros(1,66);
     zeros(1,27) 1 zeros(1,5) -1 zeros(1,65);
     zeros(1,28) 1 zeros(1,5) -1 zeros(1,64)
     ];

prog = addConstraint(prog,BoundingBoxConstraint(lb,ub));
prog = addConstraint(prog,FunctionHandleObjective(99,@cost));
prog = addConstraint(prog,LinearConstraint(zeros(13,1),zeros(13,1),A));

x0 = Point(getStateFrame(p),zeros(70,1));
x0.base_xdot = 20;
x0.base_z = 5;
u0 = zeros(29,1);

w = prog.solve([double(x0);u0]); 
xstar = w(1:70);
ustar = w(71:end);

if (nargout<1)
  utraj = ConstantTrajectory(ustar);
  utraj = setOutputFrame(utraj,getInputFrame(p));
  sys = cascade(utraj,p);
  xtraj = simulate(sys,[0 .02],xstar);
  v = p.constructVisualizer(); 
  v.playback(xtraj, struct('slider',true));
end

end

function [c,dc] = cost(x)
    xdot = x(36);
    zdot = x(28);
    c = zdot/xdot;
    dc = [zeros(1,35) -zdot/xdot^2 0 1/xdot zeros(1,32) zeros(1,29)];
end
