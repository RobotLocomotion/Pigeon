function [xstar, ustar] = findTrimWeldedLegs(p)

if (nargin<1)
  options.floating = true;
  p = RigidBodyManipulator('pigeon_17.URDF', options);
  
  p = p.weldJoint('tail_roll');
  p = p.weldJoint('tail_yaw');
  p = p.weldJoint('left_hip_roll');
  p = p.weldJoint('left_hip_pitch');
  p = p.weldJoint('left_knee_pitch');
  p = p.weldJoint('left_ankle_pitch');
  p = p.weldJoint('left_thumb_pitch');
  p = p.weldJoint('left_fingers_pitch');
  p = p.weldJoint('right_hip_roll');
  p = p.weldJoint('right_hip_pitch');
  p = p.weldJoint('right_knee_pitch');
  p = p.weldJoint('right_ankle_pitch');
  p = p.weldJoint('right_thumb_pitch');
  p = p.weldJoint('right_fingers_pitch');
  p = p.compile();
  
end

prog = FixedPointProgram(p,[1;3]);  % ignores xdot and zdot constraints

lb = [0 0 5 0 -1.5 0 -1.5*ones(1,15) ...  % positions
    20 0 -2 0 0 0 zeros(1,15) ...               % velocities
    -Inf*ones(1,15)]';                          % torques
ub = [0 0 5 0 0 0 1.5*ones(1,15) ...
    30 0 0 0 0 0 zeros(1,15) ...
    Inf*ones(1,15)]';

% A = [zeros(1,7) 1 zeros(1,6) -1 zeros(1,42);    % constraints for symmetry
%      zeros(1,8) 1 zeros(1,6) -1 zeros(1,41);
%      zeros(1,9) 1 zeros(1,6) -1 zeros(1,40);
%      zeros(1,10) 1 zeros(1,6) -1 zeros(1,39);
%      zeros(1,11) 1 zeros(1,6) -1 zeros(1,38);
%      zeros(1,12) 1 zeros(1,6) -1 zeros(1,37);
%      zeros(1,13) 1 zeros(1,6) -1 zeros(1,36);
%      ];

A = zeros(7, size(p))

prog = addConstraint(prog,BoundingBoxConstraint(lb,ub));
prog = addConstraint(prog,FunctionHandleObjective(57,@cost));
prog = addConstraint(prog,LinearConstraint(zeros(7,1),zeros(7,1),A));

x0 = Point(getStateFrame(p),zeros(42,1));
x0.base_xdot = 20;
x0.base_z = 5;
u0 = zeros(15,1);

w = prog.solve([double(x0);u0]); 
xstar = w(1:42);
ustar = w(43:end);

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
    xdot = x(22);
    zdot = x(24);
    c = zdot/xdot;
    dc = [zeros(1,21) -zdot/xdot^2 0 1/xdot zeros(1,15) zeros(1,15)];
end
