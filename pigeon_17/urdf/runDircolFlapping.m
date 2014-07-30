function [utraj, xtraj] = runDircolFlapping

options.floating = true;
p = RigidBodyManipulator('pigeon_12.URDF', options);

N = 11;
minimum_duration = .1;
maximum_duration = 3;
prog = DircolTrajectoryOptimization(p,N,[minimum_duration maximum_duration]); 

prog = prog.setSolverOptions('snopt','superbasicslimit',2000);
% prog = prog.setCheckGrad(true);
prog = prog.addDisplayFunction(@(x)displayfun(x,N,p.getNumStates(),p.getNumInputs()));

% Ideally this is what we want
tol = zeros(p.getNumStates(),1);
tol(1) = Inf; % ignore horizontal position
% but we should make this work first
% this only cares about the altitude
 tol = Inf*ones(p.getNumStates(),1);
 tol(3) = .1; % only require the bird to keep its altitude
prog = prog.addStateConstraint(PeriodicConstraint(tol),{[1 N]});

prog = prog.addRunningCost(@cost);
prog = prog.addFinalCost(@finalcost);

display('Finding trim conditions for initial guess...')
[xstar,ustar] = findTrim(p);
tf0 = 1;  % initial guess at duration 
traj_init.x = PPTrajectory(foh([0,tf0],[double(xstar),...
  double(xstar)+tf0*[double(xstar(23:44));zeros(22,1)]]));
traj_init.u = ConstantTrajectory(ustar);

display('Starting trajectory optimization...')
tic
[xtraj,utraj,z,F,info] = prog.solveTraj(tf0,traj_init);
toc

if (nargout<1)
  v = constructVisualizer(p);
  v.playback_speed = .2;
  v.playback(xtraj,struct('slider',true));
end

end

function [g,dg] = cost(t,x,u)
  Q = zeros(44,44);
  Q(4:6,4:6) = 10000*eye(3); % penalize roll pitch and yaw
  Q(26:28,26:28) = 1000*eye(3); % penalize rolldot, pitchdot, yawdot
  R = eye(16);
  g = x'*Q*x + u'*R*u;
  if (nargout>1)
    dg = [0,2*x'*Q,2*u'*R];
  end
end

function [h,dh] = finalcost(t,x)
  h = t;
  if (nargout>1)
    dh = [1,zeros(1,size(x,1))];
  end
end

function displayfun(X,num_t,num_x,num_u)
  h = X(1:num_t-1);
  x = X(num_t-1+[1:num_x*num_t]);
  u = X(num_t-1+num_x*num_t+[1:num_u*num_t]);
  x = reshape(x,num_x,num_t);
  u = reshape(u,num_u,num_t);
  figure(5);
  plot3(x(1,:),x(2,:),x(3,:));
  % plot(x(1,:),x(3,:));
  % plot([0,cumsum(h)'],x(1,:));
end
