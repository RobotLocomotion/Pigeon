function [utraj, xtraj, v, p] = runDircolFlapping(p, utraj0)

tf0 = 1;
N = 20;
if (nargin<1)
  options.floating = true;
  p = RigidBodyManipulator('pigeon_10.URDF', options);
end
if (nargin<2)
    utraj0 = PPTrajectory(foh(linspace(0,tf0,N),0*randn(p.getNumInputs(),N)));
end
x0 = Point(p.getStateFrame());
x0.base_z = 5;
x0.base_xdot = 20;
xf_lb = x0;
xf_lb.base_x = -100;
xf_ub = x0;
xf_ub.base_x = 100;
x0 = double(x0);
xf_lb = double(xf_lb);
xf_ub = double(xf_ub);

con.u.lb = p.umin;
con.u.ub = p.umax;
con.x0.lb = x0;
con.x0.ub = x0;
con.xf.lb = xf_lb;
con.xf.ub = xf_ub;

options.method='dircol';
options.grad_method={'user'};

tic
disp('Starting optimization');
[utraj,xtraj,info] = trajectoryOptimization(p,@cost,@finalcost,x0,utraj0,con,options);
if (info~=1) error('failed to find a trajectory'); end
toc

v = p.constructVisualizer();
v.playback_speed = .2;
v.playback(xtraj, struct('slider',true));

end

      function [g,dg] = cost(t,x,u)
        R = ones(15,15);
        g = u'*R*u;
        if (nargout>1)
          dg = [0,zeros(1,size(x,1)),2*u'*R];
        end
      end
        
      function [h,dh] = finalcost(t,x)
        h = t;
        if (nargout>1)
          dh = [1,zeros(1,size(x,1))];
        end
      end

