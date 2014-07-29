%start
x0 = [zeros(1,56)]';

%shoulders
x1 = [zeros(1,2), -0.785, zeros(1,6), 0.785, zeros(1, 46)]';
x2 = [zeros(1,2), 0.785, zeros(1,6), -0.785, zeros(1, 46)]';
x3 = [zeros(1,56)]';
x4 = [zeros(1,3), -0.785, zeros(1,6), -0.785, zeros(1, 45)]';
x5 = [zeros(1,3), 0.785, zeros(1,6), 0.785, zeros(1, 45)]';
x6 = [zeros(1,56)]';
x7 = [zeros(1,4), -0.785, zeros(1,6), -0.785, zeros(1, 44)]';
x8 = [zeros(1,4), 0.785, zeros(1,6), 0.785, zeros(1, 44)]';
x9 = [zeros(1,56)]';

%elbows
x10 = [zeros(1,5), 0.785, zeros(1,6), 0.785, zeros(1, 43)]';
x11 = [zeros(1,5), -0.785, zeros(1,6), -0.785, zeros(1, 43)]';
x12 = [zeros(1,56)]';

%wrists
x13 = [zeros(1,6), -0.785, zeros(1,6), 0.785, zeros(1, 42)]';
x14 = [zeros(1,6), 0.785, zeros(1,6), -0.785, zeros(1, 42)]';
x15 = [zeros(1,56)]';
x16 = [zeros(1,7), -0.785, zeros(1,6), 0.785, zeros(1, 41)]';
x17 = [zeros(1,7), 0.785, zeros(1,6), -0.785, zeros(1, 41)]';
x18 = [zeros(1,56)]';
x19 = [zeros(1,8), -0.785, zeros(1,6), -0.785, zeros(1, 40)]';
x20 = [zeros(1,8), 0.785, zeros(1,6), 0.785, zeros(1, 40)]';
x21 = [zeros(1,56)]';

%tail
x22 = [-0.785, zeros(1,55)]';
x23 = [-0.785, -0.524, zeros(1,54)]';
x24 = [-0.785, 0.524, zeros(1,54)]';
x25 = [-0.785, zeros(1,55)]';
x26 = [0.785, zeros(1,55)]';
x27 = [zeros(1,56)]';

%hips
x28 = [zeros(1,16), -0.785, zeros(1,5), 0.785, zeros(1, 33)]';
x29 = [zeros(1,56)]';
x30 = [zeros(1,17), 0.785, zeros(1,5), 0.785, zeros(1, 32)]';
x31 = [zeros(1,56)]';

%knees
x32 = [zeros(1,18), 0.785, zeros(1,5), 0.785, zeros(1, 31)]';
x33 = [zeros(1,56)]';

%ankles
x34 = [zeros(1,19), 0.785, zeros(1,5), 0.785, zeros(1, 30)]';
x35 = [zeros(1,56)]';

%thumbs
x36 = [zeros(1,20), -0.785, zeros(1,5), -0.785, zeros(1, 29)]';
x37 = [zeros(1,56)]';

%fingers
x38 = [zeros(1,21), 0.785, zeros(1,5), 0.785, zeros(1, 28)]';
x39 = [zeros(1,56)]';



poses = [ x0, x1, x2, x3, x4, x5, x6, x7, x8, x9, ...
          x10, x11, x12, x13, x14, x15, x16, x17, x18, x19, ...
          x20, x21, x22, x23, x24, x25, x26, x27, x28, x29, ... 
          x30, x31, x32, x33, x34, x35, x36, x37, x38, x39 ];

times = (0:1:39)*60/40;
xtraj = PPTrajectory(foh(times,poses));

pigeon = RigidBodyManipulator('pigeon_17.URDF');
xtraj = setOutputFrame(xtraj,getStateFrame(pigeon));
v = constructVisualizer(pigeon);

  v.playback(xtraj, struct('slider',true));