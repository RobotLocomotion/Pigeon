%start
x0 = [zeros(1,58)]';

%shoulders
x1 = [zeros(1,3), -0.785, zeros(1,6), 0.785, zeros(1, 47)]';
x2 = [zeros(1,3), 0.785, zeros(1,6), -0.785, zeros(1, 47)]';
x3 = [zeros(1,58)]';
x4 = [zeros(1,4), -0.785, zeros(1,6), -0.785, zeros(1, 46)]';
x5 = [zeros(1,4), 0.785, zeros(1,6), 0.785, zeros(1, 46)]';
x6 = [zeros(1,58)]';
x7 = [zeros(1,5), -0.785, zeros(1,6), -0.785, zeros(1, 45)]';
x8 = [zeros(1,5), 0.785, zeros(1,6), 0.785, zeros(1, 45)]';
x9 = [zeros(1,58)]';

%elbows
x10 = [zeros(1,6), 0.785, zeros(1,6), 0.785, zeros(1, 44)]';
x11 = [zeros(1,6), -0.785, zeros(1,6), -0.785, zeros(1, 44)]';
x12 = [zeros(1,58)]';

%wrists
x13 = [zeros(1,7), -0.785, zeros(1,6), 0.785, zeros(1, 43)]';
x14 = [zeros(1,7), 0.785, zeros(1,6), -0.785, zeros(1, 43)]';
x15 = [zeros(1,58)]';
x16 = [zeros(1,8), -0.785, zeros(1,6), 0.785, zeros(1, 42)]';
x17 = [zeros(1,8), 0.785, zeros(1,6), -0.785, zeros(1, 42)]';
x18 = [zeros(1,58)]';
x19 = [zeros(1,9), -0.785, zeros(1,6), -0.785, zeros(1, 41)]';
x20 = [zeros(1,9), 0.785, zeros(1,6), 0.785, zeros(1, 41)]';
x21 = [zeros(1,58)]';

%tail
x22 = [-0.785 zeros(1,57)]';
x23 = [0.785 zeros(1,57)]';
x24 = [zeros(1,58)]';
x25 = [0 -0.785 zeros(1,56)]';
x26 = [0 0.785 zeros(1,56)]';
x27 = [zeros(1,58)]';
x28 = [0 0 -0.785, zeros(1,55)]';
x29 = [0 0 0.785, zeros(1,55)]';
x30 = [zeros(1,58)]';

%hips
x31 = [zeros(1,17), -0.785, zeros(1,5), 0.785, zeros(1, 34)]';
x32 = [zeros(1,58)]';
x33 = [zeros(1,18), 1.571, zeros(1,5), 1.571, zeros(1, 33)]';
x34 = [zeros(1,58)]';

%knees
x35 = [zeros(1,19), 0.785, zeros(1,5), 0.785, zeros(1, 32)]';
x36 = [zeros(1,58)]';

%ankles
x37 = [zeros(1,20), 0.785, zeros(1,5), 0.785, zeros(1, 31)]';
x38 = [zeros(1,58)]';

%thumbs
x39 = [zeros(1,21), -0.785, zeros(1,5), -0.785, zeros(1, 30)]';
x40 = [zeros(1,58)]';

%fingers
x41 = [zeros(1,22), 0.785, zeros(1,5), 0.785, zeros(1, 29)]';
x42 = [zeros(1,58)]';



poses = [ x0, x1, x2, x3, x4, x5, x6, x7, x8, x9, ...
          x10, x11, x12, x13, x14, x15, x16, x17, x18, x19, ...
          x20, x21, x22, x23, x24, x25, x26, x27, x28, x29, ... 
          x30, x31, x32, x33, x34, x35, x36, x37, x38, x39, ...
          x40, x41, x42 ];

times = [ 0, ...                                    %start
          1, 3, 4, 5, 7, 8, 9, 11, 12, ...          %shoulders
          13, 15, 16, ...                           %elbows
          17, 19, 20, 21, 23, 24, 25, 27, 28, ...   %wrists
          29, 31, 32, 33, 35, 36, 37, 39, 40, ...   %tail
          41, 42, 44, 46, ...                       %hips
          47, 48, ...                               %knees
          49, 50, ...                               %ankles
          51, 52, ...                               %thumbs
          53, 54, ...                               %fingers
         ]*60/43;
xtraj = PPTrajectory(foh(times,poses));

pigeon = RigidBodyManipulator('pigeon_17.URDF');
xtraj = setOutputFrame(xtraj,getStateFrame(pigeon));
v = constructVisualizer(pigeon);

  v.playback(xtraj, struct('slider',true));