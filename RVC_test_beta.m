startup_rvc

coordinates = point_detector('C:\Users\JoshM\Desktop\RoboDrawer V3\Pictures\bart.jpg');

%% Denavit-Hartenberg Parameters
%         theta    d          a     alpha
DH = [
            0  0.089159       0      pi/2
            0      0       -0.425      0
            0      0      -0.39225     0
            0   0.10915       0      pi/2
            0   0.09465       0     -pi/2
            0    0.0823       0        0
    ];
    
UR5_1 = SerialLink(DH);
UR5_2 = SerialLink(DH);

v_final = [120 -40 40 0 30 0];
v_final = deg2rad(v_final);
W=0.01*coordinates;

full_traj = [];

figure('Name', 'UR5_MTH');
t = (0:0.01:0.05);

MTH_final = UR5_1.fkine(v_final);

for n1 = 1:length(coordinates)
    coordinates(n1,1) = coordinates(n1,1) + 30;
    coordinates(n1,2) = coordinates(n1,2) - 50;
    xlim([-100 100]);
    plot(coordinates(n1,1),coordinates(n1,2), 'o');
end

for i = 1:size(W)
    if i == 1
        q_in = UR5_1.ikunc(MTH_final);
        new_point = transl(0.535736116985854, W(i,1), W(i,2)) * troty(pi/2);
        q1 = UR5_1.ikunc(new_point);
        full_traj(i,1) = q1(1);
        full_traj(i,2) = q1(2);
        full_traj(i,3) = q1(3);
        full_traj(i,4) = q1(4);
        full_traj(i,5) = q1(5);
        full_traj(i,6) = q1(6);
        k1 = jtraj(q_in, q1, t);
        UR5_1.plot(k1);
    else
        new_point = transl(0.535736116985854, W(i,1), W(i,2)) * troty(pi/2);
        q1 = UR5_1.ikunc(new_point);
        full_traj(i,1) = q1(1);
        full_traj(i,2) = q1(2);
        full_traj(i,3) = q1(3);
        full_traj(i,4) = q1(4);
        full_traj(i,5) = q1(5);
        full_traj(i,6) = q1(6);
        prev_point = [full_traj(i-1,1) full_traj(i-1,2) full_traj(i-1,3) full_traj(i-1,4) full_traj(i-1,5) full_traj(i-1,6)];
        half_point = transl(0.52, W(i,1), W(i,2)) * troty(pi/2);
        q1_2 = UR5_1.ikunc(half_point);
        k1 = jtraj(prev_point, q1_2, t);
        UR5_1.plot(k1);
        k2 = jtraj(q1_2, q1, t);
        UR5_1.plot(k1);
        plot3(0.535736116985854, W(i,1), W(i,2), '.');
        xlim([-1 1]);
        ylim([-1 1]);
        zlim([-1 1]);
        hold on;
    end
end

