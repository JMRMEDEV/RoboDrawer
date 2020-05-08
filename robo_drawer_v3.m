%% Initial Setup

% Robotic Toolbox (by Peter Corke) Initiation (Included in the project
% folder)
startup_rvc

% Execution of the communication script, for accessing Coppelia UR5
% remote simulation
ur5_coppelia_com

% Call to the function that reads points form a given picture
% Some samples are available in \RoboDrawer V3\Pictures\
coordinates = point_detector('C:\Users\JoshM\Desktop\RoboDrawer V3\Pictures\catbug.jpg');

%% UR5 Definition for its use in Robotics Toolbox

% Denavit-Hartenberg Parameters

%         theta    d          a     alpha
DH = [
            0  0.089159       0      pi/2
            0      0       -0.425      0
            0      0      -0.39225     0
            0   0.10915       0      pi/2
            0   0.09465       0     -pi/2
            0    0.0823       0        0
    ];
    
UR5 = SerialLink(DH);

%% Parameters definition

% Vector for initial condition for robotics toolbox
v_initial = [0 0 0 0 0 0];

% Vector for defining the work plane and Tool Central Point
v_final = [120 -40 40 0 30 0];
v_final = deg2rad(v_final);

% Vector that takes the picture points input and rezises it
resize_factor = 0.005;
W = resize_factor * coordinates;

% Vector meant for storaging all the calculated positions
full_traj = [];

% Determination of the Forward Kinematics of the new working
% point for some functionalities
MTH_final = UR5.fkine(v_final);

% Wait for the next instruction in Coppelia
res = vrep.simxPauseCommunication(client_id, true);

% Checks Coppelia API return code. Set buffer to 1 if you are reading 
% from a buffered call
vrchk(vrep, res);

% Set the arm to its starting configuration:
% go through the entire array for moving each joint
for i = 1:6
        res = vrep.simxSetJointTargetPosition(client_id, handles.ur5_joints(i),...
            v_initial(i),...
            vrep.simx_opmode_oneshot);
        vrchk(vrep, res, true);
end

% Wait for the next instruction
res = vrep.simxPauseCommunication(client_id, false);
vrchk(vrep, res);

%% Setting the new working point (just the frame)

% Get the rotational matrix from the transformation matrix
rot_mat = MTH_final(1:3, 1:3);
% Get the position offset from the transformation matrix
pos = MTH_final(1:3, 4);
% Compute the euler angle from the rotational matrix
theta_rot = tr2eul(rot_mat);

% Set the frame to the desired euler angle
res = vrep.simxSetObjectOrientation(client_id, handles.FrameEndTarget, handles.base, theta_rot, vrep.simx_opmode_oneshot);
vrchk(vrep, res, true);

% Set the frame to the desired position
res = vrep.simxSetObjectPosition(client_id, handles.FrameEndTarget, handles.base, pos, vrep.simx_opmode_oneshot);
vrchk(vrep, res, true);

%% Coppelia's UR5 Movement

% Move the arm to the new working point
for j = 1:6
    vrep.simxSetJointTargetPosition(client_id, handles.ur5_joints(j),...
        v_final(j), ...
        vrep.simx_opmode_oneshot);     
    vrchk(vrep, res);
end

% Creation of an array for checking if the desired point is reached
% (Not working)
current_joints = zeros(1, 6);
reached_point = false;

% Obtaining of the current position of each joint
while reached_point == false
    for j = 1:6
        [returnCode,current_joints(j)] = vrep.simxGetJointPosition(client_id, handles.ur5_joints(j),...
            vrep.simx_opmode_oneshot_wait);
        vrchk(vrep, res, true);
    end
    % Comparison between the current position and the initially
    % desired one
    if v_final(1) > current_joints(1) - 0.0001 || v_final(1) < current_joints(1) + 0.0001
        reached_point = true;
    end
end

% Time required for the arm to reach the new working point
pause(10);

% Drawing functionality. Iterates size(W) times, for each point of the
% picture
for i = 1:size(W)   

    % If i == 1, the end effector moves to the first
    % point of the drawing
    if i == 1

        % Through an Homogeneous Transformation Matrix, the working plane
        % and first point, are defined and set.
        new_point = transl(MTH_final(1,4), W(i,1), W(i,2)) * troty(pi/2);
        
        % The inverse kinematics of such HTM is obtained by the use
        % of ikunc from the robotics toolbox
        q1 = UR5.ikunc(new_point);
        
        % Once the inverse kinematics is obtained, the full_traj matrix
        % is updated
        full_traj(i,1) = q1(1);
        full_traj(i,2) = q1(2);
        full_traj(i,3) = q1(3);
        full_traj(i,4) = q1(4);
        full_traj(i,5) = q1(5);
        full_traj(i,6) = q1(6);

        % Tell the arm to move to the new position
        for j = 1:6
            vrep.simxSetJointTargetPosition(client_id, handles.ur5_joints(j),...
                q1(j), ...
                vrep.simx_opmode_oneshot);     
            vrchk(vrep, res);
        end
        
    % If i ~= 0, the continuous drawing is performed
    else
        new_point = transl(MTH_final(1,4), W(i,1), W(i,2)) * troty(pi/2);
        q1 = UR5.ikunc(new_point);
        full_traj(i,1) = q1(1);
        full_traj(i,2) = q1(2);
        full_traj(i,3) = q1(3);
        full_traj(i,4) = q1(4);
        full_traj(i,5) = q1(5);
        full_traj(i,6) = q1(6);
        
        % This 'half_point' is defined to put the drawing tool away
        % from the plane
        half_point = transl(MTH_final(1,4)-0.01, W(i,1), W(i,2)) * troty(pi/2);
        q1_2 = UR5.ikunc(half_point);
        
        prev_point = [full_traj(i-1,1) full_traj(i-1,2) full_traj(i-1,3) full_traj(i-1,4) full_traj(i-1,5) full_traj(i-1,6)];

        % This pause is meant to let the end effector reach every point
        pause(1.8);
        
        for j = 1:6
            vrep.simxSetJointTargetPosition(client_id, handles.ur5_joints(j),...
                prev_point(j), ...
                vrep.simx_opmode_oneshot);     
            vrchk(vrep, res);
        end
        
        pause(1.8);
        
        % UR5 move away from the plane
        for j = 1:6
            vrep.simxSetJointTargetPosition(client_id, handles.ur5_joints(j),...
                half_point(j), ...
                vrep.simx_opmode_oneshot);     
            vrchk(vrep, res);
        end
        pause(0.4);
        
        % UR5 move to the next drawing point
        for j = 1:6
            vrep.simxSetJointTargetPosition(client_id, handles.ur5_joints(j),...
                q1(j), ...
                vrep.simx_opmode_oneshot);     
            vrchk(vrep, res);                
        end
        pause(1.8);
        
        % Sets a 'dummy' in Coppelia, for indicating where the tool 
        % drawed a point
        copyf(client_id,vrep,new_point,handles.dummy,-1); 
    end
end

% Go back to the starting position
for i = 1:6
    res = vrep.simxSetJointTargetPosition(client_id, handles.ur5_joints(i),...
        v_initial(i),...
        vrep.simx_opmode_oneshot);
    vrchk(vrep, res, true);
end

res = vrep.simxPauseCommunication(client_id, false);
vrchk(vrep, res);
