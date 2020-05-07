%% The first block of code is retrieved from https://www.youtube.com/watch?v=WaYBTA6QPY0

vrep=remApi('remoteApi');
vrep.simxFinish(-1);
client_id = vrep.simxStart('127.0.0.1', 19997, true, true, 2000, 5);

if client_id < 0,
    disp('Failed connecting to remote API server. Exiting.');
    vrep.delete();
    return;
end

fprintf('Connection to remote API server open.\n');

%% Construct the joints frame for UR5

handles = struct('client_id', client_id);
jointNames={'UR5_joint1','UR5_joint2','UR5_joint3','UR5_joint4',...
    'UR5_joint5','UR5_joint6'};
ur5_joints = -ones(1,6);

for i = 1:6
    [res, ur5_joints(i)] = vrep.simxGetObjectHandle(client_id, ...
        jointNames{i}, vrep.simx_opmode_oneshot_wait);
    vrchk(vrep, res);
end

handles.ur5_joints = ur5_joints;
[res, ur5Ref] = vrep.simxGetObjectHandle(client_id, 'UR5', ...
    vrep.simx_opmode_oneshot_wait);
vrchk(vrep, res);
[res, ur5Gripper] = vrep.simxGetObjectHandle(client_id, 'UR5_connection', ...
    vrep.simx_opmode_oneshot_wait);
vrchk(vrep, res);
handles.ur5Ref = ur5Ref;
handles.ur5Gripper = ur5Gripper;

%% Construct the Base frame

[res, handles.base] = vrep.simxGetObjectHandle(client_id, ...
    'Frame0', vrep.simx_opmode_oneshot_wait);
vrchk(vrep, res);

%% Construct coordinates for each joint and endeffctor
% And compute the transformation from base to each frame at default

handles.FrameEnd = copyf( client_id, vrep, eye(4), handles.base, handles.base);
handles.FrameEndTarget = copyf( client_id, vrep, eye(4), handles.base, handles.base);
vrchk(vrep, res);

%% Construct the dummy

[res, handles.dummy] = vrep.simxGetObjectHandle(client_id, ...
    'Dummy', vrep.simx_opmode_oneshot_wait);
vrchk(vrep, res);

%% Other Initialization Stuff

% Stream wheel angles, Hokuyo data, and robot pose (see usage below)
% Wheel angles are not used in this example, but they may/will be necessary in
% your project.

for i = 1:6,
    res = vrep.simxGetJointPosition(client_id, handles.ur5_joints(i),...
        vrep.simx_opmode_streaming);
    vrchk(vrep, res, true);
end

res = vrep.simxGetObjectPosition(client_id, handles.ur5Ref, -1,...
    vrep.simx_opmode_streaming);
vrchk(vrep, res, true);
res = vrep.simxGetObjectOrientation(client_id, handles.ur5Ref, -1,...
    vrep.simx_opmode_streaming);
vrchk(vrep, res, true);

% Stream the arm joint angles and the tip position/orientation

res = vrep.simxGetObjectPosition(client_id, handles.ur5Gripper, handles.ur5Ref, vrep.simx_opmode_streaming);
vrchk(vrep, res, true);
res = vrep.simxGetObjectOrientation(client_id, handles.ur5Gripper, handles.ur5Ref, vrep.simx_opmode_streaming);
vrchk(vrep, res, true);

for i = 1:6,
    res = vrep.simxGetJointPosition(client_id, handles.ur5_joints(i),...
        vrep.simx_opmode_streaming);
    vrchk(vrep, res, true);
end

vrep.simxGetPingTime(client_id); % make sure that all streaming data has reached the client at least once
res = vrep.simxStartSimulation(client_id, vrep.simx_opmode_oneshot_wait);