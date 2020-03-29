%% Read and process csv motion-capture data, exported from Motive:Tracker software.
% The world frame constructed in motion-capture system is y-up frame, requiring a transformation during post-processing.    
% Empty frame should be dumped.
% Perform Kalmann Filter to smooth the trajectory...

%% Parameters to setup
% Names set up in Motive:Track for each part
L_UP_STR = 'LeftUpperarm';
L_FR_STR = 'LeftForearm';
L_HD_STR = 'LeftHand';
R_UP_STR = 'RightUpperarm';
R_FR_STR = 'RightForearm';
R_HD_STR = 'RightHand';
% mocap data file exported from Motive:Track
file_name = %'demo/demo1.csv'; %'circle_motion.csv'; %'sweep_motion.csv'; %'pull_motion.csv'; %'point_motion.csv'; %'LeftHandInitState.csv';


%% Read .csv mocap data
% Get indices for each rigid bodies
tmp = importdata(file_name);
textdata = regexp(tmp.textdata{3, 1}, ',', 'split'); % split the string by ',' delimiter
L_UP_ID = find(strcmp(textdata, L_UP_STR), 1, 'first');
L_FR_ID = find(strcmp(textdata, L_FR_STR), 1, 'first');
L_HD_ID = find(strcmp(textdata, L_HD_STR), 1, 'first'); 
R_UP_ID = find(strcmp(textdata, R_UP_STR), 1, 'first'); %3; % upperarm
R_FR_ID = find(strcmp(textdata, R_FR_STR), 1, 'first'); %23; % forearm
R_HD_ID = find(strcmp(textdata, R_HD_STR), 1, 'first'); %47; % hand
% read numeric data
data = csvread(file_name, 7, 0); % pure numeric data(frame, time, rigid boy quaternion/position, markers)  
% quaternion is read in as (w,x,y,z); pos is (x,y,z) with 'meters' as units.
% note: even at different initial locations, the pos is w.r.t the world frame determined during calibration!!!  
% note: initial frames are parallel to the world frame.
left_upperarm = struct('quat', [data(:, L_UP_ID+3), data(:, L_UP_ID:L_UP_ID+2)], 'pos', data(:, L_UP_ID+4:L_UP_ID+6)); % (x,y,z,w), read in as (w,x,y,z)
left_forearm = struct('quat', [data(:, L_FR_ID+3), data(:, L_FR_ID:L_FR_ID+2)], 'pos', data(:, L_FR_ID+4:L_FR_ID+6));
left_hand = struct('quat', [data(:, L_HD_ID+3), data(:, L_HD_ID:L_HD_ID+2)], 'pos', data(:, L_HD_ID+4:L_HD_ID+6));
right_upperarm = struct('quat', [data(:, R_UP_ID+3), data(:, R_UP_ID:R_UP_ID+2)], 'pos', data(:, R_UP_ID+4:R_UP_ID+6)); % (x,y,z,w), read in as (w,x,y,z)
right_forearm = struct('quat', [data(:, R_FR_ID+3), data(:, R_FR_ID:R_FR_ID+2)], 'pos', data(:, R_FR_ID+4:R_FR_ID+6));
right_hand = struct('quat', [data(:, R_HD_ID+3), data(:, R_HD_ID:R_HD_ID+2)], 'pos', data(:, R_HD_ID+4:R_HD_ID+6));
time = data(:, 2);


%% Post-processing
% delete empty frame
empty_id = [];
for i = 1 : size(data, 1)
    if(sum(left_upperarm.quat(i, :))==0 || sum(left_upperarm.pos(i, :))==0 || ...
       sum(left_forearm.quat(i, :))==0 || sum(left_forearm.pos(i, :))==0 || ...
       sum(left_hand.quat(i, :))==0 || sum(left_hand.pos(i, :))==0 || ...
       sum(right_upperarm.quat(i, :))==0 || sum(right_upperarm.pos(i, :))==0 || ...
       sum(right_forearm.quat(i, :))==0 || sum(right_forearm.pos(i, :))==0 || ...
       sum(right_hand.quat(i, :))==0 || sum(right_hand.pos(i, :))==0)
           empty_id = [empty_id, i];
    end
end
disp(['The frame that has lost information: ', num2str(empty_id)]);
left_upperarm.quat(empty_id, :) = [];
left_upperarm.pos(empty_id, :) = [];
left_forearm.quat(empty_id, :) = [];
left_forearm.pos(empty_id, :) = [];
left_hand.quat(empty_id, :) = [];
left_hand.pos(empty_id, :) = [];
right_upperarm.quat(empty_id, :) = [];
right_upperarm.pos(empty_id, :) = [];
right_forearm.quat(empty_id, :) = [];
right_forearm.pos(empty_id, :) = [];
right_hand.quat(empty_id, :) = [];
right_hand.pos(empty_id, :) = [];
time(empty_id, :) = [];
data(empty_id, :) = [];


% convert from y-up frame to z-up frame
%
quat_shift = eul2quat([pi, -pi/2, -pi/2], 'ZYZ'); % (w,x,y,z)
for i = 1 : size(data, 1)
    % prepare
    l_up_quat = quatmultiply(quat_shift, left_upperarm.quat(i, :));
    l_fr_quat = quatmultiply(quat_shift, left_forearm.quat(i, :));
    l_hd_quat = quatmultiply(quat_shift, left_hand.quat(i, :)); 
    l_up_pos = (quat2rotm(quat_shift)*left_upperarm.pos(i, :)')';
    l_fr_pos = (quat2rotm(quat_shift)*left_forearm.pos(i, :)')';
    l_hd_pos = (quat2rotm(quat_shift)*left_hand.pos(i, :)')';
    r_up_quat = quatmultiply(quat_shift, right_upperarm.quat(i, :));
    r_fr_quat = quatmultiply(quat_shift, right_forearm.quat(i, :));
    r_hd_quat = quatmultiply(quat_shift, right_hand.quat(i, :)); 
    r_up_pos = (quat2rotm(quat_shift)*right_upperarm.pos(i, :)')';
    r_fr_pos = (quat2rotm(quat_shift)*right_forearm.pos(i, :)')';
    r_hd_pos = (quat2rotm(quat_shift)*right_hand.pos(i, :)')';
    % convert to RViz UR5 frame of reference
    quat_up_shift_rviz_l = eul2quat([pi/2, 0, 0], 'ZYZ');
    quat_fr_shift_rviz_l = eul2quat([pi/2, 0, 0], 'ZYZ');
    quat_hd_shift_rviz_l = eul2quat([pi/2, pi/2, pi], 'ZYZ');
    quat_up_shift_rviz_r = eul2quat([-pi/2, 0, 0], 'ZYZ');
    quat_fr_shift_rviz_r = eul2quat([-pi/2, 0, 0], 'ZYZ');
    quat_hd_shift_rviz_r = eul2quat([pi/2, -pi/2, -pi], 'ZYZ'); % hands' configurations are different
    l_up_quat_rviz = quatmultiply(quat_up_shift_rviz_l, l_up_quat);
    l_fr_quat_rviz = quatmultiply(quat_fr_shift_rviz_l, l_fr_quat);
    l_hd_quat_rviz = quatmultiply(quat_hd_shift_rviz_l, l_hd_quat);
    r_up_quat_rviz = quatmultiply(quat_up_shift_rviz_r, r_up_quat);
    r_fr_quat_rviz = quatmultiply(quat_fr_shift_rviz_r, r_fr_quat);
    r_hd_quat_rviz = quatmultiply(quat_hd_shift_rviz_r, r_hd_quat);
    % replace
    left_upperarm.quat(i, :) = l_up_quat_rviz; %l_up_quat;
    left_upperarm.pos(i, :) = l_up_pos;
    left_forearm.quat(i, :) = l_fr_quat_rviz; %l_fr_quat;
    left_forearm.pos(i, :) = l_fr_pos;    
    left_hand.quat(i, :) = l_hd_quat_rviz; %l_hd_quat;
    left_hand.pos(i, :) = l_hd_pos;  
    right_upperarm.quat(i, :) = r_up_quat_rviz; %r_up_quat;
    right_upperarm.pos(i, :) = r_up_pos;
    right_forearm.quat(i, :) = r_fr_quat_rviz; %r_fr_quat;
    right_forearm.pos(i, :) = r_fr_pos;    
    right_hand.quat(i, :) = r_hd_quat_rviz; %r_hd_quat;
    right_hand.pos(i, :) = r_hd_pos;  
end


%% Construct data array for wrist pos+ori(rotation matrices) and elbow pos
left_wrist_pos_ori_elbow_pos_path_1 = zeros(floor(size(data, 1)/10), 15); %zeros(floor(size(data, 1)/10), 15);
right_wrist_pos_ori_elbow_pos_path_1 = zeros(floor(size(data, 1)/10), 15); %zeros(floor(size(data, 1)/10), 15);
left_base = [-0.06, 0.235, 0.395]; % the pos of the robot's shoulder in the RViz world
right_base = [-0.06, -0.235, 0.395];
for i = 1 : floor(size(data, 1)/10)
    % left arm
    rotm_wrist = quat2rotm(left_hand.quat(i*10, :));
    % Since the position of the world frame is not the same as that of RViz(UR5) world, the position of left hand and left elbow should be converted with offset.   
    left_wrist_pos_ori_elbow_pos_path_1(i, 1:3) = left_hand.pos(i*10, :) - left_upperarm.pos(i*10, :) + left_base;
    left_wrist_pos_ori_elbow_pos_path_1(i, 4:12) = reshape(rotm_wrist', [1,9]);
    left_wrist_pos_ori_elbow_pos_path_1(i, 13:15) = left_forearm.pos(i*10, :) - left_upperarm.pos(i*10, :) + left_base;
    % right arm
    rotm_wrist = quat2rotm(right_hand.quat(i*10, :));
    % Since the position of the world frame is not the same as that of RViz(UR5) world, the position of left hand and left elbow should be converted with offset.   
    right_wrist_pos_ori_elbow_pos_path_1(i, 1:3) = right_hand.pos(i*10, :) - right_upperarm.pos(i*10, :) + right_base;
    right_wrist_pos_ori_elbow_pos_path_1(i, 4:12) = reshape(rotm_wrist', [1,9]);
    right_wrist_pos_ori_elbow_pos_path_1(i, 13:15) = right_forearm.pos(i*10, :) - right_upperarm.pos(i*10, :) + right_base;
end
combined_wrist_pos_ori_elbow_pos_path_1 = [left_wrist_pos_ori_elbow_pos_path_1, right_wrist_pos_ori_elbow_pos_path_1];


%% write data to .csv
wrist_pos_ori_elbow_pos_path_data=num2cell(combined_wrist_pos_ori_elbow_pos_path_1);
wrist_pos_ori_elbow_pos_path_data=[{'left_wrist_pos_x','left_wrist_pos_y','left_wrist_pos_z', ...
    'left_wrist_roi_1', 'left_wrist_roi_2','left_wrist_roi_3','left_wrist_roi_4','left_wrist_roi_5',...
    'left_wrist_roi_6','left_wrist_roi_7','left_wrist_roi_8','left_wrist_roi_9',...
    'left_hand_pos_x','left_hand_pos_y','left_hand_pos_z',...
    'right_wrist_pos_x','right_wrist_pos_y','right_wrist_pos_z', ...
    'right_wrist_roi_1','right_wrist_roi_2','right_wrist_roi_3','right_wrist_roi_4','right_wrist_roi_5',...
    'right_wrist_roi_6','right_wrist_roi_7', 'right_wrist_roi_8','right_wrist_roi_9',...
    'right_hand_pos_x','right_hand_pos_y','right_hand_pos_z',...
    };wrist_pos_ori_elbow_pos_path_data];
output=xlswrite('wrist_pos_ori_elbow_pos_path_data.xlsx',wrist_pos_ori_elbow_pos_path_data);

