function publish_joint(joint_state_pub, joints_pos, joints_vel, joints_eff)
% RBE 501 Panda Arm Dynamical Controller
% Publishes JointState message on joint_states topic for a single pose

    % move this outside of the function and pass this in for efficiency
    joint_state_pub = rospublisher('/joint_states', 'sensor_msgs/JointState');

    name = {'panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4',...
        'panda_joint5', 'panda_joint6', 'panda_joint7'};
    position = zeros(size(joints_pos));
    velocity = zeros(size(joints_pos));
    effort = zeros(size(joints_pos));

    for i=1:1:size(joints_pos)
        position(i) = (joints_pos(i));
        velocity(i) = (joints_vel(i));
        effort(i) = (joints_eff(i));
    end
    j_msg = rosmessage('sensor_msgs/JointState');
    j_msg.Name = name;
    j_msg.Position = position;
    j_msg.Velocity = velocity;
    j_msg.Effort = effort;
    send(joint_state_pub,j_msg);

end

