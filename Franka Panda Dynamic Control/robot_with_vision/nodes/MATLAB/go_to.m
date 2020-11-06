function go_to(trajectory, joint_update_delay)
%iterates through a list of points in a trajectory and creates a joint
%message for each one through the publish_joint function
% # takes a trajectory, which is an array of JointState values (pos, vel, effort)
% # iterate through each trajectory point and set each joint variable
    for i=1:1:size(trajectory)
        publish_joint(trajectory(0,i), trajectory(1,i), trajectory(2,i))
        rospy.sleep(joint_update_delay)
    end
end

