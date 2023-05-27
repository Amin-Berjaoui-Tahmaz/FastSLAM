function [rho,alpha,beta] = ChangeVar(GoalPose,CurrentPose)

rho=sqrt((GoalPose(1)-CurrentPose(1))^2+(GoalPose(2)-CurrentPose(2))^2);

alpha=-CurrentPose(3)+ atan2((GoalPose(2)-CurrentPose(2)),(GoalPose(1)-CurrentPose(1)));

beta=-CurrentPose(3)-alpha;

end

