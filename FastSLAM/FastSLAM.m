function [pose,map] = FastSLAM(initialpose)
initialpose=[0 0 0];
mycommand=rospublisher('/husky_velocity_controller/cmd_vel','geometry_msgs/Twist');
msg=rosmessage(mycommand);

robotposeodom=rossubscriber('/husky_velocity_controller/odom');
robotlidar=rossubscriber('/scan');

pause(0.2);
lidMinAng = robotlidar.LatestMessage.AngleMin;
lidMaxAng = robotlidar.LatestMessage.AngleMax;
lidarangles = lidMinAng:(lidMaxAng-lidMinAng)/719:lidMaxAng;

set=zeros(40,3);
p=zeros(1,40);

% A=imread('square_room.jpg');
% map_bw = 100*(1-im2bw(A,0.9))
% 
% possmap=ones(200,200,40)*100;
% 
% for x=1:40
%     possmap(:,:,x)=map_bw;
% end

possmap=ones(200,200,40)*100;

alphas=[0.7 0.7 2.5 2.5];
Z=[0.95,0.01,0.02,0.02];
lambda=0.3;
lidMax=500;
lidSTD=1;
zmax=30;

%% initialize 40x3 matrix from initial known pose

for i=1:40
    set(x,:) = [0 0 0];
end

oldodom = initialpose;

for i =1:100
    %% sample motion
    for j=1:20     
        mypose=robotposeodom.LatestMessage;

        OdomPose= mypose.Pose.Pose.Position;
        OdomX=OdomPose.X;
        OdomY=OdomPose.Y;

        OdomQuat=[mypose.Pose.Pose.Orientation.X,mypose.Pose.Pose.Orientation.Y,mypose.Pose.Pose.Orientation.Z,mypose.Pose.Pose.Orientation.W];
        eulZYX=quat2eul(OdomQuat);
        OdomOrientation=eulZYX(3);

        newodom=[OdomX, OdomY, OdomOrientation];
%         
%         if norm(newodom-oldodom)==0
%             break;
%         end
        set = Sample_motion_model(alphas,oldodom,newodom,set);
        oldodom=newodom;
        pause(1);
    end

    %% obtain lidar readings
    lidRanges = robotlidar.LatestMessage.Ranges;
    
    %% measurement model and mapping
    for k=1:40
        
        for j=1:10
             for x=1:200
                for y=1:200
                    ObjectPose=[(x-100)*0.1-0.05,(y-100)*0.1-0.05];
                    m_point = possmap(x,y,k);
                    possmap(x,y,k) = possmap(x,y,k)+inverseRangeSensorModelNew(CurrentPose,ObjectPose,lidarangles,lidRanges,zmax,m_point);
                end
             end
             imshow(possmap(:,:,k),[0,100]);
        end
        
        p(k)=measurement_model_map(set(k,:),Z,lambda,lidRanges,lidarangles,lidMax,lidSTD,possmap(:,:,k))
    end

    %% resampling
    X=1:40;
    newset=zeros(40,3);
    newsample=randsample(X,40,true,p);
    
    for k=1:40
        newset(k,:)=set(newsample(k),:);
    end
    
    [~,max_index] = max(p);
    
    pose=set(max_index,:);
    map=possmap(:,:,max_index);
    
    imshow(map,[0,100])

end
end

% sample motion model functions even when you are not doing slam. so when you
% are moving.