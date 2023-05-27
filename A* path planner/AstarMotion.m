rosshutdown
clc
clear all
close all
setenv('ROS_MASTER_URI','http://robotvirtualbox:11311');
rosinit
% using a predetermined map using the mapping script.
% The map is not fully complete however, it shows the main feature we wish
% to test the A* algorithm on and that is the concrete slab.
A = imread('playpen_map.png');
map= im2bw(A,0.9);
% map=1-map;

robotposeodom=rossubscriber('/husky_velocity_controller/odom');
mycommand=rospublisher('/husky_velocity_controller/cmd_vel','geometry_msgs/Twist');
msg=rosmessage(mycommand);

% Setting the initial point and a goal position that cannot be achieved by
% going in a straight line due to the concrete slab
startPose=[0 0];
goalPose=[5.5,-2];
pathgen = AstarPathPlanning(map, startPose, goalPose)

for i=1:length(pathgen)
   map(pathgen(i,1),pathgen(i,2))=1;% drawing the represented path on the map as a white line. 
%    For our specific start and goal point, the obtained path is included
%    in our submission files as "Astarpath.png" . please note that you have
%    to run the entire code for it to work correctly.
end

pathgen=(pathgen/10)-10;%coordinates from matrix cell coordinates to meters in x,y plane

imshow(map,[0,1]);

l=floor(length(pathgen)/10);
% Here, for the motion control, we divided the path points, that are about
% 80 points, into sections to speed up the process. dividing into ten
% sections,so instead of the robot having to pass over all the points, it
% was only have to pass over the sectioned points
%% motion control
for i=1:l
rho=1;
while rho>0.2
mypose=robotposeodom.LatestMessage;
OdomPose= mypose.Pose.Pose.Position;
OdomX=OdomPose.X;
OdomY=OdomPose.Y;

OdomQuat=[mypose.Pose.Pose.Orientation.X,mypose.Pose.Pose.Orientation.Y,mypose.Pose.Pose.Orientation.Z,mypose.Pose.Pose.Orientation.W];
eulZYX=quat2eul(OdomQuat);
OdomOrientation=eulZYX(3);

newodom=[OdomX, OdomY,OdomOrientation];


[rho,alpha,beta] = ChangeVar([pathgen(10*i,:),newodom(3)],newodom);
% a simple motion control algorithm that turns the robot angle until it
% faces the next point and then speeds up linearly towards that point. This
% is done until rho is within 0.2 meter range of the point which is
% reasonable given the large clearence we gave the robot in the A*
% algorithm. The motion is repeated for each sectioned point until the end
% point is reahed
if abs(alpha)>0.3
    if alpha < 0
            msg.Angular.Z=-0.5;
        else
            msg.Angular.Z= 0.5;
    end
else
    v=rho;
    if v<0.8
    msg.Linear.X= v;
    else
    msg.Linear.X=0.2;
    end
end

send(mycommand,msg);

pause(1);
end
end

