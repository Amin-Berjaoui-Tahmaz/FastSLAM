function [l] = inverseRangeSensorModelNew(CurrentPose,ObjectPose,lidarangles,lidRanges,zmax,m_point)

%zmax=30;
alpha=0.5;
beta=3*pi/180;

r=sqrt((ObjectPose(1)-CurrentPose(1))^2+(ObjectPose(2)-CurrentPose(2))^2);
phi=atan2(ObjectPose(2)-CurrentPose(2),ObjectPose(1)-CurrentPose(1))-CurrentPose(3);
[~, k]=min(abs(phi-lidarangles)); 

if r>min(zmax,lidRanges(k)+alpha/2) || abs(phi-lidarangles(k))>beta/2
    l=0;  

elseif lidRanges(k)<zmax && abs(r-lidRanges(k))<alpha/2 && m_point>=10
    l=-10;
    
elseif r<=lidRanges(k) && m_point<=90
    l=+5;
else
    l=0;
end