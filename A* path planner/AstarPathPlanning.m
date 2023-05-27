function [path] = AstarPathPlanning(map, startPose, goalPose)
% A* path planning function

% This first step involves transforming the poses from meter coordinates to
% matrix cells or pixels (where each cell corresponds to 0.1 meter)
goalPose=floor((goalPose+10)*10);
startPose=floor((startPose+10)*10);

% initialize the current point or cell in the search
currentPose=startPose;


n=length(map);
hcost= 10000*ones(n,n);%initialize up the heuristic values of every cell 

pathgen=[];
hmat=[];
closed=[];
while (currentPose(1) ~= goalPose(1))||(currentPose(2) ~= goalPose(2))%when the current cell coordinates is not equal to the goal cell coordinates the algorithm will keep on looping
    hmat=[];
%     only passing over the points adjacent to the current cell in each
%     loop (8 adjacent points and the 9th point is the current cell itself
for i=-1:1
   for j=-1:1
       
       if map(currentPose(1)+i,currentPose(2)+j)==0
           %if the map shows the adjacent cell to be empty, we calculate
           %the cell's h cost that is estimated as the straight distance 
           %between the goal and tested cell.

            hcost(currentPose(1)+i,currentPose(2)+j)=sqrt((currentPose(1)+i-goalPose(1))^2+(currentPose(2)+j-goalPose(2))^2);
       else
           % if the cell is occupied, then set the heuristic value or
           % hcost to infinity so the cell will not be chosen as the next
           % cell
            hcost(currentPose(1)+i,currentPose(2)+j)=inf;
       end
       f=0;
       s=size(closed);
       for k=1:s(1)
           if [currentPose(1)+i,currentPose(2)+j]==closed(k,:)
              f=f+1; 
            % a loop to check if the tested cell is already labeled as
            % closed
           end
       end
       
       for k=-10:10
           for l=-10:10
                if map(currentPose(1)+i+k,currentPose(2)+j+l)==1
                    hcost(currentPose(1)+i,currentPose(2)+j)=inf;
                    %a loop to check the 10 adjacent cells in each
                    %direction of the tested cell. since each cell is 0.1
                    %meters, then this tests for 1 meter in each direction
                    %of a given point and if a cell is occupied within the
                    %1 meter radius, then the tested cell's hcost is set to
                    %infinity. This step is to account for the width of the
                    %husky robot(0.5meter) and an additional clearance of 0.5
                    %meters for the sake of avoiding a collision.
                end
           end
       end
       if (i~=0 || j~=0) && f~=1
         %only for the adjacent cells that have not been already labeled as
         %closed will be added to the heuristic matrix
         hmat=[hmat;hcost(currentPose(1)+i,currentPose(2)+j),currentPose(1)+i,currentPose(2)+j];
       end
       
   end
end

[hmin,min_index]=min(hmat(:,1));%the adjacent cell with the minimum h cost is chosen to be add to the path
s=size(pathgen);
repeat=0;
for i=1:s(1)
    if hmat(min_index,2:3)==pathgen(i,:)
        repeat=repeat+1;
        % this loop checks if minimum hcost cell has already been passed
        % over previously and has been added to the path. if so, then the
        % algorithm might be stuck over this region and so the repeated
        % cell must be labeled as closed since it did not give the right
        % path despite having the lowest hcost
    end
end
if repeat>=1
    closed=[closed;hmat(min_index,2:3)];
    %the repeated cell being labeled as closed
end
pathgen=[pathgen;hmat(min_index,2:3)]
currentPose=hmat(min_index,2:3);
% The unrepeated, unclosed cell with clearance is added to the path and is
% set as the next iteration's current pose cell.
end
path=pathgen;%the path is outputed
end