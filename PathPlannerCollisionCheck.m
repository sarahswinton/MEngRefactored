function collision = PathPlannerCollisionCheck(node2, node1, obsOneX, obsOneY, obsTwoX, obsTwoY, obsThreeX, obsThreeY, obsFourX, obsFourY)

nodeOne = node1; 
nodeTwo = node2;
for i = 1:1:4
    if i < 4
        % Check obs 1 
        dt1Obs1 = det([1,1,1;nodeOne(1),nodeTwo(1),obsOneX(i);nodeOne(2),nodeTwo(2),obsOneY(i)])*det([1,1,1;nodeOne(1),nodeTwo(1),obsOneX(i+1);nodeOne(2),nodeTwo(2),obsOneY(i+1)]);
        dt2Obs1 = det([1,1,1;nodeOne(1),obsOneX(i),obsOneX(i+1);nodeOne(2),obsOneY(i),obsOneY(i+1)])*det([1,1,1;nodeTwo(1),obsOneX(i),obsOneX(i+1);nodeTwo(2),obsOneY(i),obsOneY(i+1)]);
        % Check obs 2
        dt1Obs2 = det([1,1,1;nodeOne(1),nodeTwo(1),obsTwoX(i);nodeOne(2),nodeTwo(2),obsTwoY(i)])*det([1,1,1;nodeOne(1),nodeTwo(1),obsTwoX(i+1);nodeOne(2),nodeTwo(2),obsTwoY(i+1)]);
        dt2Obs2 = det([1,1,1;nodeOne(1),obsTwoX(i),obsTwoX(i+1);nodeOne(2),obsTwoY(i),obsTwoY(i+1)])*det([1,1,1;nodeTwo(1),obsTwoX(i),obsTwoX(i+1);nodeTwo(2),obsTwoY(i),obsTwoY(i+1)]);
        % Check obs 3
        dt1Obs3 = det([1,1,1;nodeOne(1),nodeTwo(1),obsThreeX(i);nodeOne(2),nodeTwo(2),obsThreeY(i)])*det([1,1,1;nodeOne(1),nodeTwo(1),obsThreeX(i+1);nodeOne(2),nodeTwo(2),obsThreeY(i+1)]);
        dt2Obs3 = det([1,1,1;nodeOne(1),obsThreeX(i),obsThreeX(i+1);nodeOne(2),obsThreeY(i),obsThreeY(i+1)])*det([1,1,1;nodeTwo(1),obsThreeX(i),obsThreeX(i+1);nodeTwo(2),obsThreeY(i),obsThreeY(i+1)]);
        % Check obs 4 
        dt1Obs4 = det([1,1,1;nodeOne(1),nodeTwo(1),obsFourX(i);nodeOne(2),nodeTwo(2),obsFourY(i)])*det([1,1,1;nodeOne(1),nodeTwo(1),obsFourX(i+1);nodeOne(2),nodeTwo(2),obsFourY(i+1)]);
        dt2Obs4 = det([1,1,1;nodeOne(1),obsFourX(i),obsFourX(i+1);nodeOne(2),obsFourY(i),obsFourY(i+1)])*det([1,1,1;nodeTwo(1),obsFourX(i),obsFourX(i+1);nodeTwo(2),obsFourY(i),obsFourY(i+1)]);
    else 
        % Check obs 1 
        dt1Obs1 = det([1,1,1;nodeOne(1),nodeTwo(1),obsOneX(i);nodeOne(2),nodeTwo(2),obsOneY(i)])*det([1,1,1;nodeOne(1),nodeTwo(1),obsOneX(1);nodeOne(2),nodeTwo(2),obsOneY(1)]);
        dt2Obs1 = det([1,1,1;nodeOne(1),obsOneX(i),obsOneX(1);nodeOne(2),obsOneY(i),obsOneY(1)])*det([1,1,1;nodeTwo(1),obsOneX(i),obsOneX(1);nodeTwo(2),obsOneY(i),obsOneY(1)]);
        % Check obs 2 
        dt1Obs2 = det([1,1,1;nodeOne(1),nodeTwo(1),obsTwoX(i);nodeOne(2),nodeTwo(2),obsTwoY(i)])*det([1,1,1;nodeOne(1),nodeTwo(1),obsTwoX(1);nodeOne(2),nodeTwo(2),obsTwoY(1)]);
        dt2Obs2 = det([1,1,1;nodeOne(1),obsTwoX(i),obsTwoX(1);nodeOne(2),obsTwoY(i),obsTwoY(1)])*det([1,1,1;nodeTwo(1),obsTwoX(i),obsTwoX(1);nodeTwo(2),obsTwoY(i),obsTwoY(1)]);
        % Check obs 3 
        dt1Obs3 = det([1,1,1;nodeOne(1),nodeTwo(1),obsThreeX(i);nodeOne(2),nodeTwo(2),obsThreeY(i)])*det([1,1,1;nodeOne(1),nodeTwo(1),obsThreeX(1);nodeOne(2),nodeTwo(2),obsThreeY(1)]);
        dt2Obs3 = det([1,1,1;nodeOne(1),obsThreeX(i),obsThreeX(1);nodeOne(2),obsThreeY(i),obsThreeY(1)])*det([1,1,1;nodeTwo(1),obsThreeX(i),obsThreeX(1);nodeTwo(2),obsThreeY(i),obsThreeY(1)]);
        % Check obs 4
        dt1Obs4 = det([1,1,1;nodeOne(1),nodeTwo(1),obsFourX(i);nodeOne(2),nodeTwo(2),obsFourY(i)])*det([1,1,1;nodeOne(1),nodeTwo(1),obsFourX(1);nodeOne(2),nodeTwo(2),obsFourY(1)]);
        dt2Obs4 = det([1,1,1;nodeOne(1),obsFourX(i),obsFourX(1);nodeOne(2),obsFourY(i),obsFourY(1)])*det([1,1,1;nodeTwo(1),obsFourX(i),obsFourX(1);nodeTwo(2),obsFourY(i),obsFourY(1)]);
    end 
    if(dt1Obs1<=0 && dt2Obs1<=0)
        collision=0;         %If lines intersect in first obstacle
        break
    elseif (dt1Obs2<=0 && dt2Obs2<=0)
        collision=0;         %If lines intersect in second obstacle
        break
    elseif (dt1Obs3<=0 && dt2Obs3<=0)
        collision=0;         %If lines intersect in third obstacle
        break
    elseif (dt1Obs4<=0 && dt2Obs4<=0)
        collision=0;         %If lines intersect in four obstacle
        break
    else
        collision=1;
    end
    
end
% Loop to check for collisions with each segment of RHS trapezoid
% for i = 1:1:4
%     if i < 4
%         dt1 = det([1,1,1;nodeOne(1),nodeTwo(1),obsTwoX(i);nodeOne(2),nodeTwo(2),obsTwoY(i)])*det([1,1,1;nodeOne(1),nodeTwo(1),obsTwoX(i+1);nodeOne(2),nodeTwo(2),obsTwoY(i+1)]);
%         dt2 = det([1,1,1;nodeOne(1),obsTwoX(i),obsTwoX(i+1);nodeOne(2),obsTwoY(i),obsTwoY(i+1)])*det([1,1,1;nodeTwo(1),obsTwoX(i),obsTwoX(i+1);nodeTwo(2),obsTwoY(i),obsTwoY(i+1)]);
%     else 
%         dt1 = det([1,1,1;nodeOne(1),nodeTwo(1),obsTwoX(i);nodeOne(2),nodeTwo(2),obsTwoY(i)])*det([1,1,1;nodeOne(1),nodeTwo(1),obsTwoX(1);nodeOne(2),nodeTwo(2),obsTwoY(1)]);
%         dt2 = det([1,1,1;nodeOne(1),obsTwoX(i),obsTwoX(1);nodeOne(2),obsTwoY(i),obsTwoY(1)])*det([1,1,1;nodeTwo(1),obsTwoX(i),obsTwoX(1);nodeTwo(2),obsTwoY(i),obsTwoY(1)]);
%     end 
%     if(dt1<=0 && dt2<=0)
%         collision=0;         %If lines intersect
%         break
%     else
%         collision=1;
%     end
%     
% end
