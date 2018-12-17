function rSetPose(robot,Xo)

robot.pPos.Xs([1 2 3 6]) = Xo;

robot.pPos.Xc([1 2 3]) = robot.pPos.Xs([1 2 3]) - ...
    [robot.pPar.a*cos(robot.pPos.Xs(6)); robot.pPar.a*sin(robot.pPos.Xs(6)); 0];
robot.pPos.Xc([4 5 6]) = robot.pPos.Xs([4 5 6]);

robot.pPos.X  = robot.pPos.Xs;
robot.pPos.Xd = robot.pPos.X;

if robot.pFlag.Connected
    arrobot_setpose(robot.pPos.Xc(1)*1000,robot.pPos.Xc(2)*1000,robot.pPos.Xc(6)*180/pi);
end
