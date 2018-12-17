
function robot = mSaveData(obj,robot)

ID = robot.pID;

try
    robot.pPos.Xd = obj.pMSG.getFrom{ID}(2+(1:12));
    robot.pPos.X = obj.pMSG.getFrom{ID}(14+(1:12));
    robot.pPos.Ur = obj.pMSG.getFrom{ID}(26+(1:2));
    robot.pPos.U = obj.pMSG.getFrom{ID}(28+(1:2));
catch
    msg = sprintf('Dados do Robô %d não disponíveis!',ID);
    disp(msg);
end