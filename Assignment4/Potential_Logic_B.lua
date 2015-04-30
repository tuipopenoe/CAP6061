------------------------------------------------------------------------------ 
-- Following few lines automatically added by V-REP to guarantee compatibility 
-- with V-REP 3.1.3 and later: 
if (sim_call_type==sim_childscriptcall_initialization) then 
    simSetScriptAttribute(sim_handle_self,sim_childscriptattribute_automaticcascadingcalls,false) 
end 
if (sim_call_type==sim_childscriptcall_cleanup) then 
 
end 
if (sim_call_type==sim_childscriptcall_sensing) then 
    simHandleChildScripts(sim_call_type) 
end 
if (sim_call_type==sim_childscriptcall_actuation) then 
    if not firstTimeHere93846738 then 
        firstTimeHere93846738=0 
    end 
    simSetScriptAttribute(sim_handle_self,sim_scriptattribute_executioncount,firstTimeHere93846738) 
    firstTimeHere93846738=firstTimeHere93846738+1 
 
------------------------------------------------------------------------------ 
 
 
-- ****************************************************************************
    -- ***** The first part  is the setup of system, initialize sensors and varibles.
    -- ***** You don't need to change something of this part
    -- ***************************************************************************** 
    if (simGetScriptExecutionCount()==0) then
        -- This is executed exactly once, the first time this script is executed
        BoddyRobot=simGetObjectAssociatedWithScript(sim_handle_self)
        leftMotor=simGetObjectHandle("leftMotor")
        rightMotor=simGetObjectHandle("rightMotor")
        Goal=simGetObjectHandle("pad3")
        Zero_Point=simGetObjectHandle("Zero_Point") 
        backUntilTime=-1 
        Front=simGetObjectHandle("LaserScanner_2D_F")
        Back=simGetObjectHandle("LaserScanner_2D_B")
    
        speed=6
        PobsX={1.5, 1.5, 1.5, 3.5, 4.5, 4.5, 5.5, 6.5, 7.5, 8.5, 9.5, 10.5, 10.5}
        PobsY={2.5, 6.5,10.5, 4.5, 7.5, 9.5, 2.5, 5.5, 8.5, 3.5, 6.5, 1.5 , 10.5 }
        Destiny={}
        Limit=2
        Ka=1.2
        Kr=1.0
    end
    
    
    Dist_Ang_Rob_Point=function(Pos_Obj)
        local Pos1=simGetObjectPosition(Front,Zero_Point)
        local Pos2=simGetObjectPosition(Back,Zero_Point)
    
        local Dis_X=Pos1[1]-Pos_Obj[1]
        local Dis_Y=Pos1[2]-Pos_Obj[2]
        local Dist_Obj=(((Dis_Y^2)+(Dis_X^2)    )^(1/2))
    
        local Angle_Rob=(math.atan2((Pos1[2]-Pos2[2]),((Pos1[1]-Pos2[1]))))
        if Angle_Rob<0 then Angle_Rob=(2*3.1415)+Angle_Rob  end
        local Dis_Obj_X=Pos_Obj[1]-Pos1[1]
        local Dis_Obj_Y=Pos_Obj[2]-Pos1[2]
    
        local Pos_Obj_Rob_Y= (math.cos(Angle_Rob)*Dis_Obj_X)+ (math.sin(Angle_Rob)*Dis_Obj_Y)
        local Pos_Obj_Rob_X= (math.sin(Angle_Rob)*Dis_Obj_X)- (math.cos(Angle_Rob)*Dis_Obj_Y)
        local Angle_Obj=(math.deg(math.atan2(Pos_Obj_Rob_Y,Pos_Obj_Rob_X)))-90
        if Angle_Obj<(-180) then Angle_Obj=270+Angle_Obj+90  end
        return Dist_Obj,Angle_Obj 
    end
    
    
    
    Go_Point=function(Pos_Point)
    local dist,angle=Dist_Ang_Rob_Point(Pos_Point)
    limit=5
    if dist<0.2 then
                simSetJointTargetVelocity(leftMotor,speed*0)
                simSetJointTargetVelocity(rightMotor,speed*0)
    
    else
    if angle >limit then
        --simAddBanner((Dist_Rob_Mov),5,sim_banner_bitmapfont+sim_banner_overlay,nil,BoddyRobot,red,yellow)
                simSetJointTargetVelocity(leftMotor,speed*((90-angle)/90)*2)
                simSetJointTargetVelocity(rightMotor,speed*2.0)
    
            elseif angle<(-limit) then
                simSetJointTargetVelocity(leftMotor,speed*2.0)
                simSetJointTargetVelocity(rightMotor,speed*((90+angle)/90)*2)
            else
                simSetJointTargetVelocity(leftMotor,3*speed)
                simSetJointTargetVelocity(rightMotor,3*speed)
            end
    end
    end
    
    
    
    
    -- ****************************end
    --***************************************************
    -- ***** The second part, this is the section where you can do changes of code, 
    -- ***** to improve the performance of robot
    -- ******************************************************************************* 
    
    simHandleChildScripts(sim_call_type)
    PRobot=simGetObjectPosition(BoddyRobot,Zero_Point)
    PGoal=simGetObjectPosition(Goal,Zero_Point)
    
    DistGoal,angle=Dist_Ang_Rob_Point(PGoal)
    MinDist1=100
    MinDist2=100
    MinNode1=1
    MinNode2=2
        for i=1,13 do
            Dist1=((((PRobot[1]-PobsX[i])^2)+((PRobot[2]-PobsY[i])^2))^(1/2))
            if Dist1<MinDist2 then
                MinDist2=Dist1
                MinNode2 = i
            end
            if MinDist2 < MinDist1 then
                temp = MinDist1
                MinDist1 = MinDist2
                MinDist2 = temp
                temp = MinNode1
                MinNode1 = MinNode2
                MinNode2 = temp
            end
        end
        VrepX=(-1)*((PobsX[MinNode1]-PRobot[1])/MinDist1)+((PobsX[MinNode2]-PRobot[1])/MinDist1)
        VrepY=(-1)*((PobsY[MinNode1]-PRobot[2])/MinDist1)+((PobsY[MinNode2]-PRobot[2])/MinDist2) 
            

        if MinDist1>Limit then
            VrepX=0
            VrepY=0
            MinDist1=Limit
        end
        if MinDist2>Limit then
            VrepX=0
            VrepY=0
            MinDist2=Limit
        end
    
        VattX=((PGoal[1]-PRobot[1])/DistGoal)
        VattY=((PGoal[2]-PRobot[2])/DistGoal)
    
    
        VmovX=Ka*VattX+Kr*VrepX
        VmovY=Ka*VattY+Kr*VrepY
    
        Destiny[1]=VmovX*1+PRobot[1]
        Destiny[2]=VmovY*1+PRobot[2]
        Go_Point(Destiny)
    
        --simAddBanner((YFinal),5,sim_banner_bitmapfont+sim_banner_overlay,nil,BoddyRobot,red,yellow)
------------------------------------------------------------------------------ 
-- Following few lines automatically added by V-REP to guarantee compatibility 
-- with V-REP 3.1.3 and later: 
end 
------------------------------------------------------------------------------ 
