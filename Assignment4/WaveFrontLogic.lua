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
        Front=simGetObjectHandle("LaserScanner_2D_F")
        Back=simGetObjectHandle("LaserScanner_2D_B")
    
        speed=2
        state=0
        Destiny={}
        -- OBSTACLES POSITION
        PobsX={2, 2, 2, 4, 5, 5, 6, 7, 8, 9, 10, 11, 11}
        PobsY={3, 7,11, 5, 8,10, 3, 6, 9, 4,  7,  2 ,11 }
        
        --  EXAMPLE PATH
        pathx={7, 7, 7, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7}
        pathy={2 ,3, 4, 4, 5, 6, 7, 7, 8, 9, 10, 11}
        Count=1
    
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
                state=state+1
        else
            if angle >limit then
                simSetJointTargetVelocity(leftMotor,speed*((60-angle)/300))
                simSetJointTargetVelocity(rightMotor,speed*1.0)
    
                elseif angle<(-limit) then
                    simSetJointTargetVelocity(leftMotor,speed*1.0)
                    simSetJointTargetVelocity(rightMotor,speed*((60+angle)/300))
                else
                    simSetJointTargetVelocity(leftMotor,5*speed*dist)
                    simSetJointTargetVelocity(rightMotor,5*speed*dist)
            end
        end
    end
    
    
    -- ***************************************************
    -- ***** The second part, this is the section where you can do changes of code, 
    -- ***** to improve the performance of robot
    -- ******************************************************************************* 
    
    
    -- It is necessary to develop WAVE FRONT ALGORITHM and generate
    -- pathx and pathy
    
    
    simHandleChildScripts(sim_call_type)
    Pos_Robot=simGetObjectPosition(BoddyRobot,Zero_Point)
            
    if state == 0 then
        Destiny[1]=pathx[Count]
        Destiny[2]=pathy[Count]
        
        Go_Point(Destiny)
    end
    
    if state==1 then
        state=0
        Count=Count+1
            --if Count==7 then
                --Count=1
            --end
    
    end
 
 
------------------------------------------------------------------------------ 
-- Following few lines automatically added by V-REP to guarantee compatibility 
-- with V-REP 3.1.3 and later: 
end 
------------------------------------------------------------------------------ 
