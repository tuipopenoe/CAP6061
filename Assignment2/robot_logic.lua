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
        FrontSensor=simGetObjectHandle("Front_Sensor")
        --LeftSensor=simGetObjectHandle("Left_Sensor")
        --RightSensor=simGetObjectHandle("Right_Sensor")
        Goal=simGetObjectHandle("pad3")
        
        backUntilTime=-1 
        
        speed=2
        state=0
        LightSensor2=0
        forwardVel=1
        rotation=-1
        Count_Blue=0
        Count_Red=0
    
    end
    
    
    
    Dist_Ang_Goal=function()
        local Pos_Goal=simGetObjectPosition(Goal,BoddyRobot)
        local Dist_Obj=(((Pos_Goal[1]^2)+(Pos_Goal[2]^2)    )^(1/2))
        local Angle_Obj=math.deg(math.atan2((Pos_Goal[2]),((Pos_Goal[1]))))
        return Dist_Obj,Angle_Obj 
    end
    
    Go_Target=function()
    local dist,angle=Dist_Ang_Goal()
    limit=5
    if dist<0.2 then
                simSetJointTargetVelocity(leftMotor,speed*0)
                simSetJointTargetVelocity(rightMotor,speed*0)
    
    else
    if angle >limit then
        --simAddBanner((Dist_Rob_Mov),5,sim_banner_bitmapfont+sim_banner_overlay,nil,BoddyRobot,red,yellow)
                simSetJointTargetVelocity(leftMotor,speed*((120-angle)/150))
                simSetJointTargetVelocity(rightMotor,speed*2.0)
    
            elseif angle<(-limit) then
                simSetJointTargetVelocity(leftMotor,speed*2.0)
                simSetJointTargetVelocity(rightMotor,speed*((120+angle)/150))
            else
                simSetJointTargetVelocity(leftMotor,1*speed)
                simSetJointTargetVelocity(rightMotor,1*speed)
            end
    end
    end
    
    Follow_Wall=function()
    
    if (FrontAct >0) then 
                simSetJointTargetVelocity(leftMotor,speed*1)
                simSetJointTargetVelocity(rightMotor,speed*-0.1)
    
            else
        
                simSetJointTargetVelocity(leftMotor,1*speed)
                simSetJointTargetVelocity(rightMotor,2.0*speed)
            end
    end
    
    
    
    
    
    -- *******************************************************************************
    -- ***** The second part, this is the section where you can do changes of code, 
    -- ***** to improve the performance of robot
    -- ******************************************************************************* 
    
    simHandleChildScripts(sim_call_type)
    -- read the Touch sensor
    FrontAct=simReadProximitySensor(FrontSensor)
    
    dist, angle=Dist_Ang_Goal()
    
    
    if state == 0 then
        if (FrontAct >0 ) then
            state=1
        else
            Go_Target()
        end 
    end
    
    if state == 1 then
        if ( (FrontAct>0)) then
            Follow_Wall()
        elseif ((angle<0)  and (angle> -90)) then
            Go_Target()
        else
            Follow_Wall()
        end
    end
    
    
 
 
------------------------------------------------------------------------------ 
-- Following few lines automatically added by V-REP to guarantee compatibility 
-- with V-REP 3.1.3 and later: 
end 
------------------------------------------------------------------------------ 
