------------------------------------------------------------------------------ 
-- Handles Compatibility with modern versions of VREP
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
    -- Init Function, called once, initializes variables
    -- This is executed exactly once, the first time this script is executed
    if (simGetScriptExecutionCount() == 0) then
        -- Get a Reference to the robot object group
        BoddyRobot = simGetObjectAssociatedWithScript(sim_handle_self)
        -- Get a reference to the robot left motor by name
        leftMotor = simGetObjectHandle("leftMotor")
        -- Get a reference to the robot right motor by name
        rightMotor = simGetObjectHandle("rightMotor")
        -- Get a reference to the robot sensor by name
        FrontSensor = simGetObjectHandle("Front_Sensor")
        --Only 1 sensor used in this simulation, left and right are unused
        --LeftSensor=simGetObjectHandle("Left_Sensor")
        --RightSensor=simGetObjectHandle("Right_Sensor")
        -- Get a reference to the goal object (the plane) by name
        Goal = simGetObjectHandle("pad3")
        -- Set the time to reverse
        backUntilTime = -1
        -- Set the base speed
        speed = 2
        -- Set the base state
        state = 0
        -- Set the value of the light sensor
        LightSensor2 = 0
        -- Set the forward velocity
        forwardVel=1
        -- Set the base rotation
        rotation=-1
        -- Set the base red light detected
        Count_Blue = 0
        -- Set the base blue light detected
        Count_Red = 0
    end
    
    -- Calculates the angle and the distance between the robot and the goal
    Dist_Ang_Goal = function()
        -- Gets the positions of the object and the robot and forms a tuple
        local Pos_Goal = simGetObjectPosition(Goal, BoddyRobot)
        -- Uses the distance formula to calculate the distance between the 
        -- robot and the goal
        local Dist_Obj=(((Pos_Goal[1]^2) + (Pos_Goal[2]^2))^(1/2))
        -- Calculate the angle between the front vector of the robot and the 
        -- goal using arctan and position tuples
        local Angle_Obj=math.deg(math.atan2((Pos_Goal[2]),((Pos_Goal[1]))))
        -- returns a tuple of the distance to the goal and the angle 
        return Dist_Obj, Angle_Obj 
    end

    -- Go towards the target, rotating as necessary to adjust the angle. 
    -- If the target is within 0.2 units, halt forward motion
    Go_Target = function()
        local dist,angle = Dist_Ang_Goal()
        limit=5
        if dist<0.2 then
            simSetJointTargetVelocity(leftMotor, speed*0)
            simSetJointTargetVelocity(rightMotor, speed*0)
        -- If the angle between the robot forward vector is greater than 
        -- the limit, rotate towards the goal
        elseif angle >limit then
                --simAddBanner((Dist_Rob_Mov),5,sim_banner_bitmapfont+sim_banner_overlay,nil,BoddyRobot,red,yellow)
            simSetJointTargetVelocity(leftMotor, speed * ((120 - angle)/150))
            simSetJointTargetVelocity(rightMotor, speed * 2.0)
        -- If the angle between the robot forward vector is less than the limit
        -- rotate the robot towards the goal
        elseif angle<(-limit) then
            simSetJointTargetVelocity(leftMotor, speed * 2.0)
            simSetJointTargetVelocity(rightMotor, speed * ((120+angle)/150))
        -- Otherwise, move forward at the default speed
        else
            simSetJointTargetVelocity(leftMotor, 1 * speed)
            simSetJointTargetVelocity(rightMotor, 1 * speed)
        end
        end
    end

    -- If a wall is detected, go into a state moving along the wall using the 
    -- rotation to follow it.
    Follow_Wall = function()
        if (FrontAct > 0) then 
            simSetJointTargetVelocity(leftMotor, speed * 1)
            simSetJointTargetVelocity(rightMotor, speed * -0.1)
        else
            simSetJointTargetVelocity(leftMotor, 1 * speed)
            simSetJointTargetVelocity(rightMotor, 2.0 * speed)
        end
    end


    -- Execution Loop:
    -- Handle any child scripts
    simHandleChildScripts(sim_call_type)
    -- Get input from the front touch sensor
    FrontAct = simReadProximitySensor(FrontSensor)
    -- Calculate the distance and angle to the goal
    dist, angle = Dist_Ang_Goal()

    -- If in the forward state, rotate towards the goal, or move towards it
    if state == 0 then
        -- If a wall is detected, change to the wall following state
        if (FrontAct > 0 ) then
            state = 1
        else
            Go_Target()
        end 
    end

    -- If a wall is detected, follow the wall
    if state == 1 then
        if ( (FrontAct > 0)) then
            Follow_Wall()
        elseif ((angle<0)  and (angle> -90)) then
            Go_Target()
        else
            Follow_Wall()
        end
    end
end 