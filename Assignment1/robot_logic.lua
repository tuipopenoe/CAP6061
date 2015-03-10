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
        firstTimeHere93846738 = 0 
    end 
    simSetScriptAttribute(sim_handle_self,sim_scriptattribute_executioncount,firstTimeHere93846738) 
    firstTimeHere93846738=firstTimeHere93846738 + 1 
 
------------------------------------------------------------------------------ 
 
 
-- ****************************************************************************
    -- ***** The first part  is the setup of system, initialize sensors and varibles.
    -- ***** You don't need to change something of this part
    -- ***************************************************************************** 
    if (simGetScriptExecutionCount()== 0) then
        -- This is executed exactly once, the first time this script is executed
        BoddyRob = simGetObjectAssociatedWithScript(sim_handle_self)
        leftMotor = simGetObjectHandle("leftMotor")
        rightMotor = simGetObjectHandle("rightMotor")
        TouchSensor = simGetObjectHandle("Touch_Sensor")
        FloorSensor = simGetObjectHandle("Floor_Sensor")
        LightSensor = simGetObjectHandle("Light_Sensor")
        reverse_time = -1 
        speed = 1
    end

    -- Handle Child Scripts of BoddyRob
    simHandleChildScripts(sim_call_type)
    -- read the Touch sensor
    bumper_detected = simReadProximitySensor(TouchSensor)
    if (bumper_detected > 0) then 
    --  if the touch sensor was activated, provides the robot with 5 seconds to act before recheck the touch sensor 
        reverse_time = simGetSimulationTime() + 1
        print("Bumper Detected: Reversing")
    end

    -- Read the light sensor
    square_detected = {false}
        result, data=simReadVisionSensor(FloorSensor)
        -- If the result is not an error (-1) check the data.
        if (result>=0) then
        
        -- data[1] is the max intensity of the light
        -- 0.3 is the threshold to define if there are a black square below of robot
            square_detected=( data[1] < 0.3) 
        end
    
    -- If light sensor detect black square below of robot, then it is assigned zero to speed
    if square_detected then
        speed= 0
        print("Square Detected: Shutting Down")
        break
    else 
        speed = 1
    end

    if (reverse_time < simGetSimulationTime()) then
        -- When in forward mode, we simply move forward at the desired speed
        simSetJointTargetVelocity(leftMotor,speed*6.0)
        simSetJointTargetVelocity(rightMotor,speed*3.9)
    else
        -- When in backward mode, we simply move backward at the desired speed
        simSetJointTargetVelocity(leftMotor,-speed*5.1)
        simSetJointTargetVelocity(rightMotor,speed*3.3)
    end
 
 
------------------------------------------------------------------------------ 
-- Following few lines automatically added by V-REP to guarantee compatibility 
-- with V-REP 3.1.3 and later: 
end 
------------------------------------------------------------------------------ 
