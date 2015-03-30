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
        -- Set a timestamp variable
        timestamp = 0
        -- Set a closest distance counter
        minimum_distance = 0
        --
        circumnavigate_time = 0
        -- Set mark_variables

        -- Get the Absolute position of the robot. Use this as
        -- the initial mark_point
        mark_point = simGetObjectPosition(BoddyRobot, -1)
        -- Use this flag to determine if the robot has circumnavigated the
        -- current object, and can begin moving towards the minimum distance
        -- point
        mark = false
        -- Marks the time since the minimum distance from the goal point on
        -- an object has been reached.
        mark_time = 0
        -- Calculation of distance from goal at mark point. 
        -- TODO: replace with mark_point calculation
        mark_distance = 0
        -- Calculate initial angle between robot and target
        initial_angle, initial_distance = Vector_To_Robot()
    end

    Calculate_Vector = function(object_1, object_2)
        print("Calculate Vector")
        -- Gets the position of object_1 relative to object_2
        -- returns coordinates as (x, y, z)
        local relative_position = simGetObjectPosition(object_1, object_2)
        -- Calculate the distance between the two objects
        local distance_of_object_one_from_two = (
            ((relative_position[1]^2) + (relative_position[2]^2))^(1/2))
        -- Calculate the angle between the two objects
        local angle_between_objects = math.deg(
            math.atan2((relative_position[2]), ((relative_position[1]))))
        -- Return the distance and angle as a tuple
        return distance_of_object_one_from_two, angle_between_objects
    end

    -- Calculates the angle and distance between the robot and the goal
    -- relative to the robot
    Vector_To_Robot = function()
        print("Vector_To_Robot()")
        return Calculate_Vector(Goal, BoddyRobot)
    end

    -- Calculate the angle and distance between the goal and robot, relative to 
    -- the goal
    Vector_To_Goal = function()
        print("Vector_To_Goal()")
        return Calculate_Vector(BoddyRobot, Goal)
    end

    Distance_To_Mark_Point = function()
        local pos_bot = simGetObjectPosition(BoddyRobot, )
    end

    -- If an object is detected, go into a state moving along the object using 
    -- the rotation to follow it.
    Circumnavigate_Object = function()
        print("Circumnavigate_Object()")
        if (obstacle_detected > 0) then 
            simSetJointTargetVelocity(leftMotor, speed * 1)
            simSetJointTargetVelocity(rightMotor, speed * -0.1)
        else
            simSetJointTargetVelocity(leftMotor, 1 * speed)
            simSetJointTargetVelocity(rightMotor, 3.0 * speed)
        end
    end

    -- Go towards the target, rotating as necessary to adjust the angle. 
    -- If the target is within 0.2 units, halt forward motion
    Move_Towards_Goal = function()
        print("Move_Towards_Goal()")
        local dist, angle = Vector_To_Robot()
        limit = 5
        -- If the target is reached, halt the motion of the robot
        if dist < 0.2 then
            simSetJointTargetVelocity(leftMotor, speed * 0)
            simSetJointTargetVelocity(rightMotor, speed * 0)

        else
            -- If the angle between the robot forward vector is greater than 
            -- the limit, rotate towards the goal
            if angle > limit then
                --simAddBanner((Dist_Rob_Mov),5,sim_banner_bitmapfont+sim_banner_overlay,nil,BoddyRobot,red,yellow)
            simSetJointTargetVelocity(leftMotor, speed * ((120 - angle)/150))
            simSetJointTargetVelocity(rightMotor, speed * 2.0)
            -- If the angle between the robot forward vector is less than 
            -- the limit then rotate the robot towards the goal
            elseif angle < (-limit) then
                simSetJointTargetVelocity(leftMotor, speed * 2.0)
                simSetJointTargetVelocity(rightMotor, speed * ((120+angle)/150))
            -- Otherwise, move forward at the default speed
            else
                simSetJointTargetVelocity(leftMotor, 1 * speed)
                simSetJointTargetVelocity(rightMotor, 1 * speed)
            end
        end
    end

    -- Execution Loop:
    -- Handle any child scripts
    simHandleChildScripts(sim_call_type)
    -- Get input from the front touch sensor
    obstacle_detected = simReadProximitySensor(FrontSensor)
    -- Calculate the distance and angle to the goal, split tuple into variables
    dist, angle = Vector_To_Robot()

    -- State Machine:
    -- If in the forward state, rotate towards the goal, or move towards it
    if state == 0 then
        print("State 0")
        -- If an obstacle is detected, change to the wall following state
        if (obstacle_detected > 0 ) then
            state = 1
            -- Set the mark distance
            print("Setting mark Distance: ")
            mark_distance = Vector_To_Goal()[0]
            print(mark_distance)
            print("Mark: ")
            print(mark)
            -- Set the mark time
            mark_time = timestamp
        else
            Move_Towards_Goal()
            minimum_distance = Vector_To_Goal()[0]
        end 
    elseif state == 1 then
        print("State 1")
        current_distance = Vector_To_Goal()
        print("Current Distance: ")
        print(current_distance)
        print("Minimum Distance: ")
        print(minimum_distance)
        print("Mark: ")
        print(mark)
        print(mark_time)
        -- If current is less than closest, set closest to current
        if current_distance < minimum_distance then
            print("Setting New Minimum Distance: ")
            print(minimum_distance)
            minimum_distance = current_distance
        end
        -- If the current distance is approximately equal to the mark distance
        -- i.e. object has been circumnavigated, set marker to true, and next
        -- min distance, move towards goal.
        if (current_distance - mark_distance) < 0.05 and timestamp > mark_time + 10 then
            mark = true
        end
        -- If robot has completely circumnavigated obstacle, and the current
        -- distance is approximately the minimum distance, move towards the goal
        if mark == true and (math.abs(current_distance) - math.abs(minimum_distance)) < 0.05 then
            state = 0
            mark_time = 0
            mark_distance = 0
            mark = false
            Move_Towards_Goal()
        else
            Circumnavigate_Object()
        end
    end

    timestamp = timestamp + 1
end 