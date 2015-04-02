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
    if not scripts_initialized then 
        scripts_initialized=0 
    end
    simSetScriptAttribute(sim_handle_self,
                          sim_scriptattribute_executioncount,
                          scripts_initialized)
    scripts_initialized = scripts_initialized+1

------------------------------------------------------------------------------  
    -- Init Function, called once, initializes variables
    -- This is executed exactly once, the first time this script is executed
    if (simGetScriptExecutionCount() == 0) then
        print("Initializing Variables")
        -- Get a Reference to the robot object group
        main_robot = simGetObjectAssociatedWithScript(sim_handle_self)
        -- Get a reference to the robot left motor by name
        left_motor = simGetObjectHandle("leftMotor")
        -- Get a reference to the robot right motor by name
        right_motor = simGetObjectHandle("rightMotor")
        -- Get a reference to the robot sensor by name
        front_sensor = simGetObjectHandle("Front_Sensor")
        --Only 1 sensor used in this simulation, left and right are unused
        --LeftSensor=simGetObjectHandle("Left_Sensor")
        --RightSensor=simGetObjectHandle("Right_Sensor")
        -- Get a reference to the goal object (the plane) by name
        goal = simGetObjectHandle("pad3")
        -- Set the base speed
        speed = 2
        -- Set the base state
        state = 0
        -- Set the base rotation
        rotation= -1
        -- Set a timestamp variable
        timestamp = 0
        -- Set a closest distance counter
        minimum_distance = 0
        --------------------
        -- MARK VARIABLES
        ---------------------
        -- Get the Absolute position of the robot. Use this as
        -- the initial mark_point
        mark_point = simGetObjectPosition(main_robot, -1)
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
        initial_distance = Vector_To_Robot()[1]
        initial_angle = Vector_To_Robot()[2]
    end

    Calculate_Vector = function(object_1, object_2)
        --print("Calculate_Vector()")
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

    Calculate_Vector_Points = function(point_1, point_2)
        print("Point_1")
        print(point_1[1], point_1[2], point_1[3])
        print("Point_2")
        print(point_2[2], point_2[2], point_2[3])
        local relative_position = {(point_2[1] - point_1[1]),
                                   (point_2[2] - point_1[2]),
                                   (point_2[3] - point_1[3])}
        local distance = (
            ((relative_position[1]^2) + (relative_position[2]^2))^(1/2))

        local angle = math.deg(
            math.atan2((relative_position[2]), ((relative_position[1]))))
        return distance, angle
    end

    -- Calculates the angle and distance between the robot and the goal
    -- relative to the robot
    Vector_To_Robot = function()
        --print("Vector_To_Robot()")
        --print(Calculate_Vector(goal, main_robot))
        return Calculate_Vector(goal, main_robot)
    end

    -- Calculate the angle and distance between the goal and robot, relative to 
    -- the goal
    Vector_To_Goal = function()
        --print("Vector_To_Goal()")
        -- print(Calculate_Vector(main_robot, goal))
        return Calculate_Vector(main_robot, goal)
    end

    Distance_To_Goal = function()
        --print("Distance_To_Goal()")
        return Vector_To_Goal()
    end

    Set_Mark_Point = function()
        print("Set_Mark_Point()")
        mark_point = simGetObjectPosition(main_robot, goal)
        print("Mark Point: ")
        print(mark_point)
    end

    Calculate_Distance_To_Mark_Point = function ()
        print("Calculate_Distance_To_Mark_Point")
        local robot_position = simGetObjectPosition(main_robot, goal)
        return Calculate_Vector_Points(robot_position, mark_point)
    end

    -- If an object is detected, go into a state moving along the object using 
    -- the rotation to follow it.
    Circumnavigate_Object = function()
        --print("Circumnavigate_Object()")
        if (obstacle_detected > 0) then 
            simSetJointTargetVelocity(left_motor, speed * 1)
            simSetJointTargetVelocity(right_motor, speed * -0.1)
        else
            simSetJointTargetVelocity(left_motor, 1 * speed)
            simSetJointTargetVelocity(right_motor, 3.0 * speed)
        end
    end

    -- Go towards the target, rotating as necessary to adjust the angle. 
    -- If the target is within 0.2 units, halt forward motion
    Move_Towards_Goal = function()
        --print("Move_Towards_Goal()")
        local dist, angle = Vector_To_Robot()
        limit = 5
        -- If the target is reached, halt the motion of the robot
        if dist < 0.2 then
            simSetJointTargetVelocity(left_motor, speed * 0)
            simSetJointTargetVelocity(right_motor, speed * 0)

        else
            -- If the angle between the robot forward vector is greater than 
            -- the limit, rotate towards the goal
            if angle > limit then
                --simAddBanner((Dist_Rob_Mov),5,sim_banner_bitmapfont+sim_banner_overlay,nil,main_robot,red,yellow)
            simSetJointTargetVelocity(left_motor, speed * ((120 - angle)/150))
            simSetJointTargetVelocity(right_motor, speed * 2.0)
            -- If the angle between the robot forward vector is less than 
            -- the limit then rotate the robot towards the goal
            elseif angle < (-limit) then
                simSetJointTargetVelocity(left_motor, speed * 2.0)
                simSetJointTargetVelocity(right_motor, speed * ((120+angle)/150))
            -- Otherwise, move forward at the default speed
            else
                simSetJointTargetVelocity(left_motor, 1 * speed)
                simSetJointTargetVelocity(right_motor, 1 * speed)
            end
        end
    end

    -------------------------------------------------------------------------
    -- EXECUTION LOOP
    -------------------------------------------------------------------------
    -- Handle any child scripts
    simHandleChildScripts(sim_call_type)
    -- Get input from the front sensor
    obstacle_detected = simReadProximitySensor(front_sensor)
    -- Calculate the distance and angle to the goal, split tuple into variables
    dist, angle = Vector_To_Robot()

    -- State Machine:
    -- If in the forward state, rotate towards the goal, or move towards it
    if state == 0 then
        --print("State 0")
        -- If an obstacle is detected, change to the wall following state
        if (obstacle_detected > 0 ) then
            -- Set the mark point
            Set_Mark_Point()
            -- Set the mark time
            mark_time = timestamp
            state = 1
        -- Otherwise, continue moving towards the goal. 
        else
            Move_Towards_Goal()
            minimum_distance = Distance_To_Goal()
        end 
    -- Otherwise, circumnavigate any intervening objects, finding a point around
    -- the object a minimum distance from the goal, and move the that point.
    -- Then, resume moving towards the goal.
    elseif state == 1 then
        --print("State 1")
        if (obstacle_detected > 0) then
            Circumnavigate_Object()
        else
            current_distance = Distance_To_Goal()
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
            print("Distance To Mark Point")
            local distance_to_mark_point = Calculate_Distance_To_Mark_Point()
            print(distance_to_mark_point)
            print("distance calculated")
            if distance_to_mark_point < 0.05 and 
                timestamp > (mark_time + 20)  then
                mark = true
            end
            -- If robot has completely circumnavigated obstacle, and the current
            -- distance is approximately the minimum distance, move towards the goal
            if mark == true and (math.abs(current_distance) - math.abs(minimum_distance)) < 0.05 then
                -- Reset mark variables
                mark_time = timestamp
                mark_point = simGetObjectPosition(main_robot)
                mark = false
                Move_Towards_Goal()
                state = 0
            else
                Circumnavigate_Object()
            end
        end
    end
    -- Increment the timestamp
    timestamp = timestamp + 1
    print("Timestamp: ")
    print(timestamp)
end