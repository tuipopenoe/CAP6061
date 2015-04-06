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
        -- FrontSensor=simGetObjectHandle("Front_Sensor")
        --LeftSensor=simGetObjectHandle("Left_Sensor")
        --RightSensor=simGetObjectHandle("Right_Sensor")
        Goal=simGetObjectHandle("pad3")
        Zero_Point=simGetObjectHandle("Zero_Point") 
        backUntilTime=-1 
        Front=simGetObjectHandle("LaserScanner_2D_F")
        Back=simGetObjectHandle("LaserScanner_2D_B")
    
        speed=2
        state=0
        forwardVel=1
        Count=-180
        D_P_X={}
        D_P_Y={}
        Disp_Rob_Point={}
        i=0
        j=0
        Omin=100
        Pos_Omin={}
        Max_Dist=0
        Count_Disc=2
        Pos_Obj={}
        laserScannerHandle_F=simGetObjectHandle("LaserScanner_2D_F")
        laserScannerObjectName_F=simGetObjectName(laserScannerHandle_F) -- is not necessarily "LaserScanner_2D"!!!
        ------------
        laserScannerHandle_L=simGetObjectHandle("LaserScanner_2D_L")
        laserScannerObjectName_L=simGetObjectName(laserScannerHandle_L) -- is not necessarily "LaserScanner_2D"!!!
        -----
        laserScannerHandle_B=simGetObjectHandle("LaserScanner_2D_B")
        laserScannerObjectName_B=simGetObjectName(laserScannerHandle_B) -- is not necessarily "LaserScanner_2D"!!!
        ------
        laserScannerHandle_R=simGetObjectHandle("LaserScanner_2D_R")
        laserScannerObjectName_R=simGetObjectName(laserScannerHandle_R) -- is not necessarily "LaserScanner_2D"!!!
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
                if angle > limit then
                    simSetJointTargetVelocity(leftMotor,speed*((120-angle)/150))
                    simSetJointTargetVelocity(rightMotor,speed*2.0)
        
                elseif angle<(-limit) then
                    simSetJointTargetVelocity(leftMotor,speed*2.0)
                    simSetJointTargetVelocity(rightMotor,speed*((120+angle)/150))
                else
                    simSetJointTargetVelocity(leftMotor,3*speed)
                    simSetJointTargetVelocity(rightMotor,3*speed)
                end
        end
    end

    -- Read the laser output, and return the points of the nearest objects
    Read_Laser=function()
            laserDetectedPoints_F=simReceiveData(0,laserScannerObjectName_F.."_2D_SCANNER_DATA_F")
            if (laserDetectedPoints_F) then
                Points_F=simUnpackFloats(laserDetectedPoints_F)
                --aa=table.getn(laserDetectedPoints)
            end
            laserDetectedPoints_L=simReceiveData(0,laserScannerObjectName_L.."_2D_SCANNER_DATA_L")
            if (laserDetectedPoints_L) then
                Points_L=simUnpackFloats(laserDetectedPoints_L)
            end
            laserDetectedPoints_B=simReceiveData(0,laserScannerObjectName_B.."_2D_SCANNER_DATA_B")
            if (laserDetectedPoints_B) then
                Points_B=simUnpackFloats(laserDetectedPoints_B)
            end
            laserDetectedPoints_R=simReceiveData(0,laserScannerObjectName_R.."_2D_SCANNER_DATA_R")
            if (laserDetectedPoints_R) then
                Points_R=simUnpackFloats(laserDetectedPoints_R)
            end
        return Points_F, Points_L, Points_B, Points_R
    end
    
    -- Return the distance to the nearest object using Angle_L
    Read_Laser_Angle=function(Angle_L)
        Laser_F,Laser_L,Laser_B,Laser_R=Read_Laser()
        
        if Angle_L <(-135) then
                local Pos_x=Laser_B[(((Angle_L+226)*2)*3)+1]+6 -- 2 = Y   1=X
                local Pos_y=Laser_B[(((Angle_L+226)*2)*3)+2]+6 -- 2 = Y   1=X
                Dist_L=((((Pos_Robot[1]-Pos_x)^2)+((Pos_Robot[2]-Pos_y)^2)    )^(1/2))
        elseif Angle_L <(-45) then 
                local Pos_x=Laser_R[(((Angle_L+135)*2)*3)+1]+6 -- 2 = Y   1=X
                local Pos_y=Laser_R[(((Angle_L+135)*2)*3)+2]+6 -- 2 = Y   1=X
                Dist_L=((((Pos_Robot[1]-Pos_x)^2)+((Pos_Robot[2]-Pos_y)^2)    )^(1/2))
        elseif Angle_L <(46) then
                local Pos_x=Laser_F[(((Angle_L+45)*2)*3)+1]+6 -- 2 = Y   1=X
                local Pos_y=Laser_F[(((Angle_L+45)*2)*3)+2]+6 -- 2 = Y   1=X
                Dist_L=((((Pos_Robot[1]-Pos_x)^2)+((Pos_Robot[2]-Pos_y)^2)    )^(1/2))
        elseif Angle_L < (136) then 
                local Pos_x=Laser_L[(((Angle_L-46)*2)*3)+1]+6 -- 2 = Y   1=X
                local Pos_y=Laser_L[(((Angle_L-46)*2)*3)+2]+6 -- 2 = Y   1=X
                Dist_L=((((Pos_Robot[1]-Pos_x)^2)+((Pos_Robot[2]-Pos_y)^2)    )^(1/2))
        else
                local Pos_x=Laser_B[(((Angle_L-136)*2)*3)+1]+6 -- 2 = Y   1=X
                local Pos_y=Laser_B[(((Angle_L-136)*2)*3)+2]+6 -- 2 = Y   1=X
                Dist_L=((((Pos_Robot[1]-Pos_x)^2)+((Pos_Robot[2]-Pos_y)^2)    )^(1/2))
        end
        return  Dist_L
    end
    
    
    Disc=function()
        Laser_F,Laser_L,Laser_B,Laser_R=Read_Laser()
        for i=0,89 do
            local Pos_x1=Laser_F[(((i)*2)*3)+1]+6 -- 2 = Y   1=X
            local Pos_y1=Laser_F[(((i)*2)*3)+2]+6 -- 2 = Y   1=X
            local Pos_x2=Laser_F[(((i+1)*2)*3)+1]+6 -- 2 = Y   1=X
            local Pos_y2=Laser_F[(((i+1)*2)*3)+2]+6 -- 2 = Y   1=X
            local Dist_F=((((Pos_x1-Pos_x2)^2)+((Pos_y1-Pos_y2)^2)    )^(1/2))
            local Dist_1=((((Pos_Robot[1]-Pos_x1)^2)+((Pos_Robot[2]-Pos_y1)^2)    )^(1/2))
            local Dist_2=((((Pos_Robot[1]-Pos_x2)^2)+((Pos_Robot[2]-Pos_y2)^2)    )^(1/2))
        
            if Dist_F>2 then
                if Dist_1 < Dist_2 then
                    D_P_X[Count_Disc]=Pos_x1
                    D_P_Y[Count_Disc]=Pos_y1
                    Disp_Rob_Point[Count_Disc]=Dist_1
                    Count_Disc=Count_Disc+1
                else
                    D_P_X[Count_Disc]=Pos_x2
                    D_P_Y[Count_Disc]=Pos_y2
                    Disp_Rob_Point[Count_Disc]=Dist_2
                    Count_Disc=Count_Disc+1
                end
            end
    
            local Pos_x1=Laser_L[(((i)*2)*3)+1]+6 -- 2 = Y   1=X
            local Pos_y1=Laser_L[(((i)*2)*3)+2]+6 -- 2 = Y   1=X
            local Pos_x2=Laser_L[(((i+1)*2)*3)+1]+6 -- 2 = Y   1=X
            local Pos_y2=Laser_L[(((i+1)*2)*3)+2]+6 -- 2 = Y   1=X
            local Dist_L=((((Pos_x1-Pos_x2)^2)+((Pos_y1-Pos_y2)^2)    )^(1/2))      
            local Dist_1=((((Pos_Robot[1]-Pos_x1)^2)+((Pos_Robot[2]-Pos_y1)^2)    )^(1/2))
            local Dist_2=((((Pos_Robot[1]-Pos_x2)^2)+((Pos_Robot[2]-Pos_y2)^2)    )^(1/2))
        
            if Dist_L>2 then
                if Dist_1 < Dist_2 then
                    D_P_X[Count_Disc]=Pos_x1
                    D_P_Y[Count_Disc]=Pos_y1
                    Disp_Rob_Point[Count_Disc]=Dist_1
                    Count_Disc=Count_Disc+1
                else
                    D_P_X[Count_Disc]=Pos_x2
                    D_P_Y[Count_Disc]=Pos_y2
                    Disp_Rob_Point[Count_Disc]=Dist_2
                    Count_Disc=Count_Disc+1
                end
            end
    
            local Pos_x1=Laser_B[(((i)*2)*3)+1]+6 -- 2 = Y   1=X
            local Pos_y1=Laser_B[(((i)*2)*3)+2]+6 -- 2 = Y   1=X
            local Pos_x2=Laser_B[(((i+1)*2)*3)+1]+6 -- 2 = Y   1=X
            local Pos_y2=Laser_B[(((i+1)*2)*3)+2]+6 -- 2 = Y   1=X
            local Dist_B=((((Pos_x1-Pos_x2)^2)+((Pos_y1-Pos_y2)^2)    )^(1/2))      
            local Dist_1=((((Pos_Robot[1]-Pos_x1)^2)+((Pos_Robot[2]-Pos_y1)^2)    )^(1/2))
            local Dist_2=((((Pos_Robot[1]-Pos_x2)^2)+((Pos_Robot[2]-Pos_y2)^2)    )^(1/2))
        
            if Dist_B>2 then
                if Dist_1 < Dist_2 then
                    D_P_X[Count_Disc]=Pos_x1
                    D_P_Y[Count_Disc]=Pos_y1
                    Disp_Rob_Point[Count_Disc]=Dist_1
                    Count_Disc=Count_Disc+1
                else
                    D_P_X[Count_Disc]=Pos_x2
                    D_P_Y[Count_Disc]=Pos_y2
                    Disp_Rob_Point[Count_Disc]=Dist_2
                    Count_Disc=Count_Disc+1
                end
            end
    
            local Pos_x1=Laser_R[(((i)*2)*3)+1]+6 -- 2 = Y   1=X
            local Pos_y1=Laser_R[(((i)*2)*3)+2]+6 -- 2 = Y   1=X
            local Pos_x2=Laser_R[(((i+1)*2)*3)+1]+6 -- 2 = Y   1=X
            local Pos_y2=Laser_R[(((i+1)*2)*3)+2]+6 -- 2 = Y   1=X
            local Dist_R=((((Pos_x1-Pos_x2)^2)+((Pos_y1-Pos_y2)^2)    )^(1/2))      
            local Dist_1=((((Pos_Robot[1]-Pos_x1)^2)+((Pos_Robot[2]-Pos_y1)^2)    )^(1/2))
            local Dist_2=((((Pos_Robot[1]-Pos_x2)^2)+((Pos_Robot[2]-Pos_y2)^2)    )^(1/2))
        
            if Dist_R>2 then
                if Dist_1 < Dist_2 then
                    D_P_X[Count_Disc]=Pos_x1
                    D_P_Y[Count_Disc]=Pos_y1
                    Disp_Rob_Point[Count_Disc]=Dist_1
                    Count_Disc=Count_Disc+1
                else
                    D_P_X[Count_Disc]=Pos_x2
                    D_P_Y[Count_Disc]=Pos_y2
                    Disp_Rob_Point[Count_Disc]=Dist_2
                    Count_Disc=Count_Disc+1
                end
            end
    
            if i==89 then
                D_P_X[1]=Count_Disc-2
                D_P_Y[1]=Count_Disc-2
                Disp_Rob_Point[1]=Count_Disc-2
                Count_Disc=2
            end
        end
        print("D_P_X")
        print_table(D_P_X)
    return D_P_X, D_P_Y, Disp_Rob_Point
    end

    -- ******************************************************************************* 
    print_table = function(table)
        for i, v in ipairs(table) do print(i, v) end
    end


    simHandleChildScripts(sim_call_type)
    Pos_Robot=simGetObjectPosition(BoddyRobot,Zero_Point)

    obstacle_detected = function()
        -- Calculate the position of the Goal relative to Zero_Point
        Pos_Goal = simGetObjectPosition(Goal, Zero_Point)
        -- Calculate the distance and angle of the robot to the Goal
        dist, angle = Dist_Ang_Rob_Point(Pos_Goal)
        angle = math.floor(angle)
        -- Calculate the distance from the laser to the nearest object
        dist_laser = Read_Laser_Angle((angle))
        -- Calculate the distance and angle from the Robot to the Goal
        dist, angle = Dist_Ang_Rob_Point(Pos_Goal)
        if dist_laser < dist then
            -- Object detected
            return true
        else
            return false
        end
    end

    move_towards_tangent_point = function()

    end

    if state == 0 then
        if obstacle_detected then
            state = 1
        else
            Laser_50=Read_Laser_Angle((50))
            Laser_130=Read_Laser_Angle((130))
            Laser_m50=Read_Laser_Angle((-50))
            Laser_m130=Read_Laser_Angle((-130))

            if ((Laser_50 < 0.4) or (Laser_m130<0.4)) then
                simSetJointTargetVelocity(leftMotor,speed*2)
                simSetJointTargetVelocity(rightMotor,speed*(0.5))
            elseif ( (Laser_130<0.4) or(Laser_m50 <0.4)) then
                simSetJointTargetVelocity(leftMotor,speed*0.5)
                simSetJointTargetVelocity(rightMotor,speed*(2))
            else
            Go_Point(Pos_Goal)
            end
            state=0
        end
    end
    
    if state == 1 then  -- Go far of contact point
                -- Pos_Goal = simGetObjectPosition(Goal, Zero_Point)
                -- dist, angle = Dist_Ang_Rob_Point(Pos_Goal)
                -- angle = math.floor(angle)
                -- dist_laser = Read_Laser_Angle((angle))
                -- dist, angle = Dist_Ang_Rob_Point(Pos_Goal)
                Disc()
    end

------------------------------------------------------------------------------ 
-- Following few lines automatically added by V-REP to guarantee compatibility 
-- with V-REP 3.1.3 and later: 
end 
------------------------------------------------------------------------------ 
