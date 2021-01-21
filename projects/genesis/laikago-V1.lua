--[[
Callback function for reciving motor positions
from ROS controller node
--]]
-----------------------**********************--------------------------
	---- serial communication------
local luabit = require("bit")

function LobotSerilCommand(id, signal, tm)

        if((id ==1) or (id==4) or (id==7) or (id==10)) then    
        position =kslope[1]*signal +bslope[1]
        elseif((id ==2) or (id==5) or (id==8) or (id==11)) then       
        position =kslope[2]*signal +bslope[2]
        else
        position =kslope[3]*signal +bslope[3]
        end

        LobotSerialServoMove(id, position, tm)

end

function LobotSerialServoMove(id, position, tm)
    if(position > 450) then
        position = 450
    elseif(position <0) then
        position =0
    end

    if(tm > 65534) then
        tm =65534
    elseif(tm <0) then
        tm =0
    end
    
   local buf={0x55, 0x55, 0x01, 0x07, 0x01, 0x00, 0x00, 0x00, 0x00,0x00}
    buf[1],buf[2] = 0x55, 0x55
    buf[3] = id
    buf[4] =0x7
    buf[5] = 0x01
    buf[6] = GetLowByte(position)
    buf[7] = GetHighByte(position)
    buf[8] = GetLowByte(tm)
    buf[9] = GetHighByte(tm)
    buf[10] = math.mod(LobotCheckSum(buf),256)
    LobotSerialWrite(buf,10)
end

function LobotSerialWrite(buf,len)
    local i =1
    while(i < len+1)
    do
      sim.serialSend(SerialPort,string.char(buf[i]))  
      i = i+1
    end

end
function GetLowByte(position)
    local temp = math.abs(position)
    local temp1 = math.floor(temp)       
    local tempL=math.mod(temp1,256)
    return tempL
end

function GetHighByte(position)
    local temp =  math.abs(position)
    local temp1 = math.floor(temp)
    local tempH =math.floor(temp1/256)
    return tempH
end

function LobotCheckSum(buf)
    local i, temp =3, 0
    while(i < buf[4]+3)
    do
    temp = temp +buf[i]
    i = i+1
    end
    temp1 = math.mod(temp,256)
    local temp2=luabit:_not(temp1)
    return temp2
end

function SerialInit()

    SerialPortString="/dev/ttyUSB0"
    SerialPort=sim.serialOpen(SerialPortString,115200)
    print('Serial Port:'..SerialPort)
    if(SerialPort==-1)
    then
    
    end
        j1_min_angle=0
        j1_max_angle=450
        j1_range =j1_max_angle -j1_min_angle

        j2_min_angle=0       
        j2_max_angle=350
        j2_range =j2_max_angle -j2_min_angle


        j3_min_angle=0
        j3_max_angle=800
        j3_range =j3_max_angle -j3_min_angle


        
        j1_k=(j1_max_angle-j1_min_angle)/2.0
        j1_b=j1_max_angle - 1.0*j1_k
        j2_k=(j2_max_angle-j2_min_angle)/2.0
        j2_b=j2_max_angle - 1.0*j2_k
        j3_k=(j3_max_angle-j3_min_angle)/2.0
        j3_b=j3_max_angle -1.0*j3_k
        kslope={j1_k,j2_k,j3_k}
        bslope={j1_b,j2_b,j3_b}

    return SerialPort
end


--------------------**********************************---------------
function buttonClick()
	sim.addStatusbarMessage('You clicked the button')
	buttonClickState = true
end

function setMotorPositions()
	data={}
	if (buttonClickState == true)
	 then
   		for i = 1 , #joint_array do
 	 		data[i] = CPGData[i] --+ReflexData[i]
			simSetJointTargetPosition(joint_array[i], data[i])	
		end
	else
		for i = 1 , #joint_array do
    		data[i] = ReflexData[i] -- the ReflexData come from python rosnode
			simSetJointTargetPosition(joint_array[i], data[i])	
		end
	end
	
end

function setCPGMotorData(msg)
	CPGData=msg.data
end
function setReflexMotorData(msg)
	ReflexData=msg.data
end

--[[
Callback function for displaying data on a graph
resived from ROS controller node
--]]

function graph_cb(msg)
    data = msg.data
    simSetGraphUserData(graphHandleNeuroNetworkCPG,"CPGN0",data[1])
    simSetGraphUserData(graphHandleNeuroNetworkCPG,"CPGN1",data[2])
    simSetGraphUserData(graphHandleNeuroNetworkCPG,"PCPGN0",data[3])
    simSetGraphUserData(graphHandleNeuroNetworkCPG,"PCPGN1",data[4])
    simSetGraphUserData(graphHandleNeuroNetworkCPG,"PSN10",data[5])
    simSetGraphUserData(graphHandleNeuroNetworkCPG,"PSN11",data[6])
    simSetGraphUserData(graphHandleNeuroNetworkCPG,"VRNHip",data[7])
    simSetGraphUserData(graphHandleNeuroNetworkCPG,"VRNKnee",data[8])
	simSetGraphUserData(graphHandleNeuroNetworkCPG,"ReflexOut0",data[9])
    simSetGraphUserData(graphHandleNeuroNetworkCPG,"ReflexOut1",data[10])
    simSetGraphUserData(graphHandleNeuroNetworkCPG,"ReflexOut2",data[11])
    simSetGraphUserData(graphHandleNeuroNetworkCPG,"GRF",data[12])
end


--[[
Function for handling slider changes on the QT UI
--]]
function sliderChange(ui,id,newVal)
	if id < 100
	 then	
        newVal = newVal/200
        simUI.setLabelText(ui,1000+id-10,'Value: '..newVal)
        controlParametersRight[id-9] = newVal
	else
        newVal = newVal/200
        simUI.setLabelText(ui,10000+id-100,'Value: '..newVal)
        controlParametersLeft[id-99] = newVal
	end
end
--- init paramters slide
function initParametersSlide()
 
	 -- User Interface setup
    xml = [[ <ui closeable="false" resizable="true" style="plastique" title="Control Right Legs ">
    <label text="Psn:" wordwrap="true" />
    <label text="Value: 0" id="1000" wordwrap="true" />
    <hslider id="10" tick-position="both-sides" tick-interval="1" minimum="-200" maximum="200" onchange="sliderChange" style="plastique" />

    <label text="VRNHip" wordwrap="true" />
    <label text="Value: 0" id="1001" wordwrap="true" />
    <hslider id="11" tick-position="both-sides" tick-interval="1" minimum="-200" maximum="200" onchange="sliderChange" style="plastique" />

    <label text="VRNKnee" wordwrap="true" />
    <label text="Value: 0" id="1002" wordwrap="true" />
    <hslider id="12" tick-position="both-sides" tick-interval="1" minimum="-200" maximum="200" onchange="sliderChange" style="plastique" />

    <label text="MNBias1" wordwrap="true" />
    <label text="Value: 0" id="1003" wordwrap="true" />
    <hslider id="13" tick-position="both-sides" tick-interval="1" minimum="-200" maximum="200" onchange="sliderChange" style="plastique" />
    
    <label text="MNBias2" wordwrap="true" />
    <label text="Value: 0" id="1004" wordwrap="true" />
    <hslider id="14" tick-position="both-sides" tick-interval="1" minimum="-200" maximum="200" onchange="sliderChange" style="plastique" />
    
    <label text="MNBias3" wordwrap="true" />
    <label text="Value: 0" id="1005" wordwrap="true" />
    <hslider id="15" tick-position="both-sides" tick-interval="1" minimum="-200" maximum="200" onchange="sliderChange" style="plastique" />
   
    <label text="CPGMi" wordwrap="true" />
    <label text="Value: 0" id="1006" wordwrap="true" />
    <hslider id="16" tick-position="both-sides" tick-interval="1" minimum="-200" maximum="200" onchange="sliderChange" style="plastique" />
  
    <label text="PCPGBeta" wordwrap="true" />
    <label text="Value: 0" id="1007" wordwrap="true" />
    <hslider id="17" tick-position="both-sides" tick-interval="1" minimum="-200" maximum="200" onchange="sliderChange" style="plastique" />
      <button text='Drive the Motor' on-click = "buttonClick" />
	</ui> ]]
	buttonClickState =false
    -- Create User Interface
    uiRight=simUI.create(xml)
	controlParametersRight={0.0, 0.03,0.06, 0.0,0.0,0.0, 0.02,0.0}--Psn,VrnKnee,VrnHip,bias1,bias2,bias3,NeuroNetworkMi,PNeuroNetworkbeta


    xml = [[ <ui closeable="false" resizable="true" style="plastique" title="Control Left Legs">
    <label text="Psn:" wordwrap="true" />
    <label text="Value: 0" id="10000" wordwrap="true" />
    <hslider id="100" tick-position="both-sides" tick-interval="1" minimum="-200" maximum="200" onchange="sliderChange" style="plastique" />

    <label text="VRNHip" wordwrap="true" />
    <label text="Value: 0" id="10001" wordwrap="true" />
    <hslider id="101" tick-position="both-sides" tick-interval="1" minimum="-200" maximum="200" onchange="sliderChange" style="plastique" />

    <label text="VRNKnee" wordwrap="true" />
    <label text="Value: 0" id="10002" wordwrap="true" />
    <hslider id="102" tick-position="both-sides" tick-interval="1" minimum="-200" maximum="200" onchange="sliderChange" style="plastique" />
	</ui> ]]
    -- Create User Interface
    uiLeft=simUI.create(xml)
	controlParametersLeft={0.0, 0.03,0.06}--Psn,VrnKnee,VrnHip,

	controlParameters={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}
    -- Initialize sliders and testparameters (these are shared with the ROS node)
    for i=1,#controlParametersRight do
	simExtCustomUI_setSliderValue(uiRight,9+i,controlParametersRight[i]*200)
    sliderChange(uiRight,9+i,controlParametersRight[i]*200)
    end
    for i=1,#controlParametersLeft do
	simExtCustomUI_setSliderValue(uiLeft,99+i,controlParametersLeft[i]*200)
    sliderChange(uiLeft,99+i,controlParametersLeft[i]*200)
    end
	-- Set position of the User Interface
    x, y=simExtCustomUI_getPosition(uiRight)
    simExtCustomUI_setPosition(uiRight, x+800, y-300, true)
    x, y=simExtCustomUI_getPosition(uiLeft)
    simExtCustomUI_setPosition(uiLeft, x+600, y-300, true)

end


--[[
Initialization: Called once at the start of a simulation
--]]
function sysCall_init()
    -- init slide for adjust paramterts
    initParametersSlide() 
   -- Create all handles
    stbotHandle=sim.getObjectAssociatedWithScript(sim.handle_self)
    graphHandleNeuroNetworkCPG=sim.getObjectHandle("NeuroNetworkGraphCPG")
    graphHandleJointTrace = sim.getObjectHandle("jointTrace")
	
	FR_joint_1=sim.getObjectHandle("stbot_FR_joint_1")
    FR_joint_2=sim.getObjectHandle("stbot_FR_joint_2")
    FR_joint_3=sim.getObjectHandle("stbot_FR_joint_3")

    FL_joint_1=sim.getObjectHandle("stbot_FL_joint_1")
    FL_joint_2=sim.getObjectHandle("stbot_FL_joint_2")
    FL_joint_3=sim.getObjectHandle("stbot_FL_joint_3")
    
    RL_joint_1=sim.getObjectHandle("stbot_RL_joint_1")
    RL_joint_2=sim.getObjectHandle("stbot_RL_joint_2")
    RL_joint_3=sim.getObjectHandle("stbot_RL_joint_3")

    RR_joint_1=sim.getObjectHandle("stbot_RR_joint_1")
    RR_joint_2=sim.getObjectHandle("stbot_RR_joint_2")
    RR_joint_3=sim.getObjectHandle("stbot_RR_joint_3")
	joint_array={FR_joint_1, FR_joint_2, FR_joint_3, RR_joint_1, RR_joint_2, RR_joint_3, FL_joint_1, FL_joint_2, FL_joint_3, RL_joint_1, RL_joint_2, RL_joint_3}
	FR_foot_sensor=sim.getObjectHandle("stbot_FR_footSensor")
	FL_foot_sensor=sim.getObjectHandle("stbot_FL_footSensor")
	RR_foot_sensor=sim.getObjectHandle("stbot_RR_footSensor")
	RL_foot_sensor=sim.getObjectHandle("stbot_RL_footSensor")
	foot_sensor_array = { FR_foot_sensor, RR_foot_sensor, FL_foot_sensor,RL_foot_sensor }
	stbot_body = sim.getObjectHandle("stbot_base")
	for i = 1, #joint_array do
		print(sim.getObjectName(joint_array[i]))
	end
	
	for i = 1, #foot_sensor_array do
		print(sim.getObjectName(foot_sensor_array[i]))
	end
	print(sim.getObjectName(graphHandleJointTrace))
	-- Check if the required ROS plugin is loaded
    moduleName=0
    moduleVersion=0
    index=0
    pluginNotFound=true
    while moduleName do
        moduleName,moduleVersion=sim.getModuleName(index)
        if (moduleName=='RosInterface') then
            pluginNotFound=false
        end
        index=index+1
    end
    if (pluginNotFound) then
        sim.displayDialog('Error','The RosInterface was not found.',sim.dlgstyle_ok,false,nil,{0.8,0,0,0,0,0},{0.5,0,0,1,1,1})
    end

    -- If found then start the subscribers and publishers
    if (not pluginNotFound) then
        local motorName='MotorPositions'
        local simulationTimeName='simTime'
        local terminateControllerName='terminateController'
        local startSimulationName='startSimulation'
        local pauseSimulationName='pauseSimulation'
        local stopSimulationName='stopSimulation'
        local enableSyncModeName='enableSyncMode'
        local triggerNextStepName='pauseSimulation'
        local simulationStepDoneName='simulationStepDone'
        local simulationStateName='simulationState'
        local neuroNetworkOutputName='NeuroNetworkOutput' 
        local sensorValueName='sensorValues'
		local reflexMotorName= 'reflexMotors'
		local rosRateParameter ='100'
		local legNumber = '4'
		local motorNumber = '12'
		local sensorNumber ='23'

        -- Create the subscribers
        MotorSub=simROS.subscribe('/'..motorName,'std_msgs/Float32MultiArray','setCPGMotorData')
		ReflexMotorSub=simROS.subscribe('/'..reflexMotorName,'std_msgs/Float32MultiArray','setReflexMotorData')
        NeuroNetworkOutputSub=simROS.subscribe('/'..neuroNetworkOutputName,'std_msgs/Float32MultiArray', 'graph_cb')

        -- Create the publishers
        terminateControllerPub=simROS.advertise('/'..terminateControllerName,'std_msgs/Bool')
        sensorValuePub=simROS.advertise('/'..sensorValueName,'std_msgs/Float32MultiArray')
        simulationTimePub=simROS.advertise('/'..simulationTimeName,'rosgraph_msgs/Clock')

        -- Start the client application (c++ node)
        result=sim.launchExecutable(simGetStringParameter(sim_stringparam_scene_path) .. '/../../../../projects/stbot/genesis/catkin_ws/devel/lib/stbot/stbot_node', motorName.." "..simulationTimeName.." "..terminateControllerName.." "..startSimulationName.." "..pauseSimulationName.." "..stopSimulationName.." "..enableSyncModeName.." "..triggerNextStepName.." "..simulationStepDoneName.." "..simulationStateName.." "..neuroNetworkOutputName.." "..sensorValueName.." "..reflexMotorName.." "..rosRateParameter.." "..legNumber.." "..motorNumber.." "..sensorNumber,0)
        if (result==false) then
            sim.displayDialog('Error','External CPG ROS-Node not found',sim.dlgstyle_ok,false,nil,{0.8,0,0,0,0,0},{0.5,0,0,1,1,1})
        end

--[[
	-- Start the client application (python node)	
        result=sim.launchExecutable(simGetStringParameter(sim_stringparam_scene_path) .. '/../../../projects/stbot/catkin_ws/src/stbot/scripts/stbot_reflex_node.py', motorName.." "..simulationTimeName.." "..terminateControllerName.." "..startSimulationName.." "..pauseSimulationName.." "..stopSimulationName.." "..enableSyncModeName.." "..triggerNextStepName.." "..simulationStepDoneName.." "..simulationStateName.." "..neuroNetworkOutputName.." "..sensorValueName.." "..reflexMotorName.." "..rosRateParameter.." "..legNumber.." "..motorNumber.." "..sensorNumber,0)
        if (result==false) then
            sim.displayDialog('Error','External reflex ROS-Node not found',sim.dlgstyle_ok,false,nil,{0.8,0,0,0,0,0},{0.5,0,0,1,1,1})
        end
--]]
    end


   	 -- Publish Initial Velocity and Position
	vel , pos = {}, {}
	sensor_array = {}
	for i = 1, #joint_array do
	pos[i] = simGetJointPosition(joint_array[i])
	sensor_array[i] = pos[i]
	end
    simROS.publish(sensorValuePub,{data=sensor_array})
    simROS.publish(simulationTimePub,{clock=simGetSimulationTime()})

	--Subscriber initial  data
	CPGData,ReflexData={},{}
	
	for i = 1, #joint_array do
	CPGData[i] = 0.0
	ReflexData[i] = 0.0
	end

	--  open serial port---
--	SerialInit()
end


--[[
Actuation: This part will be executed in each simulation step
--]]
function sysCall_actuation()

    -- Publish Clock for Sync.

--	setMotorPositions(CPGdata,ReflexData)
	setMotorPositions()
	
--	LobotSerilCommand(1,sensor_array[1],0)
--    LobotSerilCommand(2,sensor_array[2],0)
--    LobotSerilCommand(3,sensor_array[3],0)
end

--[[
Sensing: This part will be executed in each simulation step
--]]
function sysCall_sensing()

	-- Publish sensor values----------------	

    -- Joint Velocity and Position, attitude angles of body ,and Foot contact force
	for i = 1, #joint_array do
	pos[i] = simGetJointPosition(joint_array[i])
	sensor_array[i] = pos[i]
	end	
	
	for i = 1, #foot_sensor_array do
	reuslt,force,torque = sim.readForceSensor(foot_sensor_array[i])
	sensor_array[i + #joint_array] = force[3]/300.0	--let the value into [0,1]
	end												-- please don't change this value,

	eularAngles=sim.getObjectOrientation(stbot_body,-1)
	for i = 1, #eularAngles do
	sensor_array[i + #joint_array + #foot_sensor_array]= eularAngles[i]
	end

	for i = 1, #foot_sensor_array do
	reuslt,force,torque = sim.readForceSensor(foot_sensor_array[i])
	sensor_array[i + #joint_array + #foot_sensor_array + #eularAngles] = force[1]/300.0	--let the value into [0,1]
	end												-- please don't change this value,
    simROS.publish(sensorValuePub,{data=sensor_array})

	-- publish clock
    simROS.publish(simulationTimePub,{clock=simGetSimulationTime()})

    -- set the parameters use parameters service
	for i=1,#controlParametersRight do
	controlParameters[i]=controlParametersRight[i]
	end
	for i=1,#controlParametersLeft do
	controlParameters[#controlParametersRight+i]=controlParametersLeft[i]
	end
	simROS.setParamDouble("PSN_right",controlParameters[1]);
	simROS.setParamDouble("VRN_knee_right",controlParameters[2]);
	simROS.setParamDouble("VRN_hip_right",controlParameters[3]);

	simROS.setParamDouble("MNB1",controlParameters[4]);
	simROS.setParamDouble("MNB2",controlParameters[5]);
	simROS.setParamDouble("MNB3",controlParameters[6]);

	simROS.setParamDouble("CPGMi",controlParameters[7]);
	simROS.setParamDouble("PCPGBeta",controlParameters[8]);

	simROS.setParamDouble("PSN_left",controlParameters[9]);
	simROS.setParamDouble("VRN_knee_left",controlParameters[10]);
	simROS.setParamDouble("VRN_hip_left",controlParameters[11]);
end

--[[
Sensing: This part will be executed one time just before a simulation ends
--]]
function sysCall_cleanup()
    
    -- Wait for the signal to arive at the nodes
    waitTimer=0
    while( waitTimer < 50000 ) do
        waitTimer = waitTimer+1
    end

    -- Close UI
    simUI.destroy(uiRight)
    simUI.destroy(uiLeft)

    -- Clode ROS related stuff
    if not pluginNotFound then
        -- Send termination signal to external ROS nodes
        simROS.publish(terminateControllerPub,{data=true})

        -- Terminate subscriber
        simROS.shutdownSubscriber(NeuroNetworkOutputSub)
        simROS.shutdownSubscriber(MotorSub)
        simROS.shutdownSubscriber(ReflexMotorSub)
		-- terminate publisher
        simROS.shutdownPublisher(simulationTimePub)
        simROS.shutdownPublisher(sensorValuePub)
        simROS.shutdownPublisher(terminateControllerPub)
    end
	-- close serial port -----
--	sim.serialClose(SerialPort)
end

