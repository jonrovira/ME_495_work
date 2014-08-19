simSetThreadSwitchTiming(2)
simDelegateChildScriptExecution()

-- Get some handles first:
local leftMotor=simGetObjectHandle("rosControlledBubbleRobLeftMotor") -- Handle of the left motor
local rightMotor=simGetObjectHandle("rosControlledBubbleRobRightMotor") -- Handle of the right motor
local noseSensor=simGetObjectHandle("rosControlledBubbleRobSensingNose") -- Handle of the proximity sensor

-- Check if the required ROS plugin is there:
moduleName=0
moduleVersion=0
index=0
pluginNotFound=true
while moduleName do
	moduleName,moduleVersion=simGetModuleName(index)
	if (moduleName=='Ros') then
		pluginNotFound=false
	end
	index=index+1
end

-- Add a banner:
if (pluginNotFound) then
	bannerText="I cannot run! (I couldn't find my ROS plugin)"
else
	bannerText="I am controlled via a ROS node! ('rosBubbleRob' controlls me)"
end
black={0,0,0,0,0,0,0,0,0,0,0,0}
red={0,0,0,0,0,0,0,0,0,1,0.2,0.2}
simAddBanner(bannerText,0,sim_banner_bitmapfont+sim_banner_overlay,nil,simGetObjectAssociatedWithScript(sim_handle_self),black,red)

-- Ok now launch the ROS client application:
if (not pluginNotFound) then

	-- Now we start the client application:
	result=simLaunchExecutable('vrep_test.py',leftMotor.." "..rightMotor.." "..noseSensor,0) -- set the last argument to 1 to see the console of the launched client (on Windows only)

end

-- This thread ends here. The bubbleRob will however still be controlled by
-- the client application via the ROS mechanism!