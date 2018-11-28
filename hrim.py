import sys
sys.path.insert(0, 'scripts')
import printing
import parsing
import compiling
import os
from classes import *

def main(args):
	if args[1] == "show":
		extend = False
		# if topic/properties not defined on the parsed file should show their full content or not when printing
		if len(args) == 4:
			if args[3] == "1":
				extend = True
		# check for file input
		if len(args) < 3:
			print "Please input file to parse"
			sys.exit()
		module = parsing.main(args[2])
		printing.main(module, extend)
	elif args[1] == "generate":
		extend = False
		# check for file input
		if len(args) < 3:
			print "Please input file to parse"
			sys.exit()
		if args[2] == "all":
			path = os.getcwd()
			for file in [
				path+"/models/composite/mobilebase/mobilebase.xml",
				path+"/models/composite/conveyor/conveyor.xml",
				path+"/models/composite/arm/arm.xml",
				path+"/models/power/battery/battery.xml",
				path+"/models/actuator/gripper/gripper.xml",
				path+"/models/actuator/electricmotor/electricmotor.xml",
				path+"/models/actuator/servo/servo.xml",
				path+"/models/sensor/lidar/lidar.xml",
				path+"/models/sensor/rangefinder/rangefinder.xml",
				path+"/models/sensor/thermometer/thermometer.xml",
				path+"/models/sensor/3dcamera/3dcamera_stereo.xml",
				path+"/models/sensor/3dcamera/3dcamera_tof.xml",
				path+"/models/sensor/3dcamera/3dcamera_depth.xml",
				path+"/models/sensor/imu/imu.xml",
				path+"/models/sensor/hygrometer/hygrometer.xml",
				path+"/models/sensor/forcetorque/forcetorque.xml",
				path+"/models/sensor/gps/gps.xml",
				path+"/models/sensor/gasdetector/gasdetector.xml",
				path+"/models/sensor/torque/torque.xml",
				path+"/models/sensor/force/force.xml",
				path+"/models/sensor/microphone/microphone.xml",
				path+"/models/sensor/camera/camera.xml",
				path+"/models/sensor/encoder/encoder.xml"

			]:
				os.chdir(path)
				module = parsing.main(file)
				compiling.main(module)
		elif args[2] == "allClean":
			path = os.getcwd()
			for file in [
				path+"/models/composite/mobilebase/mobilebase_clean.xml",
				path+"/models/composite/conveyor/conveyor_clean.xml",
				path+"/models/composite/arm/arm_clean.xml",
				path+"/models/power/battery/battery_clean.xml",
				path+"/models/actuator/gripper/gripper_clean.xml",
				path+"/models/actuator/electricmotor/electricmotor_clean.xml",
				path+"/models/actuator/servo/servo_clean.xml",
				path+"/models/sensor/lidar/lidar_clean.xml",
				path+"/models/sensor/rangefinder/rangefinder_clean.xml",
				path+"/models/sensor/thermometer/thermometer_clean.xml",
				path+"/models/sensor/3dcamera/3dcamera_stereo_clean.xml",
				path+"/models/sensor/3dcamera/3dcamera_tof_clean.xml",
				path+"/models/sensor/3dcamera/3dcamera_depth_clean.xml",
				path+"/models/sensor/imu/imu_clean.xml",
				path+"/models/sensor/hygrometer/hygrometer_clean.xml",
				path+"/models/sensor/forcetorque/forcetorque_clean.xml",
				path+"/models/sensor/gps/gps_clean.xml",
				path+"/models/sensor/gasdetector/gasdetector_clean.xml",
				path+"/models/sensor/torque/torque_clean.xml",
				path+"/models/sensor/force/force_clean.xml",
				path+"/models/sensor/microphone/microphone_clean.xml",
				path+"/models/sensor/camera/camera_clean.xml",
				path+"/models/sensor/encoder/encoder_clean.xml"
			]:
				os.chdir(path)
				module = parsing.main(file)
				compiling.main(module)
		else:
			module = parsing.main(args[2])
			compiling.main(module)
	else:
		print "Unknown command"



if __name__ == '__main__':
	main(sys.argv)
