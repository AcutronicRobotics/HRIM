import sys
sys.path.insert(0, 'scripts')
from printing import ModulePrinter
from parsing import ModuleParser
import compiling
import os
from classes import *
import argparse

def main(args):
	parser = ModuleParser()
	if args.action == "show":
		extend = args.extend
		module = parser.parseFile(args.filePath)
		ModulePrinter().printModule(module, extend)
	elif args.action == "generate":
		# check for file generation shorthands
		if args.filePath == "all":
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
				module = parser.parseFile(file)
				compiling.main(module)
		elif args.filePath == "allClean":
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
				module = parser.parseFile(file)
				compiling.main(module)
		# else try to generate the implementation based on the passed file
		else:
			module = parser.parseFile(args.filePath)
			compiling.main(module)
	else:
		print "Unknown command"



if __name__ == '__main__':

	# Add usage messages

	parser=argparse.ArgumentParser(
	    description='''Hardware Robot Information Model (HRIM) implementation generation tool.''',
		formatter_class=argparse.RawTextHelpFormatter)
	parser.add_argument('action', choices=['show','generate'], help='''Action to take:
show:
	print a representation of the passed valid XML module's model structure and values.
generate:
	generate the platform-specific implementation of the passed valid XML model.
		''')
	parser.add_argument('filePath', help='The path to a valid xml model file.')
	parser.add_argument('-p', '--platform', default='ros2', choices=['ros2'], help='The platform for the generated model, ros2 by default.')
	parser.add_argument('-e', '--extend', action='store_true', default='False', help='Whether to expand topic definitions when "show"-ing.')

	# If no argument is provided, show usage
	if len(sys.argv) == 1:
		parser.print_help()
		sys.exit(0)

	args=parser.parse_args()

	main(args)
