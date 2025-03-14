import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	#config = os.path.join(
	#	get_package_share_directory('mt10_control'),
	#	'config',
	#	'mt_sbg_config.yaml'
	#)
	config = '/home/mt10/mt10_ws/src/mt10_control/config/mt_sbg_config.yaml'

	return LaunchDescription([
		Node(
			package='sbg_driver',
		#	name='sbg_device_1',
			executable = 'sbg_device',
			output = 'screen',
			parameters = [config]
		)
	])
