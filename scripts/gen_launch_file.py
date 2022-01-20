#####################################################################################
# Generate launch file for ROS nodes needed to run an RL or other hardware controller
# Author: Sarah Bhaskaran
#####################################################################################

def write_file(names, filename='launch_file.launch'):
    contents = []
    contents.append('''<?xml version="1.0" encoding="UTF-8"?>

    <launch>

    	<arg name="description" default="following_real_vehicle_rl0719"/>

    	<arg name="margin" default="30.0"/>
    	<arg name="use_margin" default="false"/>
    	<arg name="SPEED_SCALE" default="1.0"/>
    	<arg name="HEADWAY_SCALE" default="1.0"/>
    	<arg name="enable_fs" default="true"/>
    	<arg name="hwil" default="true"/>
    	<arg name="th1" default="0.4"/>
    	<arg name="th2" default="1.2"/>
    	<arg name="th3" default="1.8"/>
    	<arg name="w1" default="4.5"/>
    	<arg name="w3" default="6.0"/>
    	<arg name="w2" default="5.25"/>
    	<arg name="ego_veltopic" default="/vel"/>
            <arg name="relative_veltopic" default="/rel_vel"/>
            <arg name="headway_topic" default="/lead_dist"/>
            <arg name="use_accel_predict" default="true"/>
    	<arg name="T" default="0.6"/>
    	<arg name='hoffman' default='false'/>
    	<arg name='readonly' default='true'/>
    	<arg name='controllers' default='09_04_rll_ttc7p0scale1p8_maxacc0p25/super_resolution.onnx'/>
        ''')
    for name in names:
        contents.append(f'	<group ns="{name}">')
        contents.append('''
        		<param name="model" type="string" value="$(find onnx2ros)/../controllers/$(arg controllers)"/>
        	        <param name="mode" type="string" value="prompt"/>
        		<param name="/margin" value="$(arg margin)"/>
        		<param name="description" value="$(arg description)_enable_$(arg enable_fs)"/>
        		<param name="SPEED_SCALE" value="$(arg SPEED_SCALE)"/>
        		<param name="HEADWAY_SCALE" value="$(arg HEADWAY_SCALE)"/>
        	        <param name="ego_vel_topic" value="$(arg ego_veltopic)"/>
        	        <param name="relative_vel_topic" value="$(arg relative_veltopic)"/>
        	        <param name="headway_topic" value="$(arg headway_topic)"/>
        	        <param name="use_lead_vel" value="false"/>
        	        <param name="use_accel_predict" value="$(arg use_accel_predict)"/>
        	        <param name="T" value="$(arg T)"/>


        		<param name="/th1" value="$(arg th1)"/>
        		<param name="/th2" value="$(arg th2)"/>
        		<param name="/th3" value="$(arg th3)"/>
        		<param name="/w1" value="$(arg w1)"/>
        		<param name="/w2" value="$(arg w2)"/>
        		<param name="/w3" value="$(arg w3)"/>

        		<param name="/readonly" value="$(arg readonly)"/>
        		<param name="/hwil" value="$(arg hwil)"/>
        		<param name="/use_margin" value="$(arg use_margin)"/>
        		<param name="/enable_fs" value="$(arg enable_fs)"/>

        		<node pkg="can_to_ros" type ="subs_fs" name="subs_fs" output="screen" if="$(arg hwil)"/>

        		<node pkg="can_to_ros" type="lead_info" name="lead_info" output="screen" if="$(arg hwil)"/>

        		<node pkg="can_to_ros" type="simple_mini_car_from_lead_distance" name="simple_mini_car_from_lead_distance" output="screen" if="$(arg hwil)"/>

        		<!-- <node pkg="can_to_ros" type="cruise_start_safety_check" name="cruise_start_safety_check" output="screen" if="$(arg hwil)"/> -->


        			<!--group ns="catvehicle"-->
        		<node pkg="margin" type="margin_node" name="margin_node" output="screen" if="$(arg use_margin)">
        		</node>
        		<node pkg="onnx2ros" type="prompt_mode" name="controller" output="screen">
        			<remap from="lead_dist" to="effective_lead_dist" if="$(arg use_margin)"/>
        		        <remap from="v_des" to="/cmd_vel" if="$(eval arg('enable_fs') == False and arg('hoffman') == False )"/>
        		        <remap from="v_des" to="/cmd_control_vel" if="$(eval arg('hwil') == False and (arg('enable_fs') == False and arg('hoffman') == True) )"/>
        			<remap from="v_des" to="v_ref" if="$(arg enable_fs)"/>
        	   	</node>


        		<node pkg="followerstopperth4rl" type="followerstopperth4rl_node" name="followerstopperth4rl_node" output="screen" if="$(arg enable_fs)">
        			<remap from="lead_dist" to="effective_lead_dist" if="$(arg use_margin)"/>
        			<remap from="/cmd_vel" to ="/cmd_control_vel" if="$(eval arg('hoffman') == True and arg('hwil') == False)"/>
        	        </node>

        		<node pkg="hoffmansubsystem" type="hoffmansubsystem_node" name="hoffmansubsystem_ego" output="screen" if="$(arg hoffman)"/>

        		<node pkg="velocity_controller" type="velocity_controller_node" name="velocity_controller_node" output="screen" if="$(eval not readonly and hwil)">
        			<remap from="/vehicle/vel" to="/vel"/>
        		</node>

        		<node pkg="velocity_controller" type="velocity_controller_node" name="velocity_controller_readonly_node" output="screen" if="$(eval readonly and hwil)">
        			<remap from="/vehicle/vel" to="/vel"/>
        			<remap from="/cmd_accel" to="/cmd_accel_null"/>
        		</node>

        		<node pkg="can_to_ros" type ="rosbag_record.sh" name="bashscript2" output="screen" args="$(arg description)_enable_$(arg enable_fs) $(arg hwil)" />

        		<node pkg="transfer_pkg" type="saveparam.py" name="saveparam" output="screen" args="$(arg description)_enable_$(arg enable_fs) $(arg hwil)">
        		</node>

        		<node pkg="accel" type="accel" name="accel">
        		</node>
        </group>
        ''')
    contents.append('</launch>\n')
    total_contents = '\n'.join(contents)
    with open(filename, 'w+') as f:
        f.write(total_contents)

def gen_from_launch_file(names, base_launch_file, output_filename='launch_file.launch', accel_names=('accel', 'accel', 'accel')):
    ''' Takes a launch file for a single vehicle and converts it to work for an arbitrary number
        of vehicles.

        accel_names: (pkg, type, name) Name of the acceleration dynamics package and command to run it.
            Pass None if it is already in the file.
    '''
    import xml.etree.ElementTree as ET
    import xml.dom.minidom as minidom

    tree = ET.parse(base_launch_file)
    root = tree.getroot()
    inner = root.findall('./*')
    for c in inner:
        root.remove(c)

    for name in names:
        child = ET.SubElement(root, 'group', {'ns': name})
        for c in inner:
            child.append(c)
        # Accel dynamics
        if accel_names is not None:
            acc = ET.SubElement(child, 'node', {
                'pkg': accel_names[0],
                'type': accel_names[1],
                'name': accel_names[2]
            })

    xmlstr = ET.tostring(root)
    m = minidom.parseString(xmlstr)
    prettier_xml = m.toprettyxml(newl='')

    with open(output_filename, 'w') as f:
        f.write(prettier_xml)

