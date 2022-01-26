import rospy
from std_msgs.msg import Float64MultiArray
# pip install sympy
from sympy import Matrix as M
from sympy.physics.mechanics import dynamicsymbols
from sympy import simplify, symbols, pi
# REMEMBER: robot.py must be in current working directory
from robot import choose_robot
from gazebo_msgs.srv import *


# Load robot configuration from robot.py
r = choose_robot('rbe500_group_project')

# Instantiate variables for joint velocities and link lengths
q = M(dynamicsymbols('θ1, θ2, d3'))
links = symbols('l1, l2, l3, l4')

# Calculate forward kinematics
fk = simplify(r.forward_kinematics_dh())

# Calculate the linear velocity component of the Jacobian
ee_position = fk[:3, 3]
jv = simplify(ee_position.jacobian(q))

# Specify some default link lengths
link_defaults = [2, 2, 2, 1]

# Specify a default (home) position
home = [pi/4, -pi/4, 0]


def velocity_kinematics(
        joint_velocity, position=home, link_lengths=link_defaults):

    # Create lists of substitutions for symbolic variables
    q_subs = list(zip([joint for joint in q], position))
    l_subs = list(zip([link for link in links], link_lengths))

    # Calculate the numerical Jacobian for this specific position
    jacobian = jv.subs(q_subs)

    # Evaluate the numerical end effector velocity for the given velocities
    ee_velocity = jacobian @ M(joint_velocity)
    evaluation = ee_velocity.subs(l_subs).evalf()

    return evaluation


def inverse_velocity_kinematics(
        ee_velocity, link_lengths=link_defaults):

  # Determine joint positions
  while(1):
    rospy.wait_for_service('gazebo/get_joint_properties')
    get_joint_properties = rospy.ServiceProxy('gazebo/get_joint_properties', GetJointProperties,persistent=False)
    position = [float(get_joint_properties("joint1").position[0]),float(get_joint_properties("joint2").position[0]),float(get_joint_properties("joint3").position[0])]
    print(position)


    # Create lists of substitutions for symbolic variables
    q_subs = list(zip([joint for joint in q], position))
    l_subs = list(zip([link for link in links], link_lengths))

    # Calculate the numerical Jacobian for this specific position
    jacobian = jv.subs(q_subs)

    # Evaluate the numerical joint velocities for the given velocity
    joint_velocity = jacobian.pinv() @ M(ee_velocity)
    evaluation = joint_velocity.subs(l_subs).evalf()

    return evaluation


def velocity_kinematics_callback(float64_multi_array):
    data = list(float64_multi_array.data)
    arguments = {}

    if len(data) % 3 != 0 or len(data) < 3 or len(data) > 9:
        rospy.logerr('Array needs to have 3, 6, or 9 elements.')
        return
    if len(data) > 6:
        arguments['link_lengths'] = data[6:10]
    if len(data) > 3:
        arguments['position'] = data[3:6]
    arguments['joint_velocity'] = data[:3]
    rospy.loginfo(f'Velocity Kinematics Input: {arguments}')

    ee_velocity = velocity_kinematics(**arguments)
    rospy.loginfo(f'End Effector Velocity: {ee_velocity}')


def inverse_velocity_kinematics_callback(float64_multi_array):
    data = list(float64_multi_array.data)
    arguments = {}

    if len(data) % 3 != 0 or len(data) < 3 or len(data) > 9:
        rospy.logerr('Array needs to have 3, 6, or 9 elements.')
        return
    if len(data) > 6:
        arguments['link_lengths'] = data[6:10]
    if len(data) > 3:
        arguments['position'] = data[3:6]
    arguments['ee_velocity'] = data[:3]
    rospy.loginfo(f'Inverse Velocity Kinematics Input: {arguments}')
    
    joint_velocity = inverse_velocity_kinematics(**arguments)
    rospy.loginfo(f'Joint Velocity: {joint_velocity}')

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber(
        'velocity_kinematics',
        Float64MultiArray,
        velocity_kinematics_callback)
    rospy.Subscriber(
        'inverse_velocity_kinematics',
        Float64MultiArray,
        inverse_velocity_kinematics_callback)
    
def main():
    print(r.dh)

if __name__ == '__main__':
    listener()
