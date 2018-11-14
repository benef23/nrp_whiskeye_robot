
@nrp.Neuron2Robot(Topic('/husky/cmd_vel', geometry_msgs.msg.Twist))
def robot_control(t, left_wheel_neuron, right_wheel_neuron):
    import run_model


