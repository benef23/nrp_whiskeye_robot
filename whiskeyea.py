
@nrp.MapRobotPublisher("topic3", Topic("/whiskeyea/world3/cmd_pos", std_msgs.msg.Float64))
@nrp.MapRobotPublisher("topic2", Topic("/whiskeyea/world1/cmd_pos", std_msgs.msg.Float64))
@nrp.MapRobotPublisher("topic", Topic("/whiskeyea/world2/cmd_pos", std_msgs.msg.Float64))
@nrp.Robot2Neuron()
def whiskeyea (t, topic, topic2, topic3):
    #log the first timestep (20ms), each couple of seconds
    if t % 2 < 0.02:
        clientLogger.info('Time: ', t)
        
    import math
    topic.send_message(std_msgs.msg.Float64(math.sin(0.45*t)))  
    topic2.send_message(std_msgs.msg.Float64(math.sin(0.45*t)))
    topic3.send_message(std_msgs.msg.Float64(-math.sin(0.45*t)))