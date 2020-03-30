/*
  -----------------------------------------------------
  A template to create behavior tree actions to publish to ROS topics.
  Don't forget to replace PUBLISHER_TEMPLATE_BT with the name of your
  action!
*/

#include <YOURPACKAGE/publisher_template.hpp>


std::map<std::string, ros::Publisher> PUBLISHER_TEMPLATE_BT::_publishers;

PUBLISHER_TEMPLATE_BT::PUBLISHER_TEMPLATE_BT(const std::string& name, const NodeConfiguration& config)
    : SyncActionNode(name, config), nh_()/*and other initializations...*/
{

    /*Some construction stuff*/
    /* --- Example --- */ 
    /*last_time = RosBlackBoard::Now();*/

}

BT::NodeStatus PUBLISHER_TEMPLATE_BT::tick()
{

    BT::Optional<std::string> topic = getInput<std::string>("topic");

    /*Get input from other input ports*/
    /*--- Examples --- */
    /*BT::Optional<double> angle = getInput<double>("angle"); */


    /*Check if inputs are set*/
    if(!topic)
    {
        throw BT::RuntimeError("missing required inputs [topic]: ",
                                   topic.error() );
    }
    
    /*if (!angle)
    {
        throw BT::RuntimeError("missing required inputs [torso_angle (deg)]: ",
                                   angle.error() );
    }*/


    /*More code if you need it*/


    auto publisher_pair = _publishers.find(topic.value());

    if(publisher_pair == _publishers.end())
    {
        _publishers[topic.value()] = nh_.advertise<YOUR_TOPIC_MESSAGE_TYPE>(topic.value(), 1, true);
        ROS_INFO_STREAM("Created publisher to topic: " << topic.value());
    }

    publisher_pair = _publishers.find(topic.value());
    ros::Publisher &pub = publisher_pair->second;


    YOUR_TOPIC_MESSAGE_TYPE command;

    /*Set it here with some fancy code...*/
    /* --- Example --- */
    /*command.data = ang_val;*/

    pub.publish(command);

    if(pub.getNumSubscribers() < 1)
    {
        return BT::NodeStatus::FAILURE;
    }else{
        return BT::NodeStatus::SUCCESS;
    }

}