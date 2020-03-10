#ifndef ROSBT_BLACKBOARD_H_
#define ROSBT_BLACKBOARD_H_
#include <behaviortree_cpp_v3/blackboard.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <typeinfo>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <behaviortree_cpp_v3/basic_types.h>


using namespace BT;



/*The purpose of this class is to avoid the creation of
duplicate ROS elements (publishers, subscribers,
service clients, actionclients). This way, the same ROS element
can be shared among all tree nodes.*/

class RosBlackBoard
{
    public:

        static BT::TimePoint Now(){
        return std::chrono::high_resolution_clock::now();
        }

        static BT::Any* getAnyPublisher(std::string& key)
        {
            return _publishers_bb->getAny(key);
        }
        static BT::Any* getAnySubscriber(std::string& key)
        {
            return _subscribers_bb->getAny(key);
        }
        static BT::Any* getAnyCallbackQueue(std::string& key)
        {
            return _subs_callback_queue_bb->getAny(key);
        }
        static BT::Any* getAnyServiceClient(std::string& key)
        {
            return _service_clients_bb->getAny(key);
        }
        static BT::Any* getAnyActionClient(std::string& key)
        {
            return _actionclients_bb->getAny(key);
        }

        /*Returning a raw pointers. Reasons: creating shared_ptrs is expensive and
        objects on each blackboard will not be replaced. It does not make sense
        to have two different types for the same topic/action/service name. An assert
        will stop execution if that happens.
        I'm using shared_ptr instead of unique_ptr because "Any" requires the object to be copiable.*/

        template <typename T>
        static ros::Publisher* getPublisherOrInit(const std::string& topic, bool latch=true)
        {
            auto anyPublisher = _publishers_bb->getAny(topic);

            /*Publisher doesnt exist yet. Create it. */
            if(!anyPublisher || anyPublisher->empty())
            {
                //It creates a nh for now... Not sure how to solve this, since ros_init needs to be called first
                ros::NodeHandle nh;

                std::shared_ptr<ros::Publisher> publisher_PTR = std::make_shared<ros::Publisher>
                    (std::move(nh.advertise<T>(topic, 1, latch)));

                ros::Publisher *publisherRawPTR = publisher_PTR.get();
                _publishers_bb->set(topic, std::move(publisher_PTR));
                return publisherRawPTR;
            }else{
                /*Check type is ok before returning*/
                const std::type_info& info = typeid(std::shared_ptr<ros::Publisher>);
                const std::type_info& gotType = anyPublisher->type();

                assert(info.hash_code() == gotType.hash_code());
                /*Type is ok, so we can return the pointer. It would be odd to have
                another thing than a publisher here, right?...*/

                return anyPublisher->cast<std::shared_ptr<ros::Publisher>>().get();
            }
        }

        template <typename T>
        static T* getSubscriberOrInit(const std::string& topic)
        {

        }

        template <typename T>
        static T* getCallbackQueueOrInit(const std::string& topic)
        {

        }

        template <typename T>
        static T* getServiceClientOrInit(const std::string& service)
        {

        }

        template <typename T>
        static T* getActionClientOrInit(const std::string& action, CoroActionNode *btnode)
        {
            auto anyClient = _actionclients_bb->getAny(action);
            T* clientRawPTR;

            /*Client doesnt exist yet. Create it. You should manage if the action server is ready from
            each BT node.*/
            if(!anyClient || anyClient->empty())
            {
                std::shared_ptr<T> client = std::make_shared<T>(action);
                clientRawPTR = client.get();
                _actionclients_bb->set(action, std::move(client));
            }else{
                /*Check type is ok before returning*/
                const std::type_info& info = typeid(std::shared_ptr<T>);
                const std::type_info& gotType = anyClient->type();

                assert(info.hash_code() == gotType.hash_code());
                /*Type is ok, so we can return the pointer*/

                clientRawPTR = anyClient->cast<std::shared_ptr<T> >().get();
            }


            ROS_INFO_STREAM("Waiting for action server of: " << action); 

            TimePoint init_time = Now();
            TimePoint timeout_time = Now()+std::chrono::milliseconds(5000);

            while(!clientRawPTR->isServerConnected())
            {
                if(Now() > timeout_time)
                {
                    ROS_WARN_STREAM("Could not connect to action server: " << action);
                    return NULL;
                }

            btnode->setStatusRunningAndYield();
            }

            ROS_INFO_STREAM("Found action server of: " << action);

            return clientRawPTR;
        }

    private:
        static Blackboard::Ptr _publishers_bb;
        static Blackboard::Ptr _subscribers_bb;
        static Blackboard::Ptr _subs_callback_queue_bb;
        static Blackboard::Ptr _service_clients_bb;
        static Blackboard::Ptr _actionclients_bb;


};


#endif //ROSBT_BLACKBOARD_H_