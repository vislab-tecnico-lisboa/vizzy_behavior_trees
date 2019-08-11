#ifndef ROSBT_BLACKBOARD_H_
#define ROSBT_BLACKBOARD_H_
#include <behaviortree_cpp/blackboard.h>
#include <typeinfo>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <behaviortree_cpp/basic_types.h>


using namespace BT;


/*The purpose of this class is to avoid the creation of
duplicate ROS elements (publishers, subscribers,
service clients, actionclients). This way, the same ROS element
can be shared among all tree nodes.*/

class RosBlackBoard
{
    public:
    
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

        /*Returning a raw pointer. Reasons: creating shared_ptrs is expensive and
        objects on each blackboard will not be replaced. It does not make sense
        to have two different types for the same topic/action/service name. An assert
        will stop execution if that happens.
        I'm using shared_ptr because "Any" requires the object to be copiable.*/

        template <typename T>
        static T* getPublisherOrInit(const std::string& topic)
        {

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
        static T* getActionClientOrInit(const std::string& action)
        {
            std::cout << "76" << std::endl;
            auto anyClient = _actionclients_bb->getAny(action);
            std::cout << "78" << std::endl;

            /*Client doesnt exist yet. Create it. You should manage if the action server is ready from
            each BT node.*/
            if(!anyClient)
            {
            std::cout << "84" << std::endl;
                std::shared_ptr<T> client = std::make_shared<T>(action);
                T* clientRawPTR = client.get();
                _actionclients_bb->set(action, std::move(client));
            std::cout << "88" << std::endl;
                return clientRawPTR;

            }else{
                /*Check type is ok before returning*/
            std::cout << "93" << std::endl;
                const std::type_info& info = typeid(std::shared_ptr<T>);
                const std::type_info& gotType = anyClient->type();

            std::cout << "97" << std::endl;
                assert(info.hash_code() == gotType.hash_code());
                /*Type is ok, so we can return the pointer*/

                return anyClient->cast<std::shared_ptr<T> >().get();
            }
        }

    private:
        static Blackboard::Ptr _publishers_bb;
        static Blackboard::Ptr _subscribers_bb;
        static Blackboard::Ptr _subs_callback_queue_bb;
        static Blackboard::Ptr _service_clients_bb;
        static Blackboard::Ptr _actionclients_bb;


};


#endif //ROSBT_BLACKBOARD_H_