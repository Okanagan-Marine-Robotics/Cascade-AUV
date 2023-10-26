#include <string>
#include <gz/common/Console.hh>
#include <gz/sim/Link.hh>
#include <gz/msgs.hh>
#include <gz/transport.hh>
#include "gz/sim/Util.hh"
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>

#include "movePlugin.hpp"

using namespace move_Plugin;
using namespace gz;
using namespace sim;
using namespace math;

gz::transport::Node node;
std::string topic = "/box/cmd_vel";
gz::msgs::Twist msg;

void movePlugin::onTopicReceive(const gz::msgs::Twist &temp_msg){
    msg=gz::msgs::Twist(temp_msg);
}

void movePlugin::PreUpdate(const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm){
    if(!gotBox){
        std::unordered_set<gz::sim::v7::Entity> set=gz::sim::entitiesFromScopedName("box",_ecm);	
        for (const auto& elem : set)
            if(_ecm.HasEntity(elem)){
                gzmsg << "Got Entity Box!"<<'\n';
                box=Link(elem);
                if (!node.Subscribe(topic, onTopicReceive)){
                    gzmsg << "Error subscribing to topic [" << topic << "]" << std::endl;
                }
                gotBox=true;
            }
    }
    else{
        //change to only apply velocity if constantly being published,
        //maybe record time of last subscription callback and reset velocity to 0 after 100ms or so
        box.SetLinearVelocity(_ecm,Vector3(msg.linear().x(),msg.linear().y(),msg.linear().z()));
        box.SetAngularVelocity(_ecm,Vector3(msg.angular().x(),msg.angular().y(),msg.angular().z()));
    }
}

void movePlugin::PostUpdate(const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm){
        //gzmsg << (int)_ecm.EntityCount() << std::endl;
}

GZ_ADD_PLUGIN(
    move_Plugin::movePlugin,
    gz::sim::System,
    move_Plugin::movePlugin::ISystemPreUpdate,
    move_Plugin::movePlugin::ISystemPostUpdate)

