#include "SimpleApp.hpp"

#include "engine/core/logger.hpp"

namespace isaac
{
    void SimpleApp::start() {
        tickOnMessage(rx_incoming_target());
        tickOnMessage(rx_incoming_speed());
        tickOnMessage(rx_incoming_bodies());
    }

    void SimpleApp::tick() {
        bool is_target_arrived = rx_incoming_target().available();
        bool is_speed_arrived = rx_incoming_speed().available();
        bool is_bodies_arrived = rx_incoming_bodies().available();
        if(is_target_arrived){
            auto incoming_target_proto = rx_incoming_target().getProto();

            auto outgoing_target_proto = tx_outgoing_target().initProto();
            outgoing_target_proto.setX(incoming_target_proto.getX());
            outgoing_target_proto.setY(incoming_target_proto.getY());
            outgoing_target_proto.setZ(incoming_target_proto.getZ());
            tx_outgoing_target().publish();
            LOG_INFO("pose is published to control node");
        }
        if(is_speed_arrived){
            auto incoming_speed_proto = rx_incoming_speed().getProto();
            LOG_INFO("main node received speed (%lf, %lf)", incoming_speed_proto.getX(), incoming_speed_proto.getY());
            auto outgoing_speed_proto = tx_outgoing_speed().initProto();
            outgoing_speed_proto.setX(incoming_speed_proto.getX()); // linear speed
            outgoing_speed_proto.setY(incoming_speed_proto.getY()); // angular speed
            LOG_INFO("main node published speed (%lf, %lf)", outgoing_speed_proto.getX(), outgoing_speed_proto.getY());
            tx_outgoing_speed().publish();
        }

        if(is_bodies_arrived){
            auto incoming_bodies_proto = rx_incoming_bodies().getProto();
            auto outgoing_bodies_proto = tx_outgoing_bodies().initProto();
            outgoing_bodies_proto.setAngularVelocity(incoming_bodies_proto.getBodies().begin()->getAngularVelocity());
            outgoing_bodies_proto.setRefTBody(incoming_bodies_proto.getBodies().begin()->getRefTBody());
            tx_outgoing_bodies().publish();
            LOG_INFO("bodies are published to control node");
        }
    }
} // namespace isaac
