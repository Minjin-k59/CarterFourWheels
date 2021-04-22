#include "SimpleApp.hpp"

#include "engine/core/logger.hpp"

namespace isaac
{
    void SimpleApp::start() {
        tickOnMessage(rx_incoming_target());
        tickOnMessage(rx_incoming_torque());
        tickOnMessage(rx_incoming_bodies());
    }

    void SimpleApp::tick() {
        bool is_target_arrived = rx_incoming_target().available();
        bool is_torque_arrived = rx_incoming_torque().available();
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

        if(is_bodies_arrived){
            auto incoming_bodies_proto = rx_incoming_bodies().getProto();
            auto outgoing_bodies_proto = tx_outgoing_bodies().initProto();
            outgoing_bodies_proto.setAngularVelocity(incoming_bodies_proto.getAngularVelocity());
            outgoing_bodies_proto.setRefTBody(incoming_bodies_proto.getRefTBody());
            tx_outgoing_bodies().publish();
            LOG_INFO("bodies are published to control node");
        }
        if(is_torque_arrived){
            auto incoming_torque_proto = rx_incoming_torque().getProto();
            LOG_INFO("main node received speed (%lf, %lf)", incoming_torque_proto.getX(), incoming_torque_proto.getY());
            auto outgoing_torque_proto = tx_outgoing_torque().initProto();
            outgoing_torque_proto.setX(incoming_torque_proto.getX()); // linear speed x : right
            outgoing_torque_proto.setY(incoming_torque_proto.getY()); // linear speed y : forward
            outgoing_torque_proto.setZ(incoming_torque_proto.getZ()); // angular speed
            outgoing_torque_proto.setW(incoming_torque_proto.getW());
            LOG_INFO("main node published speed (%lf, %lf)", outgoing_torque_proto.getX(), outgoing_torque_proto.getY());
            tx_outgoing_torque().publish();
        }
    }
} // namespace isaac
