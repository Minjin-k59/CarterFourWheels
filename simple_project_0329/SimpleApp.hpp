#pragma once

#include <string>
#include <vector>

#include "engine/alice/alice_codelet.hpp"
#include "messages/math.capnp.h"
#include "messages/alice.capnp.h"
#include "messages/rigid_body_3_group.capnp.h"

namespace isaac {
    class SimpleApp : public alice::Codelet {
        public:
            void start() override;
            void tick() override;

            ISAAC_PROTO_RX(Vector3dProto, incoming_target);
            ISAAC_PROTO_TX(Vector3dProto, outgoing_target);

            ISAAC_PROTO_RX(RigidBody3Proto, incoming_bodies);
            ISAAC_PROTO_TX(RigidBody3Proto, outgoing_bodies);  

            ISAAC_PROTO_RX(Vector4dProto, incoming_torque);
            ISAAC_PROTO_TX(Vector4dProto, outgoing_torque);
    };

}

ISAAC_ALICE_REGISTER_CODELET(isaac::SimpleApp);
