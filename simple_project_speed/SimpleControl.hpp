#pragma once

#include "engine/alice/alice_codelet.hpp"
#include "messages/math.capnp.h"
#include "messages/rigid_body_3_group.capnp.h"

namespace isaac {
    class SimpleControl : public alice::Codelet {
        public:
            void start() override;
            void tick() override;

        ISAAC_PROTO_RX(Vector3dProto, incoming_target);
        ISAAC_PROTO_RX(RigidBody3Proto, incoming_bodies);  
        ISAAC_PROTO_TX(Vector2dProto, outgoing_speed);
    };


}

ISAAC_ALICE_REGISTER_CODELET(isaac::SimpleControl);
