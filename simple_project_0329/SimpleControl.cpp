
#include "SimpleControl.hpp"
#include "engine/core/logger.hpp"
#include <cmath>

struct vector3d{
  float x;
  float y;
  float z;
};
struct speed{
  double v;
  double vn;
  double w;
};
struct torque{
  double fr;
  double fl;
  double bl;
  double br;
};
struct quat{
  float w, x, y, z;
};
struct euler{
  float roll, pitch, yaw;
};
euler ToEulerAngles(quat q) {
    euler angles;
    // roll (x-axis rotation)
    float sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    float cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);
    // pitch (y-axis rotation)
    float sinp = 2 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1)
        angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = std::asin(sinp);
    // yaw (z-axis rotation)
    float siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    float cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);
    return angles;
}
float maxMotorTorque = 30.0f;
float proportionalGain = 100.0f;
float wheelBase = 1.3f;
struct torque desiredSpeed;

float clamp(float x, float min, float Max){
  if(x > Max){
    return Max;
  }
  else if (x<min){
    return min;
  }
  else {
    return x;
  }
}

float proportionalController(float current, float desired) {
    float result = clamp((desired - current) * proportionalGain, -maxMotorTorque, maxMotorTorque);
    return result;
}

float GetTorque(float current, float desired) {
    return proportionalController(current, desired);
}


void GetDesiredSpeed(double linearXSpeed, double linearYSpeed, double angularSpeed){ //didn't include brakeRequested line
  desiredSpeed.fr = (float)(linearYSpeed + wheelBase * angularSpeed);
  desiredSpeed.fl = (float)(linearXSpeed - wheelBase * angularSpeed);
  desiredSpeed.bl = (float)(linearYSpeed - wheelBase * angularSpeed);
  desiredSpeed.br = (float)(linearXSpeed + wheelBase * angularSpeed);
}


namespace isaac{
  Vector3dProto::Reader target_proto;
  RigidBody3Proto::Reader rigid_proto;
  Vector4dProto::Reader current_speed_proto;
  float angular_velocity;
  struct vector3d local;
  struct vector3d trans;
  struct quat q;
  struct euler euler_angles;
  struct speed speed_result;
  struct speed trans_result;
  struct torque torque_result;
  struct torque currentSpeed;
  //struct torque desiredSpeed;
  float y_rotation;
  float tan;
  float theta;
  float theta_deg;
  float r_body;
  float r_wheel;
  float mass;
  void SimpleControl::start(){
        tickOnMessage(rx_incoming_target());
        tickOnMessage(rx_incoming_bodies());
        tickOnMessage(rx_incoming_current_speed());
        r_body = 0.42426406871; //r_body of carter wheeled
        r_wheel = 0.25;
        mass = 50;
        desiredSpeed.fr = 0;
        desiredSpeed.fl = 0;
        desiredSpeed.bl = 0;
        desiredSpeed.br = 0;
  }
  void SimpleControl::tick(){
    bool is_target_arrived = rx_incoming_target().available();
    bool is_bodies_arrived = rx_incoming_bodies().available();
    bool is_current_speed_arrived = rx_incoming_current_speed().available();

    if(is_target_arrived){
      target_proto = rx_incoming_target().getProto();
    }
    if(is_bodies_arrived){
      rigid_proto = rx_incoming_bodies().getProto();
    }
    if(is_current_speed_arrived){
      current_speed_proto = rx_incoming_current_speed().getProto();
    }
    q.x = rigid_proto.getRefTBody().getRotation().getQ().getX();
    q.y = rigid_proto.getRefTBody().getRotation().getQ().getY();
    q.z = rigid_proto.getRefTBody().getRotation().getQ().getZ();
    q.w = rigid_proto.getRefTBody().getRotation().getQ().getW();
    euler_angles = ToEulerAngles(q);
    y_rotation = euler_angles.yaw * (180/M_PI);
    local.x = target_proto.getX() - rigid_proto.getRefTBody().getTranslation().getX();
    local.z = target_proto.getZ() - rigid_proto.getRefTBody().getTranslation().getY();
    angular_velocity = sqrt(rigid_proto.getAngularVelocity().getX()*rigid_proto.getAngularVelocity().getX() + rigid_proto.getAngularVelocity().getY()*rigid_proto.getAngularVelocity().getY() + rigid_proto.getAngularVelocity().getZ()*rigid_proto.getAngularVelocity().getZ());
    theta = atan2(local.z, local.x);
    theta_deg = theta*(180/M_PI);
    LOG_INFO("local x : %lf, local z : %lf, tan : %lf, theta_deg : %lf",local.x, local.z, tan, theta_deg);
    //coordinate transformation && linear velocity
    //trans.x = (1/sqrt(2))*local.x + (-1/sqrt(2))*local.z;
    //trans.z = (1/sqrt(2))*local.x + (1/sqrt(2)*local.z);
    trans.x = local.x;
    trans.z = local.z;
    trans.y = sqrt((trans.x*trans.x) + (trans.z*trans.z)); // stored the length of vector in trans.y for normalizing.
    speed_result.v = -trans.z/trans.y;
    speed_result.vn = trans.x/trans.y;
    //angular velocity
    if(local.z >0){
      if(y_rotation < theta_deg-5) speed_result.w = 1.0;
      else if (y_rotation > theta_deg +5) speed_result.w = -1.0;
      else speed_result.w = 0.0;
    }
    else{
      if(y_rotation > theta_deg-5) speed_result.w = -1.0;
      else if (y_rotation < theta_deg -5) speed_result.w = 1.0;
      else speed_result.w = 0.0;
    }
    trans_result.v = (1/sqrt(2))*speed_result.v + (1/sqrt(2))*speed_result.vn;
    trans_result.vn = (-1/sqrt(2))*speed_result.v + (1/sqrt(2)*speed_result.vn);
    trans_result.w = speed_result.w;
    //trans_result.w =0;


    LOG_INFO("y_rotation : %f, theta_deg : %f", y_rotation, theta_deg);

    // velocity to torque
    currentSpeed.fr = current_speed_proto.getX();
    currentSpeed.fl = current_speed_proto.getY();
    currentSpeed.bl = current_speed_proto.getZ();
    currentSpeed.br = current_speed_proto.getW();
    GetDesiredSpeed(trans_result.v*30, trans_result.vn*30, trans_result.w*8);

    //Publish
    //result.v = 1.0;
    //result.vn = 0.0;
    //result.w = 0.0;//rf, rl , bl, br
    //speed_result.v = 0.0;
    //speed_result.vn = 0.0;
    //speed_result.w = 0.0;
    LOG_INFO("speed) v : %f, vn : %f, w : %f",speed_result.v, speed_result.vn, speed_result.w);
    LOG_INFO("trans) v : %f, vn : %f, w : %f",trans_result.v, trans_result.vn, trans_result.w);
    //LOG_INFO("mass: %f, r_wheel: %f, r_body : %f", mass, r_wheel, r_body);
    
    /*
    torque_result.fr = (1.0/4.0)*(mass * r_wheel)*(sqrt(2)*(-speed_result.v + speed_result.vn)+r_body*speed_result.w);
    torque_result.fl = (1.0/4.0)*(mass * r_wheel)*(sqrt(2)*(speed_result.v + speed_result.vn)-r_body*speed_result.w);
    torque_result.bl = (1.0/4.0)*(mass * r_wheel)*(sqrt(2)*(-speed_result.v + speed_result.vn)-r_body*speed_result.w);
    torque_result.br = (1.0/4.0)*(mass * r_wheel)*(sqrt(2)*(speed_result.v + speed_result.vn)+r_body*speed_result.w);
    
    torque_result.fr = 3.0f;
    torque_result.fl = -3.0f;
    torque_result.bl = -3.0f;
    torque_result.br = 3.0f;
    */
    torque_result.fr = GetTorque(currentSpeed.fr, desiredSpeed.fr);
    torque_result.fl = GetTorque(currentSpeed.fl, desiredSpeed.fl);
    torque_result.bl = GetTorque(currentSpeed.bl, desiredSpeed.bl);
    torque_result.br = GetTorque(currentSpeed.br, desiredSpeed.br);
    auto torque_proto = tx_outgoing_torque().initProto();
    torque_proto.setX(torque_result.fr);
    torque_proto.setY(torque_result.fl);
    torque_proto.setZ(torque_result.bl);
    torque_proto.setW(torque_result.br);
    tx_outgoing_torque().publish();
    LOG_INFO("current (%lf, %lf, %lf, %lf) is published to main node", currentSpeed.fr, currentSpeed.fl, currentSpeed.bl, currentSpeed.br);
    LOG_INFO("desired (%lf, %lf, %lf, %lf) is published to main node", desiredSpeed.fr, desiredSpeed.fl, desiredSpeed.bl, desiredSpeed.br);
    LOG_INFO("torque (%lf, %lf, %lf, %lf) is published to main node", torque_result.fr, torque_result.fl, torque_result.bl, torque_result.br);
  }
}