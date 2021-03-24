#include "SimpleControl.hpp"
#include "engine/core/logger.hpp"
#include <cmath>

struct vector3d{
  float x;
  float y;
  float z;
};

struct speed{
  float linear_speed;
  float angular_speed;
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

namespace isaac{

  Vector3dProto::Reader target_proto;
  RigidBody3Proto::Reader rigid_proto;
  float angular_velocity;
  struct vector3d local;
  struct quat q;
  struct euler euler_angles;
  struct speed result;
  float y_rotation;
  float tan;
  float theta;
  float theta_deg;



  void SimpleControl::start(){
        tickOnMessage(rx_incoming_target());
        tickOnMessage(rx_incoming_bodies());
  }

  void SimpleControl::tick(){
    bool is_target_arrived = rx_incoming_target().available();
    bool is_bodies_arrived = rx_incoming_bodies().available();
    if(is_target_arrived){
      target_proto = rx_incoming_target().getProto();

    }
    if(is_bodies_arrived){
      rigid_proto = rx_incoming_bodies().getProto();
    }

    q.x = rigid_proto.getRefTBody().getRotation().getQ().getX();
    q.y = rigid_proto.getRefTBody().getRotation().getQ().getY();
    q.z = rigid_proto.getRefTBody().getRotation().getQ().getZ();
    q.w = rigid_proto.getRefTBody().getRotation().getQ().getW();
    euler_angles = ToEulerAngles(q);
    y_rotation = euler_angles.yaw * (180/M_PI);


    local.x = target_proto.getX() - rigid_proto.getRefTBody().getTranslation().getX();
    //local.y = target_proto.getY() - rigid_proto.getRefTBody().getTranslation().getY();
    local.z = target_proto.getZ() - rigid_proto.getRefTBody().getTranslation().getY();


    angular_velocity = sqrt(rigid_proto.getAngularVelocity().getX()*rigid_proto.getAngularVelocity().getX() + rigid_proto.getAngularVelocity().getY()*rigid_proto.getAngularVelocity().getY() + rigid_proto.getAngularVelocity().getZ()*rigid_proto.getAngularVelocity().getZ());

    tan = local.z/local.x;
    theta = atan(tan);
    theta_deg = theta*(180/M_PI);
    LOG_INFO("local x : %lf, local z : %lf, tan : %lf, theta : %lf",local.x, local.z, tan, theta);

    if(local.z>0){
      if(local.x>0){
            if(y_rotation < theta_deg){
              result.linear_speed = 0.0;
              result.angular_speed = 0.3;
            }
            else if(y_rotation > theta_deg + 5) {
              result.linear_speed = 0.0;
              result.angular_speed = -0.3;
            }
            else{
              if(angular_velocity < 0.1){
                result.linear_speed = 3.0;
                result.angular_speed = 0.0;
              }
              else{
                result.linear_speed = 0.0;
                result.angular_speed = 0.0;
              }
            }
          }
          else{
            if(y_rotation < 180-theta_deg*(-1)){
              result.linear_speed = 0.0;
              result.angular_speed = 0.3;
            }
            else if(y_rotation > 180-theta_deg*(-1)+5){
              result.linear_speed = 0.0;
              result.angular_speed = -0.3;
            }
            else{
              if(angular_velocity < 0.1){
                result.linear_speed = 3.0;
                result.angular_speed = 0.0;
              }
              else{
                result.linear_speed = 0.0;
                result.angular_speed = 0.0;
              }
            }
          }
    }
    else{
      if(local.x>0){
            if(y_rotation > theta_deg){
              result.linear_speed = 0.0;
              result.angular_speed = -0.3;
            }
            else if (y_rotation < theta_deg -5){
              result.linear_speed = 0.0;
              result.angular_speed = 0.3;
            }
            else{
              if(angular_velocity < 0.1){
                result.linear_speed = 3.0;
                result.angular_speed = 0.0;
              }
              else{
                result.linear_speed = 0.0;
                result.angular_speed = 0.0;
              }
            }
          }
          else{
            if(y_rotation > -180+theta_deg){
              result.linear_speed = 0.0;
              result.angular_speed = -0.3;
            }
            else if (y_rotation < -180+theta_deg-5){
              result.linear_speed = 0.0;
              result.angular_speed = 0.3;
            }
            else{
              if(angular_velocity < 0.1){
                result.linear_speed = 3.0;
                result.angular_speed = 0.0;
              }
              else{
                result.linear_speed = 0.0;
                result.angular_speed = 0.0;
              }
            }
          }
    }
    //Publish
    auto speed_proto = tx_outgoing_speed().initProto();
    speed_proto.setX(result.linear_speed);
    speed_proto.setY(result.angular_speed);
    tx_outgoing_speed().publish();
    LOG_INFO("speed (%lf, %lf) is published to main node", result.linear_speed, result.angular_speed);
  }
}
