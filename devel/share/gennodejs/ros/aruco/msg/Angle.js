// Auto-generated. Do not edit!

// (in-package aruco.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Angle {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.angle_base = null;
      this.angle_aruco = null;
      this.angle_cam = null;
    }
    else {
      if (initObj.hasOwnProperty('angle_base')) {
        this.angle_base = initObj.angle_base
      }
      else {
        this.angle_base = 0.0;
      }
      if (initObj.hasOwnProperty('angle_aruco')) {
        this.angle_aruco = initObj.angle_aruco
      }
      else {
        this.angle_aruco = 0.0;
      }
      if (initObj.hasOwnProperty('angle_cam')) {
        this.angle_cam = initObj.angle_cam
      }
      else {
        this.angle_cam = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Angle
    // Serialize message field [angle_base]
    bufferOffset = _serializer.float32(obj.angle_base, buffer, bufferOffset);
    // Serialize message field [angle_aruco]
    bufferOffset = _serializer.float32(obj.angle_aruco, buffer, bufferOffset);
    // Serialize message field [angle_cam]
    bufferOffset = _serializer.float32(obj.angle_cam, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Angle
    let len;
    let data = new Angle(null);
    // Deserialize message field [angle_base]
    data.angle_base = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [angle_aruco]
    data.angle_aruco = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [angle_cam]
    data.angle_cam = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'aruco/Angle';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd2b3e9a194f5925c37a312cac19f86be';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    float32 angle_base
    float32 angle_aruco
    float32 angle_cam
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Angle(null);
    if (msg.angle_base !== undefined) {
      resolved.angle_base = msg.angle_base;
    }
    else {
      resolved.angle_base = 0.0
    }

    if (msg.angle_aruco !== undefined) {
      resolved.angle_aruco = msg.angle_aruco;
    }
    else {
      resolved.angle_aruco = 0.0
    }

    if (msg.angle_cam !== undefined) {
      resolved.angle_cam = msg.angle_cam;
    }
    else {
      resolved.angle_cam = 0.0
    }

    return resolved;
    }
};

module.exports = Angle;
