// Auto-generated. Do not edit!

// (in-package naoqi_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class AudioCustomMsg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.rearLeft = null;
      this.rearRight = null;
      this.frontLeft = null;
      this.frontRight = null;
    }
    else {
      if (initObj.hasOwnProperty('rearLeft')) {
        this.rearLeft = initObj.rearLeft
      }
      else {
        this.rearLeft = [];
      }
      if (initObj.hasOwnProperty('rearRight')) {
        this.rearRight = initObj.rearRight
      }
      else {
        this.rearRight = [];
      }
      if (initObj.hasOwnProperty('frontLeft')) {
        this.frontLeft = initObj.frontLeft
      }
      else {
        this.frontLeft = [];
      }
      if (initObj.hasOwnProperty('frontRight')) {
        this.frontRight = initObj.frontRight
      }
      else {
        this.frontRight = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type AudioCustomMsg
    // Serialize message field [rearLeft]
    bufferOffset = _arraySerializer.int16(obj.rearLeft, buffer, bufferOffset, null);
    // Serialize message field [rearRight]
    bufferOffset = _arraySerializer.int16(obj.rearRight, buffer, bufferOffset, null);
    // Serialize message field [frontLeft]
    bufferOffset = _arraySerializer.int16(obj.frontLeft, buffer, bufferOffset, null);
    // Serialize message field [frontRight]
    bufferOffset = _arraySerializer.int16(obj.frontRight, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type AudioCustomMsg
    let len;
    let data = new AudioCustomMsg(null);
    // Deserialize message field [rearLeft]
    data.rearLeft = _arrayDeserializer.int16(buffer, bufferOffset, null)
    // Deserialize message field [rearRight]
    data.rearRight = _arrayDeserializer.int16(buffer, bufferOffset, null)
    // Deserialize message field [frontLeft]
    data.frontLeft = _arrayDeserializer.int16(buffer, bufferOffset, null)
    // Deserialize message field [frontRight]
    data.frontRight = _arrayDeserializer.int16(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 2 * object.rearLeft.length;
    length += 2 * object.rearRight.length;
    length += 2 * object.frontLeft.length;
    length += 2 * object.frontRight.length;
    return length + 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'naoqi_driver/AudioCustomMsg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'bcac904b0030b6d70386338d161f4882';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # AudioCustomMsg.msg
    int16[] rearLeft
    int16[] rearRight
    int16[] frontLeft
    int16[] frontRight
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new AudioCustomMsg(null);
    if (msg.rearLeft !== undefined) {
      resolved.rearLeft = msg.rearLeft;
    }
    else {
      resolved.rearLeft = []
    }

    if (msg.rearRight !== undefined) {
      resolved.rearRight = msg.rearRight;
    }
    else {
      resolved.rearRight = []
    }

    if (msg.frontLeft !== undefined) {
      resolved.frontLeft = msg.frontLeft;
    }
    else {
      resolved.frontLeft = []
    }

    if (msg.frontRight !== undefined) {
      resolved.frontRight = msg.frontRight;
    }
    else {
      resolved.frontRight = []
    }

    return resolved;
    }
};

module.exports = AudioCustomMsg;
