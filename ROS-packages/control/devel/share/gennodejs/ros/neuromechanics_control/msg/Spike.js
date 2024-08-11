// Auto-generated. Do not edit!

// (in-package neuromechanics_control.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Spike {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.neuron_index = null;
      this.time = null;
    }
    else {
      if (initObj.hasOwnProperty('neuron_index')) {
        this.neuron_index = initObj.neuron_index
      }
      else {
        this.neuron_index = 0;
      }
      if (initObj.hasOwnProperty('time')) {
        this.time = initObj.time
      }
      else {
        this.time = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Spike
    // Serialize message field [neuron_index]
    bufferOffset = _serializer.uint32(obj.neuron_index, buffer, bufferOffset);
    // Serialize message field [time]
    bufferOffset = _serializer.float64(obj.time, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Spike
    let len;
    let data = new Spike(null);
    // Deserialize message field [neuron_index]
    data.neuron_index = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [time]
    data.time = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'neuromechanics_control/Spike';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ffb331888bb7ffb461985148bcf50fd8';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint32 neuron_index
    float64 time
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Spike(null);
    if (msg.neuron_index !== undefined) {
      resolved.neuron_index = msg.neuron_index;
    }
    else {
      resolved.neuron_index = 0
    }

    if (msg.time !== undefined) {
      resolved.time = msg.time;
    }
    else {
      resolved.time = 0.0
    }

    return resolved;
    }
};

module.exports = Spike;
