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

class Spike_group {
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
        this.neuron_index = [];
      }
      if (initObj.hasOwnProperty('time')) {
        this.time = initObj.time
      }
      else {
        this.time = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Spike_group
    // Serialize message field [neuron_index]
    bufferOffset = _arraySerializer.uint32(obj.neuron_index, buffer, bufferOffset, null);
    // Serialize message field [time]
    bufferOffset = _arraySerializer.float64(obj.time, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Spike_group
    let len;
    let data = new Spike_group(null);
    // Deserialize message field [neuron_index]
    data.neuron_index = _arrayDeserializer.uint32(buffer, bufferOffset, null)
    // Deserialize message field [time]
    data.time = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.neuron_index.length;
    length += 8 * object.time.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'neuromechanics_control/Spike_group';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '981939a1e507ff75c61993d24854ca72';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint32[] neuron_index
    float64[] time
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Spike_group(null);
    if (msg.neuron_index !== undefined) {
      resolved.neuron_index = msg.neuron_index;
    }
    else {
      resolved.neuron_index = []
    }

    if (msg.time !== undefined) {
      resolved.time = msg.time;
    }
    else {
      resolved.time = []
    }

    return resolved;
    }
};

module.exports = Spike_group;
