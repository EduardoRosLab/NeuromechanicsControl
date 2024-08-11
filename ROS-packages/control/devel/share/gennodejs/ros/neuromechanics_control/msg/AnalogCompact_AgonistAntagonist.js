// Auto-generated. Do not edit!

// (in-package neuromechanics_control.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class AnalogCompact_AgonistAntagonist {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.agonist = null;
      this.antagonist = null;
      this.names = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('agonist')) {
        this.agonist = initObj.agonist
      }
      else {
        this.agonist = [];
      }
      if (initObj.hasOwnProperty('antagonist')) {
        this.antagonist = initObj.antagonist
      }
      else {
        this.antagonist = [];
      }
      if (initObj.hasOwnProperty('names')) {
        this.names = initObj.names
      }
      else {
        this.names = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type AnalogCompact_AgonistAntagonist
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [agonist]
    bufferOffset = _arraySerializer.float64(obj.agonist, buffer, bufferOffset, null);
    // Serialize message field [antagonist]
    bufferOffset = _arraySerializer.float64(obj.antagonist, buffer, bufferOffset, null);
    // Serialize message field [names]
    bufferOffset = _arraySerializer.string(obj.names, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type AnalogCompact_AgonistAntagonist
    let len;
    let data = new AnalogCompact_AgonistAntagonist(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [agonist]
    data.agonist = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [antagonist]
    data.antagonist = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [names]
    data.names = _arrayDeserializer.string(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 8 * object.agonist.length;
    length += 8 * object.antagonist.length;
    object.names.forEach((val) => {
      length += 4 + val.length;
    });
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'neuromechanics_control/AnalogCompact_AgonistAntagonist';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'cbcf5b90b78f051386b46f54c96ce043';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    float64[] agonist
    float64[] antagonist
    string[]  names
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    # 0: no frame
    # 1: global frame
    string frame_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new AnalogCompact_AgonistAntagonist(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.agonist !== undefined) {
      resolved.agonist = msg.agonist;
    }
    else {
      resolved.agonist = []
    }

    if (msg.antagonist !== undefined) {
      resolved.antagonist = msg.antagonist;
    }
    else {
      resolved.antagonist = []
    }

    if (msg.names !== undefined) {
      resolved.names = msg.names;
    }
    else {
      resolved.names = []
    }

    return resolved;
    }
};

module.exports = AnalogCompact_AgonistAntagonist;
