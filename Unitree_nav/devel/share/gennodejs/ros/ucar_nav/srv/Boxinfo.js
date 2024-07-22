// Auto-generated. Do not edit!

// (in-package ucar_nav.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------


//-----------------------------------------------------------

class BoxinfoRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.box_x = null;
      this.box_y = null;
      this.box_w = null;
      this.box_h = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('box_x')) {
        this.box_x = initObj.box_x
      }
      else {
        this.box_x = 0;
      }
      if (initObj.hasOwnProperty('box_y')) {
        this.box_y = initObj.box_y
      }
      else {
        this.box_y = 0;
      }
      if (initObj.hasOwnProperty('box_w')) {
        this.box_w = initObj.box_w
      }
      else {
        this.box_w = 0;
      }
      if (initObj.hasOwnProperty('box_h')) {
        this.box_h = initObj.box_h
      }
      else {
        this.box_h = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type BoxinfoRequest
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [box_x]
    bufferOffset = _serializer.uint16(obj.box_x, buffer, bufferOffset);
    // Serialize message field [box_y]
    bufferOffset = _serializer.uint16(obj.box_y, buffer, bufferOffset);
    // Serialize message field [box_w]
    bufferOffset = _serializer.uint16(obj.box_w, buffer, bufferOffset);
    // Serialize message field [box_h]
    bufferOffset = _serializer.uint16(obj.box_h, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type BoxinfoRequest
    let len;
    let data = new BoxinfoRequest(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [box_x]
    data.box_x = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [box_y]
    data.box_y = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [box_w]
    data.box_w = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [box_h]
    data.box_h = _deserializer.uint16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ucar_nav/BoxinfoRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'beb47c590a7a389074e521a1e16ec95d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    uint16 box_x
    uint16 box_y
    uint16 box_w
    uint16 box_h
    
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
    string frame_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new BoxinfoRequest(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.box_x !== undefined) {
      resolved.box_x = msg.box_x;
    }
    else {
      resolved.box_x = 0
    }

    if (msg.box_y !== undefined) {
      resolved.box_y = msg.box_y;
    }
    else {
      resolved.box_y = 0
    }

    if (msg.box_w !== undefined) {
      resolved.box_w = msg.box_w;
    }
    else {
      resolved.box_w = 0
    }

    if (msg.box_h !== undefined) {
      resolved.box_h = msg.box_h;
    }
    else {
      resolved.box_h = 0
    }

    return resolved;
    }
};

class BoxinfoResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.pla_x = null;
      this.pla_y = null;
      this.road = null;
    }
    else {
      if (initObj.hasOwnProperty('pla_x')) {
        this.pla_x = initObj.pla_x
      }
      else {
        this.pla_x = 0.0;
      }
      if (initObj.hasOwnProperty('pla_y')) {
        this.pla_y = initObj.pla_y
      }
      else {
        this.pla_y = 0.0;
      }
      if (initObj.hasOwnProperty('road')) {
        this.road = initObj.road
      }
      else {
        this.road = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type BoxinfoResponse
    // Serialize message field [pla_x]
    bufferOffset = _serializer.float32(obj.pla_x, buffer, bufferOffset);
    // Serialize message field [pla_y]
    bufferOffset = _serializer.float32(obj.pla_y, buffer, bufferOffset);
    // Serialize message field [road]
    bufferOffset = _serializer.float32(obj.road, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type BoxinfoResponse
    let len;
    let data = new BoxinfoResponse(null);
    // Deserialize message field [pla_x]
    data.pla_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [pla_y]
    data.pla_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [road]
    data.road = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 12;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ucar_nav/BoxinfoResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2ce00a84622f710e839e3b8478187df7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 pla_x
    float32 pla_y
    float32 road
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new BoxinfoResponse(null);
    if (msg.pla_x !== undefined) {
      resolved.pla_x = msg.pla_x;
    }
    else {
      resolved.pla_x = 0.0
    }

    if (msg.pla_y !== undefined) {
      resolved.pla_y = msg.pla_y;
    }
    else {
      resolved.pla_y = 0.0
    }

    if (msg.road !== undefined) {
      resolved.road = msg.road;
    }
    else {
      resolved.road = 0.0
    }

    return resolved;
    }
};

module.exports = {
  Request: BoxinfoRequest,
  Response: BoxinfoResponse,
  md5sum() { return 'dc4ccb29fbc5c9e5a5b7a5542979e09e'; },
  datatype() { return 'ucar_nav/Boxinfo'; }
};
