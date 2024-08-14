// Auto-generated. Do not edit!

// (in-package unitree_legged_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let IMU = require('./IMU.js');
let Cartesian = require('./Cartesian.js');

//-----------------------------------------------------------

class HighState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.levelFlag = null;
      this.commVersion = null;
      this.robotID = null;
      this.SN = null;
      this.bandWidth = null;
      this.mode = null;
      this.imu = null;
      this.position = null;
      this.velocity = null;
      this.yawSpeed = null;
      this.footPosition2Body = null;
      this.footSpeed2Body = null;
      this.footForce = null;
      this.wirelessRemote = null;
      this.reserve = null;
      this.crc = null;
    }
    else {
      if (initObj.hasOwnProperty('levelFlag')) {
        this.levelFlag = initObj.levelFlag
      }
      else {
        this.levelFlag = 0;
      }
      if (initObj.hasOwnProperty('commVersion')) {
        this.commVersion = initObj.commVersion
      }
      else {
        this.commVersion = 0;
      }
      if (initObj.hasOwnProperty('robotID')) {
        this.robotID = initObj.robotID
      }
      else {
        this.robotID = 0;
      }
      if (initObj.hasOwnProperty('SN')) {
        this.SN = initObj.SN
      }
      else {
        this.SN = 0;
      }
      if (initObj.hasOwnProperty('bandWidth')) {
        this.bandWidth = initObj.bandWidth
      }
      else {
        this.bandWidth = 0;
      }
      if (initObj.hasOwnProperty('mode')) {
        this.mode = initObj.mode
      }
      else {
        this.mode = 0;
      }
      if (initObj.hasOwnProperty('imu')) {
        this.imu = initObj.imu
      }
      else {
        this.imu = new IMU();
      }
      if (initObj.hasOwnProperty('position')) {
        this.position = initObj.position
      }
      else {
        this.position = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('velocity')) {
        this.velocity = initObj.velocity
      }
      else {
        this.velocity = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('yawSpeed')) {
        this.yawSpeed = initObj.yawSpeed
      }
      else {
        this.yawSpeed = 0.0;
      }
      if (initObj.hasOwnProperty('footPosition2Body')) {
        this.footPosition2Body = initObj.footPosition2Body
      }
      else {
        this.footPosition2Body = new Array(4).fill(new Cartesian());
      }
      if (initObj.hasOwnProperty('footSpeed2Body')) {
        this.footSpeed2Body = initObj.footSpeed2Body
      }
      else {
        this.footSpeed2Body = new Array(4).fill(new Cartesian());
      }
      if (initObj.hasOwnProperty('footForce')) {
        this.footForce = initObj.footForce
      }
      else {
        this.footForce = new Array(4).fill(0);
      }
      if (initObj.hasOwnProperty('wirelessRemote')) {
        this.wirelessRemote = initObj.wirelessRemote
      }
      else {
        this.wirelessRemote = new Array(40).fill(0);
      }
      if (initObj.hasOwnProperty('reserve')) {
        this.reserve = initObj.reserve
      }
      else {
        this.reserve = 0;
      }
      if (initObj.hasOwnProperty('crc')) {
        this.crc = initObj.crc
      }
      else {
        this.crc = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type HighState
    // Serialize message field [levelFlag]
    bufferOffset = _serializer.uint8(obj.levelFlag, buffer, bufferOffset);
    // Serialize message field [commVersion]
    bufferOffset = _serializer.uint16(obj.commVersion, buffer, bufferOffset);
    // Serialize message field [robotID]
    bufferOffset = _serializer.uint16(obj.robotID, buffer, bufferOffset);
    // Serialize message field [SN]
    bufferOffset = _serializer.uint32(obj.SN, buffer, bufferOffset);
    // Serialize message field [bandWidth]
    bufferOffset = _serializer.uint8(obj.bandWidth, buffer, bufferOffset);
    // Serialize message field [mode]
    bufferOffset = _serializer.uint8(obj.mode, buffer, bufferOffset);
    // Serialize message field [imu]
    bufferOffset = IMU.serialize(obj.imu, buffer, bufferOffset);
    // Check that the constant length array field [position] has the right length
    if (obj.position.length !== 3) {
      throw new Error('Unable to serialize array field position - length must be 3')
    }
    // Serialize message field [position]
    bufferOffset = _arraySerializer.float32(obj.position, buffer, bufferOffset, 3);
    // Check that the constant length array field [velocity] has the right length
    if (obj.velocity.length !== 3) {
      throw new Error('Unable to serialize array field velocity - length must be 3')
    }
    // Serialize message field [velocity]
    bufferOffset = _arraySerializer.float32(obj.velocity, buffer, bufferOffset, 3);
    // Serialize message field [yawSpeed]
    bufferOffset = _serializer.float32(obj.yawSpeed, buffer, bufferOffset);
    // Check that the constant length array field [footPosition2Body] has the right length
    if (obj.footPosition2Body.length !== 4) {
      throw new Error('Unable to serialize array field footPosition2Body - length must be 4')
    }
    // Serialize message field [footPosition2Body]
    obj.footPosition2Body.forEach((val) => {
      bufferOffset = Cartesian.serialize(val, buffer, bufferOffset);
    });
    // Check that the constant length array field [footSpeed2Body] has the right length
    if (obj.footSpeed2Body.length !== 4) {
      throw new Error('Unable to serialize array field footSpeed2Body - length must be 4')
    }
    // Serialize message field [footSpeed2Body]
    obj.footSpeed2Body.forEach((val) => {
      bufferOffset = Cartesian.serialize(val, buffer, bufferOffset);
    });
    // Check that the constant length array field [footForce] has the right length
    if (obj.footForce.length !== 4) {
      throw new Error('Unable to serialize array field footForce - length must be 4')
    }
    // Serialize message field [footForce]
    bufferOffset = _arraySerializer.int16(obj.footForce, buffer, bufferOffset, 4);
    // Check that the constant length array field [wirelessRemote] has the right length
    if (obj.wirelessRemote.length !== 40) {
      throw new Error('Unable to serialize array field wirelessRemote - length must be 40')
    }
    // Serialize message field [wirelessRemote]
    bufferOffset = _arraySerializer.uint8(obj.wirelessRemote, buffer, bufferOffset, 40);
    // Serialize message field [reserve]
    bufferOffset = _serializer.uint32(obj.reserve, buffer, bufferOffset);
    // Serialize message field [crc]
    bufferOffset = _serializer.uint32(obj.crc, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type HighState
    let len;
    let data = new HighState(null);
    // Deserialize message field [levelFlag]
    data.levelFlag = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [commVersion]
    data.commVersion = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [robotID]
    data.robotID = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [SN]
    data.SN = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [bandWidth]
    data.bandWidth = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [mode]
    data.mode = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [imu]
    data.imu = IMU.deserialize(buffer, bufferOffset);
    // Deserialize message field [position]
    data.position = _arrayDeserializer.float32(buffer, bufferOffset, 3)
    // Deserialize message field [velocity]
    data.velocity = _arrayDeserializer.float32(buffer, bufferOffset, 3)
    // Deserialize message field [yawSpeed]
    data.yawSpeed = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [footPosition2Body]
    len = 4;
    data.footPosition2Body = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.footPosition2Body[i] = Cartesian.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [footSpeed2Body]
    len = 4;
    data.footSpeed2Body = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.footSpeed2Body[i] = Cartesian.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [footForce]
    data.footForce = _arrayDeserializer.int16(buffer, bufferOffset, 4)
    // Deserialize message field [wirelessRemote]
    data.wirelessRemote = _arrayDeserializer.uint8(buffer, bufferOffset, 40)
    // Deserialize message field [reserve]
    data.reserve = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [crc]
    data.crc = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 172;
  }

  static datatype() {
    // Returns string type for a message object
    return 'unitree_legged_msgs/HighState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5ac674d268805caf9dbab61ef025c30a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8 levelFlag
    uint16 commVersion
    uint16 robotID
    uint32 SN
    uint8 bandWidth
    uint8 mode
    IMU imu
    float32[3] position 
    float32[3] velocity 
    float32 yawSpeed
    Cartesian[4] footPosition2Body 
    Cartesian[4] footSpeed2Body	
    int16[4] footForce
    uint8[40] wirelessRemote
    uint32 reserve
    uint32 crc
    
    
    
    
    
    
    ================================================================================
    MSG: unitree_legged_msgs/IMU
    float32[4] quaternion
    float32[3] gyroscope
    float32[3] accelerometer
    float32[3] rpy
    int8 temperature
    ================================================================================
    MSG: unitree_legged_msgs/Cartesian
    float32 x
    float32 y
    float32 z
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new HighState(null);
    if (msg.levelFlag !== undefined) {
      resolved.levelFlag = msg.levelFlag;
    }
    else {
      resolved.levelFlag = 0
    }

    if (msg.commVersion !== undefined) {
      resolved.commVersion = msg.commVersion;
    }
    else {
      resolved.commVersion = 0
    }

    if (msg.robotID !== undefined) {
      resolved.robotID = msg.robotID;
    }
    else {
      resolved.robotID = 0
    }

    if (msg.SN !== undefined) {
      resolved.SN = msg.SN;
    }
    else {
      resolved.SN = 0
    }

    if (msg.bandWidth !== undefined) {
      resolved.bandWidth = msg.bandWidth;
    }
    else {
      resolved.bandWidth = 0
    }

    if (msg.mode !== undefined) {
      resolved.mode = msg.mode;
    }
    else {
      resolved.mode = 0
    }

    if (msg.imu !== undefined) {
      resolved.imu = IMU.Resolve(msg.imu)
    }
    else {
      resolved.imu = new IMU()
    }

    if (msg.position !== undefined) {
      resolved.position = msg.position;
    }
    else {
      resolved.position = new Array(3).fill(0)
    }

    if (msg.velocity !== undefined) {
      resolved.velocity = msg.velocity;
    }
    else {
      resolved.velocity = new Array(3).fill(0)
    }

    if (msg.yawSpeed !== undefined) {
      resolved.yawSpeed = msg.yawSpeed;
    }
    else {
      resolved.yawSpeed = 0.0
    }

    if (msg.footPosition2Body !== undefined) {
      resolved.footPosition2Body = new Array(4)
      for (let i = 0; i < resolved.footPosition2Body.length; ++i) {
        if (msg.footPosition2Body.length > i) {
          resolved.footPosition2Body[i] = Cartesian.Resolve(msg.footPosition2Body[i]);
        }
        else {
          resolved.footPosition2Body[i] = new Cartesian();
        }
      }
    }
    else {
      resolved.footPosition2Body = new Array(4).fill(new Cartesian())
    }

    if (msg.footSpeed2Body !== undefined) {
      resolved.footSpeed2Body = new Array(4)
      for (let i = 0; i < resolved.footSpeed2Body.length; ++i) {
        if (msg.footSpeed2Body.length > i) {
          resolved.footSpeed2Body[i] = Cartesian.Resolve(msg.footSpeed2Body[i]);
        }
        else {
          resolved.footSpeed2Body[i] = new Cartesian();
        }
      }
    }
    else {
      resolved.footSpeed2Body = new Array(4).fill(new Cartesian())
    }

    if (msg.footForce !== undefined) {
      resolved.footForce = msg.footForce;
    }
    else {
      resolved.footForce = new Array(4).fill(0)
    }

    if (msg.wirelessRemote !== undefined) {
      resolved.wirelessRemote = msg.wirelessRemote;
    }
    else {
      resolved.wirelessRemote = new Array(40).fill(0)
    }

    if (msg.reserve !== undefined) {
      resolved.reserve = msg.reserve;
    }
    else {
      resolved.reserve = 0
    }

    if (msg.crc !== undefined) {
      resolved.crc = msg.crc;
    }
    else {
      resolved.crc = 0
    }

    return resolved;
    }
};

module.exports = HighState;
