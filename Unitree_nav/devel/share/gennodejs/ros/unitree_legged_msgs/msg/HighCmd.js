// Auto-generated. Do not edit!

// (in-package unitree_legged_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let LED = require('./LED.js');

//-----------------------------------------------------------

class HighCmd {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.levelFlag = null;
      this.commVersion = null;
      this.robotID = null;
      this.SN = null;
      this.bandWidth = null;
      this.mode = null;
      this.gaitType = null;
      this.speedLevel = null;
      this.dFootRaiseHeight = null;
      this.dBodyHeight = null;
      this.position = null;
      this.rpy = null;
      this.velocity = null;
      this.yawSpeed = null;
      this.led = null;
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
      if (initObj.hasOwnProperty('gaitType')) {
        this.gaitType = initObj.gaitType
      }
      else {
        this.gaitType = 0;
      }
      if (initObj.hasOwnProperty('speedLevel')) {
        this.speedLevel = initObj.speedLevel
      }
      else {
        this.speedLevel = 0;
      }
      if (initObj.hasOwnProperty('dFootRaiseHeight')) {
        this.dFootRaiseHeight = initObj.dFootRaiseHeight
      }
      else {
        this.dFootRaiseHeight = 0.0;
      }
      if (initObj.hasOwnProperty('dBodyHeight')) {
        this.dBodyHeight = initObj.dBodyHeight
      }
      else {
        this.dBodyHeight = 0.0;
      }
      if (initObj.hasOwnProperty('position')) {
        this.position = initObj.position
      }
      else {
        this.position = new Array(2).fill(0);
      }
      if (initObj.hasOwnProperty('rpy')) {
        this.rpy = initObj.rpy
      }
      else {
        this.rpy = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('velocity')) {
        this.velocity = initObj.velocity
      }
      else {
        this.velocity = new Array(2).fill(0);
      }
      if (initObj.hasOwnProperty('yawSpeed')) {
        this.yawSpeed = initObj.yawSpeed
      }
      else {
        this.yawSpeed = 0.0;
      }
      if (initObj.hasOwnProperty('led')) {
        this.led = initObj.led
      }
      else {
        this.led = new Array(4).fill(new LED());
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
    // Serializes a message object of type HighCmd
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
    // Serialize message field [gaitType]
    bufferOffset = _serializer.uint8(obj.gaitType, buffer, bufferOffset);
    // Serialize message field [speedLevel]
    bufferOffset = _serializer.uint8(obj.speedLevel, buffer, bufferOffset);
    // Serialize message field [dFootRaiseHeight]
    bufferOffset = _serializer.float32(obj.dFootRaiseHeight, buffer, bufferOffset);
    // Serialize message field [dBodyHeight]
    bufferOffset = _serializer.float32(obj.dBodyHeight, buffer, bufferOffset);
    // Check that the constant length array field [position] has the right length
    if (obj.position.length !== 2) {
      throw new Error('Unable to serialize array field position - length must be 2')
    }
    // Serialize message field [position]
    bufferOffset = _arraySerializer.float32(obj.position, buffer, bufferOffset, 2);
    // Check that the constant length array field [rpy] has the right length
    if (obj.rpy.length !== 3) {
      throw new Error('Unable to serialize array field rpy - length must be 3')
    }
    // Serialize message field [rpy]
    bufferOffset = _arraySerializer.float32(obj.rpy, buffer, bufferOffset, 3);
    // Check that the constant length array field [velocity] has the right length
    if (obj.velocity.length !== 2) {
      throw new Error('Unable to serialize array field velocity - length must be 2')
    }
    // Serialize message field [velocity]
    bufferOffset = _arraySerializer.float32(obj.velocity, buffer, bufferOffset, 2);
    // Serialize message field [yawSpeed]
    bufferOffset = _serializer.float32(obj.yawSpeed, buffer, bufferOffset);
    // Check that the constant length array field [led] has the right length
    if (obj.led.length !== 4) {
      throw new Error('Unable to serialize array field led - length must be 4')
    }
    // Serialize message field [led]
    obj.led.forEach((val) => {
      bufferOffset = LED.serialize(val, buffer, bufferOffset);
    });
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
    //deserializes a message object of type HighCmd
    let len;
    let data = new HighCmd(null);
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
    // Deserialize message field [gaitType]
    data.gaitType = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [speedLevel]
    data.speedLevel = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [dFootRaiseHeight]
    data.dFootRaiseHeight = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [dBodyHeight]
    data.dBodyHeight = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [position]
    data.position = _arrayDeserializer.float32(buffer, bufferOffset, 2)
    // Deserialize message field [rpy]
    data.rpy = _arrayDeserializer.float32(buffer, bufferOffset, 3)
    // Deserialize message field [velocity]
    data.velocity = _arrayDeserializer.float32(buffer, bufferOffset, 2)
    // Deserialize message field [yawSpeed]
    data.yawSpeed = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [led]
    len = 4;
    data.led = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.led[i] = LED.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [wirelessRemote]
    data.wirelessRemote = _arrayDeserializer.uint8(buffer, bufferOffset, 40)
    // Deserialize message field [reserve]
    data.reserve = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [crc]
    data.crc = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 104;
  }

  static datatype() {
    // Returns string type for a message object
    return 'unitree_legged_msgs/HighCmd';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '59333cac6112fd0a7a3d3c5b564af567';
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
    uint8 gaitType		   
    uint8 speedLevel	
    float32 dFootRaiseHeight		   
    float32 dBodyHeight	 
    float32[2] position 
    float32[3] rpy	   
    float32[2] velocity 
    float32 yawSpeed		   
    LED[4] led
    uint8[40] wirelessRemote
    uint32 reserve
    uint32 crc
    ================================================================================
    MSG: unitree_legged_msgs/LED
    uint8 r
    uint8 g
    uint8 b
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new HighCmd(null);
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

    if (msg.gaitType !== undefined) {
      resolved.gaitType = msg.gaitType;
    }
    else {
      resolved.gaitType = 0
    }

    if (msg.speedLevel !== undefined) {
      resolved.speedLevel = msg.speedLevel;
    }
    else {
      resolved.speedLevel = 0
    }

    if (msg.dFootRaiseHeight !== undefined) {
      resolved.dFootRaiseHeight = msg.dFootRaiseHeight;
    }
    else {
      resolved.dFootRaiseHeight = 0.0
    }

    if (msg.dBodyHeight !== undefined) {
      resolved.dBodyHeight = msg.dBodyHeight;
    }
    else {
      resolved.dBodyHeight = 0.0
    }

    if (msg.position !== undefined) {
      resolved.position = msg.position;
    }
    else {
      resolved.position = new Array(2).fill(0)
    }

    if (msg.rpy !== undefined) {
      resolved.rpy = msg.rpy;
    }
    else {
      resolved.rpy = new Array(3).fill(0)
    }

    if (msg.velocity !== undefined) {
      resolved.velocity = msg.velocity;
    }
    else {
      resolved.velocity = new Array(2).fill(0)
    }

    if (msg.yawSpeed !== undefined) {
      resolved.yawSpeed = msg.yawSpeed;
    }
    else {
      resolved.yawSpeed = 0.0
    }

    if (msg.led !== undefined) {
      resolved.led = new Array(4)
      for (let i = 0; i < resolved.led.length; ++i) {
        if (msg.led.length > i) {
          resolved.led[i] = LED.Resolve(msg.led[i]);
        }
        else {
          resolved.led[i] = new LED();
        }
      }
    }
    else {
      resolved.led = new Array(4).fill(new LED())
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

module.exports = HighCmd;
