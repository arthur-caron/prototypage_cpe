
"use strict";

let SetTorqueLimit = require('./SetTorqueLimit.js')
let SetComplianceSlope = require('./SetComplianceSlope.js')
let SetComplianceMargin = require('./SetComplianceMargin.js')
let StopController = require('./StopController.js')
let TorqueEnable = require('./TorqueEnable.js')
let SetCompliancePunch = require('./SetCompliancePunch.js')
let SetSpeed = require('./SetSpeed.js')
let StartController = require('./StartController.js')
let RestartController = require('./RestartController.js')

module.exports = {
  SetTorqueLimit: SetTorqueLimit,
  SetComplianceSlope: SetComplianceSlope,
  SetComplianceMargin: SetComplianceMargin,
  StopController: StopController,
  TorqueEnable: TorqueEnable,
  SetCompliancePunch: SetCompliancePunch,
  SetSpeed: SetSpeed,
  StartController: StartController,
  RestartController: RestartController,
};
