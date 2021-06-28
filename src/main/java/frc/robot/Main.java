/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import badlog.lib.BadLog;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * Do NOT add any static variables to this class, or any initialization at all. Unless you know what
 * you are doing, do not modify this file except to change the parameter class to the startRobot
 * call.
 */
public final class Main {
  public static BadLog log;
  private Main() {
  }

  /**
   * Main initialization function. Do not perform any initialization here.
   *
   * <p>If you change your main robot class, change the parameter type.
   */
  public static void main(String... args) {
    // logging setup
    log = BadLog.init("log.bag");

    BadLog.createTopicSubscriber("LeftMasterSupply", "Amps", badlog.lib.DataInferMode.LAST);
    BadLog.createTopicSubscriber("RightMasterSupply", "Amps", badlog.lib.DataInferMode.LAST);
    BadLog.createTopicSubscriber("LeftFollowSupply", "Amps", badlog.lib.DataInferMode.LAST);
    BadLog.createTopicSubscriber("RightFollowSupply", "Amps", badlog.lib.DataInferMode.LAST);
    
    log.finishInitialization();

    RobotBase.startRobot(Robot::new);
  }
}
