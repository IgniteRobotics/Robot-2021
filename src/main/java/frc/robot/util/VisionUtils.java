/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

import frc.robot.constants.Constants;

/**
 * Add your docs here.
 */
public class VisionUtils {


    public static  double getDistanceToTarget(double pitch){
        //d = (h2-h1) / tan(a1+a2)
        return (Constants.TARGET_HEIGHT - Constants.LIMELIGHT_HEIGHT) / Math.tan(Math.toRadians(Constants.LIMELIGHT_ANGLE + pitch));
    }



}
