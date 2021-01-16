/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

/**
 * Add your docs here.
 */
public class VisionUtils {

    //the height of the camera from the floor, in inches.
    public static final double cameraHeight = 24.5;

    //the height of the upper port target
    public static final double upperTargetHeight = 50.5;

    //the angle of the camera, relative to the floor.
    public static final double cameraAngle = 0;


    public static  double getDistanceToTarget(double pitch){
        //d = (h2-h1) / tan(a1+a2)
        return (upperTargetHeight - cameraHeight) / Math.tan(Math.toRadians(cameraAngle + pitch));
    }



}
