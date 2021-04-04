package frc.robot.util;


/**
  * Represents the unique characteristics for each path in order to determine which path to be used.
  * 
  * There are two characteristics that used together can uniquely identify each path - the distance of the closest ball,
  * and the offset of the closest ball (in px). For example, the red spots on path a could be characterized as 90 cm and 0 px,
  * since the robot would be 90 cm away from the ball and the ball would be in the center of the camera. (The robot should be placed in the middle.)
  */
public class PathData {
    private double dist;
    private double ballOffset; //pixel offset
    private String trajectory;
    
    public PathData(double dist, double ballOffset, String trajectory) {
        this.dist = dist;
        this.ballOffset = ballOffset;
        this.trajectory = trajectory;
    }
    
    public double getDist() {
        return this.dist;
    }
    
    public double getOffset() {
        return this.ballOffset;
    }
    
    public String getTrajectory() {
        return this.trajectory;
    }
    /**
    * Returns the average relative percent error of the values from the PathData parameter compared to this path
    * We want to loop through a list of all possible paths, run this method on each of those paths along with the path from the camera data,
    * and choose the path with the least error.
    */
    public double getPathError(PathData d) {
        //double distError = Math.abs(this.dist - d.getDist() / d.getDist()   );
        // double offsetError = Math.abs(this.ballOffset - d.getOffset() / d.getOffset());
        double offsetError = Math.abs(this.ballOffset - d.getOffset()); 
        return (offsetError);
    }
}