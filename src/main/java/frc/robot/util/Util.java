package frc.robot.util;

public class Util {

	private static int ENCODER_TICKS_PER_REVOLUTION = 8192;
	private static int WHEEL_DIAMETER = 6; //in inches
	private static double WHEEL_DIAMETER_METERS = 0.1524;

	private static double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;

	public static double applyDeadband(double value, double deadband) {
		if (Math.abs(value) > deadband) {
			if (value > 0.0) {
				return (value - deadband) / (1.0 - deadband);
			} else {
				return (value + deadband) / (1.0 - deadband);
			}
		} else {
			return 0.0;
		}
	}

	public static double getEncoderTicksFromInches(double inches) {
		return getEncoderTicksFromInches(WHEEL_DIAMETER, inches);
	}

	private static double getEncoderTicksFromInches(int wheel_diameter, double inches) {

		double inchesPerRevolution = Math.PI * wheel_diameter;

		double revolutions = inches / inchesPerRevolution;
		double ticks = revolutions * ENCODER_TICKS_PER_REVOLUTION;

		return ticks;
	}

	public static double getInchesFromEncoderTicks(double encoder_ticks) {
		return getInchesFromEncoderTicks(WHEEL_DIAMETER, encoder_ticks);
	}

	private static double getInchesFromEncoderTicks(int wheel_diameter, double encoder_ticks) {

		double inchesPerRevolution = Math.PI * wheel_diameter;
		double revolutions = encoder_ticks * ENCODER_TICKS_PER_REVOLUTION;

		double inches = (revolutions) / (inchesPerRevolution);

		return inches;
	}

	public static double getMetersFromEncoderTicks(double ticks) {
		return (WHEEL_CIRCUMFERENCE_METERS / ENCODER_TICKS_PER_REVOLUTION) * ticks;
	}
	public static double stepsPerDecisecToMetersPerSec(double d) {
		return getMetersFromEncoderTicks(d * 10);
	}

	public static double metersPerSecToStepsPerDecisec(double metersPerSec) {
		return getEncoderTicksFromMeters(metersPerSec) * .1;
	}

	public static double getEncoderTicksFromMeters(double meters) {
		return (meters / WHEEL_CIRCUMFERENCE_METERS) * ENCODER_TICKS_PER_REVOLUTION;
	}

}