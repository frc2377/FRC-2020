package frc.robot.utils;

public class Utils {

  /**
   *
   * @param degrees degrees to convert to tics
   * @param gearRatio gear ratio of the drive
   * @param ppr pulses per revolution of the encoder
   * @return Number of encoder tics corresponding to degrees input
   */
  public static int degrees2Tics(double degrees, double gearRatio, int ppr) {
    return (int)((degrees * gearRatio * ppr) / 360.0);
  }

  /**
   *
   * @param velocity velocity in raw tics / 100ms to convert to rps
   * @param gearRatio gear ratio of the drive
   * @param ppr pulses per revolution of the encoder
   * @return the corresponding rotations per second
   */
  public static int tics2Rps(int velocity, double gearRatio, int ppr) {
    return (int)((velocity * 10) / (gearRatio * ppr));
  }

  /**
   *
   * @param rps rotations per second to convert to raw tics / 100ms
   * @param gearRatio gear ratio of the drive
   * @param ppr pulses per revolution on the encoder
   * @return raw tics per 100ms
   */
  public static int rps2Tics(int rps, double gearRatio, int ppr) {
    return (int)((rps * ppr * gearRatio) / 10);
  }
}
