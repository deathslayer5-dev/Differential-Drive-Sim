// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    
  }
  //Declare Drive Constants for Unit 4
  public static class DriveConstants {
    public static final int kLeftLeaderMotorPort = 1;
    public static final int kLeftFollowerMotorPort = 2;
    public static final int kRightLeaderMotorPort = 3;
    public static final int kRightFollowerMotorPort = 4;
  }
  //Declare Arm Constants for Unit 4
  public static class ArmConstants{
    public final static int canID=12;
    public final static double gearRatio=25;
    public final static double kP=10;
    public final static double kI=0;
    public final static double kD=0;
    public final static double maxVel=1000;
    public final static double maxAcc=1000;
    public final static boolean brakeMode=true;
    public final static boolean enableStatorLimit=true;
    public final static double statorLimit=40;
    public final static boolean enableSupplyLimit=true;
    public final static double supplyLimit=40;
    public final static double armLength=40;
    public final static double minAngleDeg =0;
    public final static double maxAngleDeg = 90;
    public final static double kS=0;
    public final static double kG=0;
    public final static double kA=0;
    public final static double kV=0;
  }
}
