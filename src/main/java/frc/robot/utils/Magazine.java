/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.MagazineConstants;

/**
 * Add your docs here.
 */
public class Magazine {
  static DigitalInput garageSensor1 = new DigitalInput(MagazineConstants.kGarageSensorPort1);
  static DigitalInput garageSensor2 = new DigitalInput(MagazineConstants.kGarageSensorPort2);
  static DigitalInput garageSensor3 = new DigitalInput(MagazineConstants.kGarageSensorPort3);
  static DigitalInput garageSensor4 = new DigitalInput(MagazineConstants.kGarageSensorPort4);
  static DigitalInput garageSensor5 = new DigitalInput(MagazineConstants.kGarageSensorPort5);

public static int numberOfPowerCells() {
  int num = 0;
  if (!garageSensor1.get()) {
    num++;
  }
  if (!garageSensor2.get()) {
    num++;
  }
  if (!garageSensor3.get()) {
    num++;
  }
  if (!garageSensor4.get()) {
    num++;
  }
  if (!garageSensor5.get()) {
    num++;
  }
  return num;
}

public static boolean sensor1Blocked() {
  return !garageSensor1.get();
}
public static boolean sensor2Blocked() {
    return !garageSensor2.get();
  }
public static boolean sensor3Blocked() {
    return !garageSensor3.get();
  }
public static boolean sensor4Blocked() {
    return !garageSensor4.get();
  }
public static boolean sensor5Blocked() {
    return !garageSensor5.get();
  }

  /**
   * Check if the conveyor is full
   *
   * @return True if conveyor contains 5 Power Cells
   */
  public static boolean magazineIsFull() {
    return numberOfPowerCells() == 5;
  }

}
