// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANdleConstants;
import frc.robot.Constants.EnabledSubsystems;

public class CANdleSubsystem extends SubsystemBase {

  private final int LEDOFFSET = 8; // move LED numbers because there are 8 LEDs on the controller
  private CANdle candle;

  public enum AnimationTypes {
    ColorFlow,
    Fire,
    Larson,
    Rainbow,
    RgbFade,
    SingleFade,
    Strobe,
    Twinkle,
    TwinkleOff,
    SetAll
  }

  /** Creates a new CANdleSubsystem. */
  public CANdleSubsystem() {

    // Check if need to initialize arm
    if (!EnabledSubsystems.candle) {
      return;
    }

    System.out.println("Initializing CANdle");

    candle = new CANdle(CANdleConstants.CANdleCANID);
    candle.configLEDType(LEDStripType.GRB);

  }

  public void setLEDBlue()
  {
    candle.setLEDs(10,10,200, 0, 13, CANdleConstants.LedCount - 10);
    candle.modulateVBatOutput(0.9);
  }

  public void setLEDRed()
  {
    candle.setLEDs(220,0,0, 0, 15, CANdleConstants.LedCount - 10);
    candle.modulateVBatOutput(0.9);
  }

  public void setLEDGreen()
  {
    candle.setLEDs(10,200,10, 0, 1, 12);
    candle.modulateVBatOutput(0.9);
  }

  public void setLEDAngle()
  {
    candle.setLEDs(10,10,200, 0, 13, 2);
    candle.modulateVBatOutput(0.9);
  }

  public void setLEDAngleOff()
  {
    candle.setLEDs(0,0,0, 0, 13, 2);
    candle.modulateVBatOutput(0);
  }

  public void blinkGreen()
  {
    candle.animate(new StrobeAnimation(57, 128, 13, 0, 98.0 / 256.0, CANdleConstants.LedCount));
  }

    public void blinkRed()
  {
    candle.animate(new StrobeAnimation(220, 0, 0, 0, 1.0 / 2560000.0, CANdleConstants.LedCount));
  }
    public void stopBlinking() {
      candle.animate(null);
    }

  public void setLEDOff()
  {
    candle.setLEDs(0,0,0, 0, 12, CANdleConstants.LedCount - 2);
    candle.modulateVBatOutput(0);
  }

  public void setLEDOffGreen()
  {
    candle.setLEDs(0,0,0, 0, 1, 12);
    candle.modulateVBatOutput(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
