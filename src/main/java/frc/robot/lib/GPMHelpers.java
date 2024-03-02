package frc.robot.lib;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class GPMHelpers {

    public InterpolatingDoubleTreeMap GPM_0_Angle = new InterpolatingDoubleTreeMap();
    public InterpolatingDoubleTreeMap GPM_0_IntakePower = new InterpolatingDoubleTreeMap();
    public InterpolatingDoubleTreeMap GPM_0_ShooterPower = new InterpolatingDoubleTreeMap();

    public InterpolatingDoubleTreeMap GPM_60_Angle = new InterpolatingDoubleTreeMap();
    public InterpolatingDoubleTreeMap GPM_60_IntakePower = new InterpolatingDoubleTreeMap();
    public InterpolatingDoubleTreeMap GPM_60_ShooterPower = new InterpolatingDoubleTreeMap();

    public InterpolatingDoubleTreeMap Measured_Shooter_Speeds_From_Power = new InterpolatingDoubleTreeMap();



    public GPMHelpers() {
        setGPM0Angle();
        setGPM0IntakePower();
        setGPM0ShooterPower();
        setGPM60Angle();
        setGPM60IntakePower();
        setGPM60ShooterPower();
        setMeasuredShootingSpeedsFromPower();
    }

    public double getAngleTouchingAmp() {
        return 2.42;
    }

    public double getIntakePowerTouchingAmp() {
        return 0.34;
    }

    public double getShooterPowerTouchingAmp() {
        return 0.4;
    }

    public void setGPM0Angle() {
        GPM_0_Angle.put(0.0, -72.00);
        GPM_0_Angle.put(1.0, -65.26);
        GPM_0_Angle.put(2.0, -50.70);
        GPM_0_Angle.put(3.0, -41.82);
        GPM_0_Angle.put(4.0, -37.81);
    }

    public void setGPM0IntakePower() {
        GPM_0_IntakePower.put(0.0, 0.34);
        GPM_0_IntakePower.put(1.0, 0.51);
        GPM_0_IntakePower.put(2.0, 0.51);
        GPM_0_IntakePower.put(3.0, 0.55);
        GPM_0_IntakePower.put(4.0, 0.60);
    }

    public void setGPM0ShooterPower() {
        GPM_0_ShooterPower.put(0.0, 0.60);
        GPM_0_ShooterPower.put(1.0, 0.60);
        GPM_0_ShooterPower.put(2.0, 0.67);
        GPM_0_ShooterPower.put(3.0, 0.84);
        GPM_0_ShooterPower.put(4.0, 0.9);
    }

    public void setGPM60Angle() {
        GPM_60_Angle.put(0.0, -71.52);
        GPM_60_Angle.put(1.0, -63.81);
        GPM_60_Angle.put(2.0, -51.81);
        GPM_60_Angle.put(3.0, -44.61);
    }

    public void setGPM60IntakePower() {
        GPM_60_IntakePower.put(0.0, 0.51);
        GPM_60_IntakePower.put(1.0, 0.51);
        GPM_60_IntakePower.put(2.0, 0.51);
        GPM_60_IntakePower.put(3.0, 0.51);
    }

    public void setGPM60ShooterPower() {
        GPM_60_ShooterPower.put(0.0, 0.65);
        GPM_60_ShooterPower.put(1.0, 0.65);
        GPM_60_ShooterPower.put(2.0, 0.73);
        GPM_60_ShooterPower.put(3.0, 0.85);
    }

    public void setMeasuredShootingSpeedsFromPower() {   
        Measured_Shooter_Speeds_From_Power.put(getGPM0ShooterPower(0.0), 0.0);
        Measured_Shooter_Speeds_From_Power.put(getGPM0ShooterPower(1.0), 0.0);
        Measured_Shooter_Speeds_From_Power.put(getGPM0ShooterPower(2.0), 0.0);
        Measured_Shooter_Speeds_From_Power.put(getGPM0ShooterPower(3.0), 0.0);

        Measured_Shooter_Speeds_From_Power.put(getGPM60ShooterPower(0.0), 0.0);
        Measured_Shooter_Speeds_From_Power.put(getGPM60ShooterPower(1.0), 0.0);
        Measured_Shooter_Speeds_From_Power.put(getGPM60ShooterPower(2.0), 0.0);
        Measured_Shooter_Speeds_From_Power.put(getGPM60ShooterPower(3.0), 0.0);
    }

    public double getMeasuredShootingSpeedFromPower(double power)   {
        return Measured_Shooter_Speeds_From_Power.get(power);
    }

    public double getGPM0Angle(double distance){
        return GPM_0_Angle.get(distance);
    }

    public double getGPM0IntakePower(double distance){
        return GPM_0_IntakePower.get(distance);
    }

    public double getGPM0ShooterPower(double distance){
        return GPM_0_ShooterPower.get(distance);
    }

    public double getGPM60Angle(double distance){
        return GPM_60_Angle.get(distance);
    }

    public double getGPM60IntakePower(double distance){
        return GPM_60_IntakePower.get(distance);
    }

    public double getGPM60ShooterPower(double distance){
        return GPM_60_ShooterPower.get(distance);
    }

}
