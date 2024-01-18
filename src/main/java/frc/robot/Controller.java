package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.Constants.OIConstants;

public class Controller extends Joystick {

    double dx; // X deadband for performance
    double dy; // Y deadband for performance
    double dm; // Omega deadband for performance
    boolean cubeControllerLeftStick; // If false use linear transformation from joystick input to power
    // If true use cube transformation
    // This concept was taken from nu23 of team 125 code.
    boolean cubeControllerRightStick;

    // The next three variables will contain cube values of deadbands used for cube driving.
    // This is done for performance reasons to reduce periodic calculations for cube driving.
    private double cubeDeadbandX;
    private double cubeDeadbandY;
    private double cubeDeadbandOmega;

    private XboxController xboxController;

    public Controller() {
        super(OIConstants.XBOX_CONTROLLER_PORT); // This needs to be done because the joystick parent class has a non-default constructor
        this.dx = OIConstants.XBOX_DEADBAND_X;
        this.dy = OIConstants.XBOX_DEADBAND_Y;
        this.dm = OIConstants.XBOX_DEADBAND_OMEGA;

        /**
         * These flags determine whether a particular joystick should use cube driving, which will use not a linear input (e.g. a value from joystick axis)
         * but a cube of that value. Since the joystick axis generates values from -1..1, the cube of that value in that range also will be between -1..1,
         * but it will provide much greater precision for the low speeds (in other words, will make speed adjustments in low speed range easier to make)
         * at the expense of the high-speed range. Since the low speeds are often used when the robot needs to move very short distances, such change will
         * make small teleop position changes easier to do. The algorithm that we "borrowed" from team 125 nu23 code properly accounts for the
         * deadband range as well. We optimized it by reducing the need for recurring calculations and made it a bit more readable by using ternary operators.
         */
        this.cubeControllerLeftStick = OIConstants.XBOX_CUBE_CONTROLLER_LEFT_STICK;
        this.cubeControllerRightStick = OIConstants.XBOX_CUBE_CONTROLLER_RIGHT_STICK;

        this.cubeDeadbandX = dx * dx * dx;
        this.cubeDeadbandY = dy * dy * dy;
        this.cubeDeadbandOmega = dm * dm * dm;

        // Assuming XboxController is used for Xbox, change this if another type is introduced
        this.xboxController = new XboxController(OIConstants.XBOX_CONTROLLER_PORT);
    }

    public double getLeftStickY() {
        double rawY = this.getY();
        double result;

        if (this.cubeControllerLeftStick) {
            double cubeY = rawY * rawY * rawY;
            result = (cubeY - (rawY > 0 ? 1 : -1) * cubeDeadbandY) / (1 - cubeDeadbandY); // cubeController
            result = Math.abs(result) > this.cubeDeadbandY ? result : 0; // Ignores range of deadband values
        } else {
            result = (MathUtil.applyDeadband(rawY, dy)); // linear controller values
        }

        return result;
    }

    public double getLeftStickX() {
        double rawX = this.getX();
        double result;

        if (this.cubeControllerLeftStick) {
            double cubeX = rawX * rawX * rawX;
            result = (cubeX - (rawX > 0 ? 1 : -1) * cubeDeadbandX) / (1 - cubeDeadbandX); // cubeController
            result = Math.abs(result) > this.cubeDeadbandX ? result : 0; // Ignores range of deadband values
        } else {
            result = (MathUtil.applyDeadband(rawX, dx)); // linear controller values
        }

        return result;
    }

    public double getLeftStickOmega() {
        double rawOmega = this.getTwist();
        double result;

        if (this.cubeControllerRightStick) {
            double cubeOmega = rawOmega * rawOmega * rawOmega;
            result = (cubeOmega - (rawOmega > 0 ? 1 : -1) * cubeDeadbandOmega) / (1 - cubeDeadbandOmega); // cubeController
            result = Math.abs(result) > this.cubeDeadbandOmega ? result : 0; // Ignores range of deadband values
        } else {
            result = (MathUtil.applyDeadband(rawOmega, dm)); // linear controller values
        }

        return result;
    }

    public double getLeftStickY_Xbox() {
        return xboxController.getRawAxis(3); //TODO: Need to check axis number
    }

    public double getLeftStickX_Xbox() {
        return xboxController.getRawAxis(4); //TODO: Need to check axis number
    }

    public double getLeftStickOmega_Xbox() {
        return xboxController.getRawAxis(5); //TODO: Need to check axis number
    }
}

