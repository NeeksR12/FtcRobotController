package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MecanumDrive {

    //Declarations
    public DcMotor leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive;
    public IMU imu;

    /**
     * Description: Initializes the drivetrain hardware
     * Pre-Condition: Must be given a hardware map with the correct names
     * Post-Condition: The software variables are linked up to the correct hardware components
     * @param hwMap The hardware map
     */
    public void init(HardwareMap hwMap) {
        // Initialize the hardware variables
        leftFrontDrive = hwMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hwMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hwMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hwMap.get(DcMotor.class, "right_back_drive");

        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Motor Mode
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Orientation
        imu = hwMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);

        imu.initialize(new IMU.Parameters(RevOrientation));
    }

    /**
     * Description: Moves the drivetrain oriented to the robot
     * Pre-Condition: Params must be doubles
     * Post-Condition: The drivetrain moves as commanded relative to the robot
     * @param axial The "Forwards" value
     * @param lateral The "Sideways" value
     * @param yaw The "Rotational" value
     */
    public void moveDrivetrain(double axial, double lateral, double yaw) {

        // Variables
        double max;
        double leftFrontPower;
        double leftBackPower;
        double rightFrontPower;
        double rightBackPower;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        leftFrontPower = (axial + lateral) + yaw;
        rightFrontPower = (axial - lateral) - yaw;
        leftBackPower = (axial - lateral) + yaw;
        rightBackPower = (axial + lateral) - yaw;

        // Normalize the values so no wheel power exceeds 100%
        max = JavaUtil.maxOfList(JavaUtil.createListWith(Math.abs(leftFrontPower),
                Math.abs(rightFrontPower), Math.abs(leftBackPower), Math.abs(rightBackPower)));
        if (max > 1) {
            leftFrontPower = leftFrontPower / max;
            rightFrontPower = rightFrontPower / max;
            leftBackPower = leftBackPower / max;
            rightBackPower = rightBackPower / max;
        }

        // Send calculated power to wheels.
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    /**
     * Description: Moves the drivetrain oriented to the field
     * Pre-Condition: Params must be doubles
     * Post-Condition: The drivetrain moves as commanded relative to the field
     * @param axial The initial field "Forwards" input
     * @param lateral The initial field "Sideways" input
     * @param yaw The "Rotational" input
     */
    public void driveField(double axial, double lateral, double yaw) {
        // Polar coordinates
        double theta = Math.atan2(axial, lateral);
        double r = Math.hypot(lateral, axial);

        theta = AngleUnit.normalizeRadians(theta -
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        // Cartesian coordinates
        double newAxial = r * Math.sin(theta);
        double newLateral = r * Math.cos(theta);

        moveDrivetrain(newAxial, newLateral, yaw);
    }

}
