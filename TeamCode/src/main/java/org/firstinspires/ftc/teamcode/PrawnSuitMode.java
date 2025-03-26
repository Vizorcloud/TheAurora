package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
@TeleOp(name = "Prawn Suit Mode")
public class PrawnSuitMode extends LinearOpMode {
    private PIDController controller;

    public static double p = 0.014, i = 0, d = 0.0001;
    public static double f = 0.25;
    public static int target = 0;
    public static int startingPos;

    public static boolean extended = false;

    private final double ticks_in_degree = 537.7/180.0;

    private DcMotorEx leftLinearMotor;
    private DcMotorEx rightLinearMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration

        int slidesTargetPosition = 0;

        //Expansion Hub
        Servo rightWrist = hardwareMap.get(Servo.class, "rightWrist");
        Servo leftWrist = hardwareMap.get(Servo.class, "leftWrist");
        CRServo rightIntake = hardwareMap.crservo.get("rightIntake");
        CRServo leftIntake = hardwareMap.crservo.get("leftIntake");
        Servo rightIntakeWrist = hardwareMap.get(Servo.class, "rightIntakeWrist");
        Servo leftIntakeWrist = hardwareMap.get(Servo.class, "leftIntakeWrist");
        //

        //Control Hub
        Servo leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        Servo leftTransferArm = hardwareMap.get(Servo.class, "leftTransferArm");
        Servo rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        Servo rightTransferArm = hardwareMap.get(Servo.class, "rightTransferArm");
        Servo LeftHorizontalSlide = hardwareMap.get(Servo.class, "leftHorizontalSlide");
        Servo RightHorizontalSlide = hardwareMap.get(Servo.class, "rightHorizontalSlide");
        //

        DcMotor LeftFrontMotor = hardwareMap.dcMotor.get("frontLeft");
        DcMotor LeftBackMotor = hardwareMap.dcMotor.get("backLeft");
        DcMotor RightFrontMotor = hardwareMap.dcMotor.get("frontRight");
        DcMotor RightBackMotor = hardwareMap.dcMotor.get("backRight");

        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        leftLinearMotor = hardwareMap.get(DcMotorEx.class, "leftLinearSlide");
        rightLinearMotor = hardwareMap.get(DcMotorEx.class, "rightLinearSlide");

        leftLinearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLinearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        startingPos = leftLinearMotor.getCurrentPosition();
        target = startingPos;
        leftLinearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLinearMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        rightLinearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLinearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLinearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        RightBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            double y = -gamepad2.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad2.left_stick_x;
            double rx = gamepad2.right_stick_x;

            controller.setPID(p, i, d);
            int slidePos = leftLinearMotor.getCurrentPosition();
            double pid = controller.calculate(slidePos, target);

            double power = pid + f;

            leftLinearMotor.setPower(power);
            rightLinearMotor.setPower(power);

            telemetry.addData("pos: ", startingPos);
            telemetry.addData("target: ", target);
            telemetry.update();
            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad2.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            RightFrontMotor.setPower(frontRightPower*0.9);
            RightBackMotor.setPower(backRightPower*0.9);
            LeftFrontMotor.setPower(frontLeftPower*0.9);
            LeftBackMotor.setPower(backLeftPower*0.9);

            telemetry.addData("Left Slide: ", leftLinearMotor.getCurrentPosition());
            telemetry.addData("Right Slide: ", rightLinearMotor.getCurrentPosition());

            telemetry.addData("Left Target: ", slidesTargetPosition);
            telemetry.addData("Right Target: ", slidesTargetPosition);

            telemetry.addData("Left Power: ", leftLinearMotor.getPower());
            telemetry.addData("Right Power: ", rightLinearMotor.getPower());

            // double p = 0.025;
            // double target = slidesTargetPosition;
            // double error = target - leftLinearMotor.getCurrentPosition();
            // leftLinearMotor.setPower(error * p);
            // rightLinearMotor.setPower(error * p);

            if (gamepad2.right_bumper) {

                LeftHorizontalSlide.setPosition(0.7);
                RightHorizontalSlide.setPosition(0.3);

                //

                rightWrist.setPosition(0);
                leftWrist.setPosition(1);

                leftIntakeWrist.setPosition(0.45);
                rightIntakeWrist.setPosition(0.55);
            } else {
                LeftHorizontalSlide.setPosition(0.45);
                RightHorizontalSlide.setPosition(0.55);

                //

                rightWrist.setPosition(0.5);
                leftWrist.setPosition(0.5);

                leftIntakeWrist.setPosition(0.7);
                rightIntakeWrist.setPosition(0.3);
            }

//            if (gamepad2.circle) {
//                Wrist.setPosition(1);
//
//            } else {
//                Wrist.setPosition(0.2);
//            }

            if (gamepad2.square) {
                rightIntake.setPower(1);
                leftIntake.setPower(-1);
            } else if (gamepad2.cross) {
                rightIntake.setPower(-1);
                leftIntake.setPower(1);
            } else {
                rightIntake.setPower(0);
                leftIntake.setPower(0);
            }

//            if (gamepad2.dpad_up) {
//                leftTransferArm.setPosition(0.1);
//                rightTransferArm.setPosition(0.9);
//            } else {
//                leftTransferArm.setPosition(0.9);
//                rightTransferArm.setPosition(0.1);
//            }

            if (gamepad2.dpad_up) {
                leftTransferArm.setPosition(0);
                rightTransferArm.setPosition(1);
            } else {
                leftTransferArm.setPosition(0.85);
                rightTransferArm.setPosition(0.15);
            }

            if (gamepad2.circle) {
                leftClaw.setPosition(0.8);
                rightClaw.setPosition(0.2);
            } else {
                leftClaw.setPosition(0.1);
                rightClaw.setPosition(0.9);
            }

//            if (gamepad2.triangle) {
//                if (extended) {
//                    extended = false;
//                } else {
//                    extended = true;
//                }
//
//            }
//

//
//            if (gamepad2.left_bumper) {
//                Claw.setPosition(0.4);
//            } else {
//                Claw.setPosition(0.1);
//            }
//
//            //Linear Extension
//            if (gamepad2.dpad_up) {
//                ArmRight.setPosition(0.9);
//                ArmLeft.setPosition(0.1);
////             } else if (gamepad2.dpad_down) {
////                ArmRight.setPosition(0.24);
////                ArmLeft.setPosition(0.76);
//            } else {
//                ArmRight.setPosition(0.1);
//                ArmLeft.setPosition(0.9);
//            }

            if (gamepad2.right_trigger != 0) {
                target = 4200;
            } else {
                target = startingPos;
            }
        }
    }
}