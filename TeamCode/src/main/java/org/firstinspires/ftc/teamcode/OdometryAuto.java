package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Click This One")

public class OdometryAuto extends LinearOpMode {
    private PIDController controller;

    public static double p = 0.014, i = 0, d = 0.0001;
    public static double f = 0.25;
    public static int startingPos;
    private final double ticks_in_degree = 537.7/180.0;

    private DcMotorEx leftLinearMotor;
    private DcMotorEx rightLinearMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        PinpointDrive drive = new PinpointDrive(hardwareMap, new Pose2d(0, 0, 0));
        Servo leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        Servo rightClaw = hardwareMap.get(Servo.class, "rightClaw");

        Servo rightWrist = hardwareMap.get(Servo.class, "rightWrist");
        Servo leftWrist = hardwareMap.get(Servo.class, "leftWrist");
        CRServo rightIntake = hardwareMap.crservo.get("rightIntake");
        CRServo leftIntake = hardwareMap.crservo.get("leftIntake");
        Servo rightIntakeWrist = hardwareMap.get(Servo.class, "rightIntakeWrist");
        Servo leftIntakeWrist = hardwareMap.get(Servo.class, "leftIntakeWrist");

        Servo LeftHorizontalSlide = hardwareMap.get(Servo.class, "leftHorizontalSlide");
        Servo RightHorizontalSlide = hardwareMap.get(Servo.class, "rightHorizontalSlide");

        Servo leftTransferArm = hardwareMap.get(Servo.class, "leftTransferArm");
        Servo rightTransferArm = hardwareMap.get(Servo.class, "rightTransferArm");

        controller = new PIDController(p, i, d);

        leftLinearMotor = hardwareMap.get(DcMotorEx.class, "leftLinearSlide");
        rightLinearMotor = hardwareMap.get(DcMotorEx.class, "rightLinearSlide");

        leftLinearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLinearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        startingPos = leftLinearMotor.getCurrentPosition();
        leftLinearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLinearMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        rightLinearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLinearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLinearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        LeftHorizontalSlide.setPosition(0);
        RightHorizontalSlide.setPosition(0.95);

        rightWrist.setPosition(0.7);
        leftWrist.setPosition(0.3);

        leftIntakeWrist.setPosition(0.75);
        rightIntakeWrist.setPosition(0.25);

        leftTransferArm.setPosition(0.25);
        rightTransferArm.setPosition(0);

        leftClaw.setPosition(0);
        rightClaw.setPosition(1);

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0, 0, 0))
                        .strafeToSplineHeading(new Vector2d(8, 15), -(Math.toRadians(45)))
                        .stopAndAdd(new ServoAction(leftClaw, rightClaw, 0.8, 0.2))
                        .waitSeconds(0.5)
                        .stopAndAdd(new SlideAction(4150))
                        .stopAndAdd(new ServoAction(leftTransferArm, rightTransferArm, 0.9, 0))
                        .waitSeconds(1)
                        .stopAndAdd(new ServoAction(leftClaw, rightClaw, 0.2, 0.8))
                        .waitSeconds(0.25)
                        .stopAndAdd(new ServoAction(leftTransferArm, rightTransferArm, 0.25, 0))
                        .stopAndAdd(new SlideAction(startingPos))
                        //
                        .stopAndAdd(new ServoAction(leftWrist, rightWrist, 0.8, 0.2))
                        .stopAndAdd(new ServoAction(leftIntakeWrist, rightIntakeWrist, 0.4, 0.6))
                        .stopAndAdd(new CRServoAction(leftIntake, rightIntake, -1, 1))
                        .strafeToSplineHeading(new Vector2d(10, 9), (Math.toRadians(0)))
                        .strafeToSplineHeading(new Vector2d(30, 9), (Math.toRadians(0)))
//                        //
                        .stopAndAdd(new ServoAction(leftWrist, rightWrist, 0.3, 0.7))
                        .stopAndAdd(new ServoAction(leftIntakeWrist, rightIntakeWrist, 0.75, 0.25))
                        .strafeToSplineHeading(new Vector2d(8, 15), -(Math.toRadians(45)))
                        .stopAndAdd(new ServoAction(leftClaw, rightClaw, 0.8, 0.2))
                        .waitSeconds(0.5)
                        .stopAndAdd(new SlideAction(4150))
                        .stopAndAdd(new ServoAction(leftTransferArm, rightTransferArm, 0.9, 0))
                        .waitSeconds(1)
                        .stopAndAdd(new ServoAction(leftClaw, rightClaw, 0.2, 0.8))
                        .waitSeconds(0.25)
                        .stopAndAdd(new ServoAction(leftTransferArm, rightTransferArm, 0.25, 0))
                        .stopAndAdd(new SlideAction(startingPos))
//                        //
                        .stopAndAdd(new ServoAction(leftWrist, rightWrist, 0.8, 0.2))
                        .stopAndAdd(new ServoAction(leftIntakeWrist, rightIntakeWrist, 0.4, 0.6))
                        .stopAndAdd(new CRServoAction(leftIntake, rightIntake, -1, 1))
                        .strafeToSplineHeading(new Vector2d(10, 20), (Math.toRadians(0)))
                        .strafeToSplineHeading(new Vector2d(30, 20), (Math.toRadians(0)))
//                        //
                        .stopAndAdd(new ServoAction(leftWrist, rightWrist, 0.3, 0.7))
                        .stopAndAdd(new ServoAction(leftIntakeWrist, rightIntakeWrist, 0.75, 0.25))
                        .strafeToSplineHeading(new Vector2d(8, 15), -(Math.toRadians(45)))
                        .stopAndAdd(new ServoAction(leftClaw, rightClaw, 0.8, 0.2))
                        .waitSeconds(0.5)
                        .stopAndAdd(new SlideAction(4150))
                        .stopAndAdd(new ServoAction(leftTransferArm, rightTransferArm, 0.9, 0))
                        .waitSeconds(1)
                        .stopAndAdd(new ServoAction(leftClaw, rightClaw, 0.2, 0.8))
                        .waitSeconds(0.25)
                        .stopAndAdd(new ServoAction(leftTransferArm, rightTransferArm, 0.25, 0))
                        .stopAndAdd(new SlideAction(startingPos))
                       // DONT
//                        .strafeTo(new Vector2d(0, 10))
//                        .strafeTo(new Vector2d(3, 0))
//                        //
//                        .turn(Math.PI / 2)
//                        .strafeTo(new Vector2d(50, 0))
//                        .strafeTo(new Vector2d(50, 8))
//                        .strafeTo(new Vector2d(2, 8))
//                        //
//                        .strafeTo(new Vector2d(50, 8))
//                        .strafeTo(new Vector2d(50, 16))
//                        .strafeTo(new Vector2d(7, 16))
//                        //
//                        .strafeTo(new Vector2d(50, 18))
//                        .strafeTo(new Vector2d(50, 23))
//                        .strafeTo(new Vector2d(6, 23))

                        .build());
    }

    public class SlideAction implements Action {
        int target;

        ElapsedTime timer;

        public SlideAction(int t) {
            this.target = t;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (timer == null) {
                timer = new ElapsedTime();
            }

            controller.setPID(p, i, d);
            int slidePos = leftLinearMotor.getCurrentPosition();

            double pid = controller.calculate(slidePos, target);

            double power = pid + f;

            leftLinearMotor.setPower(power);
            rightLinearMotor.setPower(power);

            return timer.seconds() < 2.5;
        }
    }
    public class ServoAction implements Action {
        Servo leftServo;
        Servo rightServo;
        double positionLeft;

        double positionRight;

        public ServoAction(Servo l, Servo r, double pL, double pR) {
            this.leftServo = l;
            this.rightServo = r;
            this.positionLeft = pL;
            this.positionRight = pR;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            leftServo.setPosition(positionLeft);
            rightServo.setPosition(positionRight);

            return false;
        }
    }

    public class CRServoAction implements Action {
        CRServo leftServo;
        CRServo rightServo;
        double powerLeft;

        double powerRight;

        public CRServoAction(CRServo l, CRServo r, double pL, double pR) {
            this.leftServo = l;
            this.rightServo = r;
            this.powerLeft = pL;
            this.powerRight = pR;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            leftServo.setPower(powerLeft);
            rightServo.setPower(powerRight);

            return false;
        }
    }
}
