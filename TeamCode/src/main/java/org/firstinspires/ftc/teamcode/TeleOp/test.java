package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Disabled
public class test extends LinearOpMode {

    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;

    private DcMotor armMotor;

    private Servo rightClaw;
    private Servo leftClaw;

    @Override
    public void runOpMode() {
        frontLeftMotor = hardwareMap.get(DcMotor.class, "FrontLeft");
        frontRightMotor = hardwareMap.get(DcMotor.class, "FrontRight");
        backLeftMotor = hardwareMap.get(DcMotor.class, "BackLeft");
        backRightMotor = hardwareMap.get(DcMotor.class, "BackRight");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        armMotor = hardwareMap.get(DcMotor.class, "ArmMotor");
        rightClaw = hardwareMap.get(Servo.class, "RightClaw");
        leftClaw =  hardwareMap.get(Servo.class, "LeftClaw");

        waitForStart();

        while (opModeIsActive()) {
            setDriveTrainSpeeds();
            setArmStates();
        }
    }

    private void setArmStates() {
        setClawStates();

        armMotor.setPower(0.1 + (gamepad1.right_trigger - gamepad1.left_trigger / 1.5));
    }

//    private void setArmPosition(Boolean up) {
//        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        if (up) {
//            armMotor.setTargetPosition(80);
//        }
//        else {
//            armMotor.setTargetPosition(-80);
//        }
//
//        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        armMotor.setPower(0.9);
//    }

    private void setClawStates() {
        if (gamepad1.a) {    // Open claw
            rightClaw.setPosition(0.7);
            leftClaw.setPosition(0.3);

            armMotor.setPower(-0.5);
            sleep(200);
            armMotor.setPower(0);

            sleep(100);

            frontLeftMotor.setPower(-0.8);
            backLeftMotor.setPower(-0.8);
            frontRightMotor.setPower(-0.8);
            backRightMotor.setPower(-0.8);

            sleep(300);

            frontLeftMotor.setPower(0);
            backLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backRightMotor.setPower(0);

            rightClaw.setPosition(0);
            leftClaw.setPosition(1);
        }
        if (gamepad1.b) {    // Close claw (lowers arm first)
            rightClaw.setPosition(0);
            leftClaw.setPosition(1);
        }
        if (gamepad1.x) {    // Open claw
            rightClaw.setPosition(0.7);
            leftClaw.setPosition(0.3);
        }
    }

    private void setDriveTrainSpeeds() {
        double FLMotorSpeed;
        double FRMotorSpeed;
        double BLMotorSpeed;
        double BRMotorSpeed;

        if (gamepad1.dpad_right) {
            FLMotorSpeed = -1;
            FRMotorSpeed = 1;
            BRMotorSpeed = 1;
            BLMotorSpeed = -1;
        }
        else if (gamepad1.dpad_left) {
            FLMotorSpeed = 1;
            FRMotorSpeed = -1;
            BRMotorSpeed = -1;
            BLMotorSpeed = 1;
        }
        else {
            FLMotorSpeed = gamepad1.left_stick_y;
            BLMotorSpeed = gamepad1.left_stick_y;

            FRMotorSpeed = gamepad1.right_stick_y;
            BRMotorSpeed = gamepad1.right_stick_y;
        }

        double motorSpeedDivider = getMotorSpeedDivider();

        frontLeftMotor.setPower(FLMotorSpeed / motorSpeedDivider);
        backLeftMotor.setPower(BLMotorSpeed / motorSpeedDivider);
        frontRightMotor.setPower(FRMotorSpeed / motorSpeedDivider);
        backRightMotor.setPower(BRMotorSpeed / motorSpeedDivider);
    }

    private Double getMotorSpeedDivider() {
        double motorSpeedDivider = 1.5;

        if (gamepad1.right_bumper) {
            motorSpeedDivider = 1;
        }
        else if (gamepad1.left_bumper) {
            motorSpeedDivider = 3;
        }
        return motorSpeedDivider;
    }
}