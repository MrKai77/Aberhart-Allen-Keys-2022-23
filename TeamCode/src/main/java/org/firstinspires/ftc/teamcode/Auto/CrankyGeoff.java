package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class CrankyGeoff extends LinearOpMode {

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;
    DcMotor armMotor;

    static final double FEET_PER_METER = 3.28084;

    // This is fancy stuff, idk man I don't think this needs to be changed.
    // If it keeps being finicky than maybe.

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagSize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family
    static final int LEFT = 1;
    static final int MIDDLE = 2;
    static final int RIGHT = 3;

    @Override
    public void runOpMode() {

        // -- Initializing camera --
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagSize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        // -- Initializing motors --
        frontLeftMotor = hardwareMap.get(DcMotor.class, "FrontLeft");
        frontRightMotor = hardwareMap.get(DcMotor.class, "FrontRight");
        backLeftMotor = hardwareMap.get(DcMotor.class, "BackLeft");
        backRightMotor = hardwareMap.get(DcMotor.class, "BackRight");

        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        armMotor = hardwareMap.get(DcMotor.class, "ArmMotor");

        waitForStart();

        // Raises arm so that tag can be read
        armMotor.setPower(1);
        sleep(1000);
        armMotor.setPower(0.1);

        moveRobotToPosition(findTag());
    }

    int findTag() {
        AprilTagDetection tagOfInterest = null;

        while (tagOfInterest == null) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {    // If we see a tag
                boolean tagFound = false;

                for (AprilTagDetection tag: currentDetections) {
                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see the right tags :(");

                    if (tagOfInterest != null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nWe HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {    // If we don't see a tag
                telemetry.addLine("Don't see any tags :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        telemetry.addLine("Tag snapshot:\n");
        tagToTelemetry(tagOfInterest);
        telemetry.update();

        return tagOfInterest.id;
    }

    void moveRobotToPosition(int position) {
        switch (position) {
            case RIGHT:
                frontLeftMotor.setPower(0.5);
                frontRightMotor.setPower(0.5);
                backLeftMotor.setPower(0.5);
                backRightMotor.setPower(0.5);

                sleep(200);

                frontLeftMotor.setPower(0.5);
                frontRightMotor.setPower(0.5);
                backLeftMotor.setPower(-0.5);
                backRightMotor.setPower(-0.5);

                sleep(2600);

                frontLeftMotor.setPower(0.5);
                frontRightMotor.setPower(0.5);
                backLeftMotor.setPower(0.5);
                backRightMotor.setPower(0.5);

                sleep(1350);

                frontLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                backLeftMotor.setPower(0);
                backRightMotor.setPower(0);
                break;

            case LEFT:
                frontLeftMotor.setPower(0.5);
                frontRightMotor.setPower(0.5);
                backLeftMotor.setPower(0.5);
                backRightMotor.setPower(0.5);

                sleep(200);

                frontLeftMotor.setPower(-0.5);
                frontRightMotor.setPower(-0.5);
                backLeftMotor.setPower(0.5);
                backRightMotor.setPower(0.5);

                sleep(2500);

                frontLeftMotor.setPower(0.5);
                frontRightMotor.setPower(0.5);
                backLeftMotor.setPower(0.5);
                backRightMotor.setPower(0.5);

                sleep(1600);

                frontLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                backLeftMotor.setPower(0);
                backRightMotor.setPower(0);
                break;

            default:    // Middle/no tag
                frontLeftMotor.setPower(0.5);
                frontRightMotor.setPower(0.5);
                backLeftMotor.setPower(0.5);
                backRightMotor.setPower(0.5);

                sleep(1800);

                frontLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                backLeftMotor.setPower(0);
                backRightMotor.setPower(0);
                break;
        }
    }

    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}
