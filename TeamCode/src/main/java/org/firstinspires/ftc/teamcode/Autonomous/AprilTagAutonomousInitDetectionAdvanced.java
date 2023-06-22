/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.HardwareMap.HardwareMap_CompetitionBot;
import org.firstinspires.ftc.teamcode.HardwareMap.HardwareMap_Holonomic;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class AprilTagAutonomousInitDetectionAdvanced extends LinearOpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    /* Declare OpMode members. */ // I inserted the below variables
    HardwareMap_CompetitionBot robot = new HardwareMap_CompetitionBot(); // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime runtimeLift = new ElapsedTime();
    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;


    final int highJunction = -8500;
    //final int level1 = -4530;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
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
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

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

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */ // This is where the robot moves to the right region of
        //the field based on the tag # it detected

        if (tagOfInterest == null || tagOfInterest.id == LEFT) {
            int state = 0;
            if (state == 0) {
                //init robot
                robot.init(hardwareMap);

                //Send telemetry message to signify robot waiting;
                telemetry.addData("Status", "Resetting Encoders");
                telemetry.update();

                //Send telemetry message to indicate successful Encoder reset
                telemetry.addData("Path0", "Starting at %7d :%7d",
                        robot.leftFront.getCurrentPosition(),
                        robot.rightFront.getCurrentPosition());
                telemetry.update();
                waitForStart();
                state = 1;
            }
            //clamp down on cone (it starts touching the grabbers)
            if (state == 1) {
                telemetry.addData("State", "1");
                telemetry.update();
                robot.rightClaw.setPosition(0);
                robot.leftClaw.setPosition(180);
                state = 2;
            }
            //strafe right (really left) into mid-substation
            if (state == 2) {
                telemetry.addData("State", "1");
                telemetry.update();
                strafeLeft(DRIVE_SPEED, 7);
                state = 3;
            }
            //drive forward into region 3 (with cone) --> **backward is forward now**
            if (state == 3) {
                telemetry.addData("State", "2");
                telemetry.update();
                encoderDrive(DRIVE_SPEED, -5.75, -5.75, -5.75, -5.75, 4.0);
                state = 4;
            }
            //strafe right (really left) to align with the high junction
            //if (state == 4) {
                //telemetry.addData("State", "2");
                //telemetry.update();
                //strafeLeft(DRIVE_SPEED, 1);
                //state = 5;
            //}
            //lift up slide system (with cone)

            if (state == 4) {
                telemetry.addData("State", "4");
                telemetry.update();
                //liftUp(3, 1);
                liftUpPosition(-10000, 1);
                state = 5;
            }
            //move forward a tad
            if (state == 8) {
                telemetry.addData("State", "2");
                telemetry.update();
                encoderDrive(DRIVE_SPEED, -0.5, -0.5, -0.5, -0.5, 4.0);
                state = 9;
            }
            //deposit cone on high junction (un-clamp grabbers)
            if (state == 7) {
                telemetry.addData("State", "2");
                telemetry.update();
                robot.rightClaw.setPosition(180);
                robot.leftClaw.setPosition(0);
                state = 8;
            }
            //back up 1 unit (to bring down lift)
            if (state == 8) {
                telemetry.addData("State", "2");
                telemetry.update();
                encoderDrive(DRIVE_SPEED, -1, -1, -1, -1, 4.0);
                state = 9;
            }
            //bring down lift
            if (state == 9) {
                telemetry.addData("State", "2");
                telemetry.update();
                //liftUp(3, -1);

                state = 10;
            }
            //strafe into region 1 (strafe left - but really right)
            if (state == 10) {
                telemetry.addData("State", "2");
                telemetry.update();
                //strafeRight(DRIVE_SPEED, 10);

                state = 11;
            }
            //stop robot
            if (state == 11) {
                telemetry.addData("State", "3");
                telemetry.update();
                stopMotors();
                state = 12;
            }

        } else if (tagOfInterest.id == MIDDLE) {
            int state = 0;
            if (state == 0) {
                //init robot
                robot.init(hardwareMap);

                // Send telemetry message to signify robot waiting;
                telemetry.addData("Status", "Resetting Encoders");    //
                telemetry.update();

                // Send telemetry message to indicate successful Encoder reset
                telemetry.addData("Path0", "Starting at %7d :%7d",
                        robot.leftFront.getCurrentPosition(),
                        robot.rightFront.getCurrentPosition());
                telemetry.update();
                waitForStart();
                state = 1;
            }
            //drive forward into region 2
            if (state == 1) {
                telemetry.addData("State", "1");
                telemetry.update();
                encoderDrive(DRIVE_SPEED, 6, 6, 6, 6, 4.0);
                state = 2;
            }
            //stop robot
            if (state == 2) {
                telemetry.addData("State", "2");
                telemetry.update();
                stopMotors();
                state = 3;
            }

        } else {
            int state = 0;
            if (state == 0) {
                //init robot
                robot.init(hardwareMap);

                // Send telemetry message to signify robot waiting;
                telemetry.addData("Status", "Resetting Encoders");    //
                telemetry.update();

                // Send telemetry message to indicate successful Encoder reset
                telemetry.addData("Path0", "Starting at %7d :%7d",
                        robot.leftFront.getCurrentPosition(),
                        robot.rightFront.getCurrentPosition());
                telemetry.update();
                waitForStart();
                state = 1;
            }
            //strafe right (really left) into mid-substation (to align with region 3)
            if (state == 1) {
                telemetry.addData("State", "1");
                telemetry.update();
                strafeLeft(DRIVE_SPEED, 6);
                state = 2;
            }
            //drive forward into region 3
            if (state == 2) {
                telemetry.addData("State", "2");
                telemetry.update();
                encoderDrive(DRIVE_SPEED, 6, 6, 6, 6, 4.0);
                state = 3;
            }
            //stop robot
            if (state == 3) {
                telemetry.addData("State", "3");
                telemetry.update();
                stopMotors();
                state = 4;
            }
        }
        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        //while (opModeIsActive()) {sleep(20);}
    }

    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches, double leftBackInches, double rightBackInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftFront.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.rightFront.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            newLeftBackTarget = robot.leftBack.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightBackTarget = robot.rightBack.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            robot.leftFront.setTargetPosition(newLeftTarget);
            robot.rightFront.setTargetPosition(newRightTarget);
            robot.leftBack.setTargetPosition(newLeftBackTarget);
            robot.rightBack.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftFront.setPower(Math.abs(speed));
            robot.rightFront.setPower(Math.abs(speed));
            robot.leftBack.setPower(Math.abs(speed));
            robot.rightBack.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftFront.isBusy() && robot.rightFront.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.leftFront.getCurrentPosition(),
                        robot.rightFront.getCurrentPosition());
                telemetry.update();
            }
            // Stop all motion;
            robot.leftFront.setPower(0);
            robot.rightFront.setPower(0);
            robot.leftBack.setPower(0);
            robot.rightBack.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    public void stopMotors() {
        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);
    }

    public void strafeLeft(double power, int distance) {
        Orientation angles;
        double error;
        double k = 3 / 360.0;
        double startAngle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        int leftFrontTarget = robot.leftFront.getCurrentPosition() - (int) (distance * COUNTS_PER_INCH);
        int rightFrontTarget = robot.rightFront.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
        int leftBackTarget = robot.leftBack.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
        int rightBackTarget = robot.rightBack.getCurrentPosition() - (int) (distance * COUNTS_PER_INCH);
        robot.leftFront.setTargetPosition(leftFrontTarget);
        robot.rightFront.setTargetPosition(rightFrontTarget);
        robot.leftBack.setTargetPosition(leftBackTarget);
        robot.rightBack.setTargetPosition(rightBackTarget);

        while (opModeIsActive()
                && (robot.leftFront.getCurrentPosition() > leftFrontTarget && robot.rightFront.getCurrentPosition() < rightFrontTarget && robot.leftBack.getCurrentPosition() < leftBackTarget && robot.rightBack.getCurrentPosition() > rightBackTarget)) {
            angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            //finds the angle given by the imu [-180, 180]
            double angle = angles.firstAngle;
            error = startAngle - angle;
            robot.leftFront.setPower(-(power + (error * k)));
            robot.rightFront.setPower((power + (error * k)));
            robot.leftBack.setPower((power - (error * k)));
            robot.rightBack.setPower(-(power - (error * k)));
            telemetry.addData("error: ", error);
            telemetry.addData("leftfront dest: ", leftFrontTarget);
            telemetry.addData("leftFront pos: ", robot.leftFront.getCurrentPosition());
            telemetry.update();
        }
        stopMotors();
    }

    public void strafeRight(double power, int distance) {
        Orientation angles;
        double error;
        double k = 3 / 360.0;
        double startAngle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        int leftFrontTarget = robot.leftFront.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
        int rightFrontTarget = robot.rightFront.getCurrentPosition() - (int) (distance * COUNTS_PER_INCH);
        int leftBackTarget = robot.leftBack.getCurrentPosition() - (int) (distance * COUNTS_PER_INCH);
        int rightBackTarget = robot.rightBack.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
        robot.leftFront.setTargetPosition(leftFrontTarget);
        robot.rightFront.setTargetPosition(rightFrontTarget);
        robot.leftBack.setTargetPosition(leftBackTarget);
        robot.rightBack.setTargetPosition(rightBackTarget);

        while (opModeIsActive()
                && (robot.leftFront.getCurrentPosition() < leftFrontTarget && robot.rightFront.getCurrentPosition() > rightFrontTarget && robot.leftBack.getCurrentPosition() > leftBackTarget && robot.rightBack.getCurrentPosition() < rightBackTarget)) {
            angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            //finds the angle given by the imu [-180, 180]
            double angle = angles.firstAngle;
            error = startAngle - angle;
            robot.leftFront.setPower((power - (error * k)));
            robot.rightFront.setPower(-(power - (error * k)));
            robot.leftBack.setPower(-(power + (error * k)));
            robot.rightBack.setPower((power + (error * k)));
            telemetry.addData("error: ", error);
            telemetry.addData("leftfront dest: ", leftFrontTarget);
            telemetry.addData("leftFront pos: ", robot.leftFront.getCurrentPosition());
            telemetry.update();
        }
        stopMotors();
    }

    public void liftUp(double liftTime, double liftSpeed) {
        runtimeLift.reset();
        while (opModeIsActive() &&
                (runtimeLift.seconds() < liftTime)) {
            robot.slideSystem.setPower(liftSpeed);
        }
        robot.slideSystem.setPower(0);
    }

    public void liftUpPosition(int position, double liftSpeed) {

        robot.slideSystem.setTargetPosition(position);
        robot.slideSystem.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.slideSystem.setPower(Math.abs(liftSpeed));

    }
}