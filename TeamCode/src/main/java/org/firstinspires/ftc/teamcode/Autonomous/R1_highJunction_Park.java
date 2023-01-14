package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.HardwareMap.HardwareMap_Holonomic;
import org.firstinspires.ftc.teamcode.HardwareMap.HardwareMap_CompetitionBot;


/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="R1 highJunction Park", group="Red")
//@Disabled
public class R1_highJunction_Park extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareMap_CompetitionBot robot = new HardwareMap_CompetitionBot(); // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime runtimeLift = new ElapsedTime();
    //private ElapsedTime runtimeClaw = new ElapsedTime();
    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    @Override
    public void runOpMode() {
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
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
       //speed -1 goes up; speed 1 goes down
        if (state == 1) {
            telemetry.addData("State", "4");
            telemetry.update();
            liftUp(2, -1);
            //setSlideSystemPosition();
            state = 2;
        }
        //forward 9
        if (state == 1) {
            telemetry.addData("State", "1");
            telemetry.update();
            encoderDrive(DRIVE_SPEED, 9, 9, 9, 9, 4.0);
            state = 2;
        }
        //turn left
        if (state == 2) {
            telemetry.addData("State", "4");
            telemetry.update();
            encoderDrive(DRIVE_SPEED, -5, 5, -5, -5, 4.0);
            state = 3;
        }
        //forward 5
        if (state == 3) {
            telemetry.addData("State", "4");
            telemetry.update();
            encoderDrive(DRIVE_SPEED, 5, 5, 5, 5, 4.0);
            state = 4;
        }
        //lift to top cone in stack (of 5 cones)

        //claws clamp
        if (state == 5) {
            telemetry.addData("State", "4");
            leftClaw(0);
            rightClaw(0);
            telemetry.update();
            state = 6;
        }
        //lift a bit to get over stack
        if (state == 6) {
            telemetry.addData("State", "4");
            telemetry.update();
            //encoderDrive(DRIVE_SPEED, 5, 5, 5, 5, 4.0);
            liftUp(0.125, 0.5);
            state = 7;
        }
        //turn right
        if (state == 7) {
            telemetry.addData("State", "4");
            telemetry.update();
            encoderDrive(DRIVE_SPEED, 5, -5, 5, -5, 4.0);
            state = 8;
        }
        //strafe right
        if (state == 8) {
            telemetry.addData("State", "4");
            telemetry.update();
            strafeLeft(DRIVE_SPEED, 7);
            state = 9;
        }
        //lift completely to high junction
        if (state == 9) {
            telemetry.addData("State", "4");
            telemetry.update();
            liftUp(1, 0.5);
            state = 11;
        }
        //release claws
        if (state == 11) {
            telemetry.addData("State", "4");
            telemetry.update();
            leftClaw(180);
            rightClaw(180);
            state = 12;
        }
        //move lift down; strafe right
        if (state == 12) {
            telemetry.addData("State", "4");
            telemetry.update();
            liftUp(9, -1);
            strafeRight(DRIVE_SPEED, 8);
            state = 13;
        }
        //back into parking spot
        if (state == 13) {
            telemetry.addData("State", "4");
            telemetry.update();
            encoderDrive(DRIVE_SPEED, 9, 9, 9, 9, 4.0);
            state = 14;
        }
        //stop motors + servos
        if (state == 14) {
            telemetry.addData("State", "4");
            telemetry.update();
            stopMotors();
            state = 15;
        }
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
        public void encoderDrive ( double speed,
        double leftInches, double rightInches, double leftBackInches, double rightBackInches,
        double timeoutS){
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
        public void stopMotors () {
            robot.leftFront.setPower(0);
            robot.leftBack.setPower(0);
            robot.rightFront.setPower(0);
            robot.rightBack.setPower(0);
        }
        public void turnLeft ( double power){
            robot.leftFront.setPower(-power);
            robot.rightFront.setPower(power);
            robot.leftBack.setPower(-power);
            robot.rightBack.setPower(power);
        }
        public void strafeLeft ( double power, int distance){
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

        public void strafeRight ( double power, int distance){
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
        public void liftUp ( double liftTime, double liftSpeed){
            runtimeLift.reset();
            while (opModeIsActive() &&
                    (runtimeLift.seconds() < liftTime)) {
                robot.slideSystem.setPower(liftSpeed);
            }
            robot.slideSystem.setPower(0);
        }
        public void leftClaw ( double leftClawPosition){
            while (opModeIsActive())
                robot.leftClaw.setPosition(leftClawPosition);
        }

        public void rightClaw ( double rightClawPosition){
            while (opModeIsActive())
                robot.rightClaw.setPosition(rightClawPosition);
        }
        public void setSlideSystemPosition ( int position, double speed) {
            robot.slideSystem.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.slideSystem.setTargetPosition(position);
            robot.slideSystem.setPower(speed);

            while ((robot.slideSystem.getCurrentPosition() > robot.slideSystem.getTargetPosition() + 1
                    || robot.slideSystem.getCurrentPosition() < robot.slideSystem.getTargetPosition() - 1)
                    && opModeIsActive()) {
                telemetry.addData("Encoder Position", robot.slideSystem.getCurrentPosition());
                telemetry.update();
                idle();
            }
            robot.slideSystem.setPower(0);
        }
}



