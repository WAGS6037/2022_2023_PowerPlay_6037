package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.HardwareMap.HardwareMap_CompetitionBot;

// name this OpMode and determine a group
@TeleOp (name="CompetitionBot", group="TeleOP")
public class CompetitionBot extends OpMode {

    /* Declare OpMode members. */
    HardwareMap_CompetitionBot robot       = new HardwareMap_CompetitionBot();

    @Override
    public void init() {

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        robot.lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello WAGS Driver!!");    //

    }
    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

    }

    @Override
    public void loop() {

        double speedDuck = 0.5;
        double rStickX;
        double rStickY;
        double lStickX;
        double targetAngle;
        double mag1;
        double mag2;
        double rotationPower;
        double maxPower;
        double scaleDown;

        //gamepad 1 driving
        rStickX = gamepad1.right_stick_x;
        rStickY = -gamepad1.right_stick_y;
        lStickX = gamepad1.left_stick_x;

        targetAngle = (Math.atan2(rStickY,rStickX));

        rotationPower = -lStickX;
        mag1 = Math.sqrt(Math.pow(rStickX,2) + Math.pow(rStickY,2)) * (Math.sin(targetAngle + Math.PI / 4));
        mag2 = Math.sqrt(Math.pow(rStickX,2) + Math.pow(rStickY,2)) * (Math.sin(targetAngle - Math.PI / 4));

        maxPower = Math.max(Math.abs(mag1) +  Math.abs(rotationPower) , Math.abs(mag2) +  Math.abs(rotationPower)) + 0.15;
        scaleDown = 1.0;

        if (maxPower > 1)
            scaleDown = 1.0 / maxPower;


        robot.leftFront.setPower((mag2 - rotationPower) * scaleDown);
        robot.rightFront.setPower((mag1 + rotationPower) * scaleDown);
        robot.leftBack.setPower((mag1 - rotationPower) * scaleDown);
        robot.rightBack.setPower((mag2 + rotationPower) * scaleDown);

        //end of gamepad driving 1
        boolean isButtonB2 = gamepad2.b;
        boolean isButtonA2 = gamepad2.a;
        boolean isButtonX2 = gamepad2.x;
        boolean isButtonY2 = gamepad2.y;

        boolean isButtonLB2 = gamepad2.left_bumper;
        boolean isButtonRB2 = gamepad2.right_bumper;

        boolean isButtonDU2 = gamepad2.dpad_up;
        boolean isButtonDR2 = gamepad2.dpad_right;
        boolean isButtonDL2 = gamepad2.dpad_left;
        boolean isButtonDD2 = gamepad2.dpad_down;

        boolean isButtonDU1 = gamepad1.dpad_up;
        boolean isButtonDR1 = gamepad1.dpad_right;
        boolean isButtonDL1 = gamepad1.dpad_left;
        boolean isButtonDD1 = gamepad1.dpad_down;

        //programming buttons for gamepad 2 bumpers
        //lift right bumper and left bumper
        if (isButtonRB2) {
            robot.lift.setPower(1);
            telemetry.addData("Button","RB");
            //A is retract
        } else if (isButtonLB2) {
            robot.lift.setPower(-1);
            telemetry.addData("Button","LB");
            //A is retract
        }else {
            telemetry.addData("Button","None");
            robot.lift.setPower(0);
        }


        //programming buttons for gamepad 2
        //X, Y, A, B
        //intake and retract
        if (isButtonA2) {
            robot.leftIntake.setPower(1);
            robot.rightIntake.setPower(1);
            telemetry.addData("Button","A2");
            //A is retract
        } else if (isButtonB2) {
            robot.leftIntake.setPower(0.5);
            robot.rightIntake.setPower(0.5);
            telemetry.addData("Button","B2");
            //B is extend
        } else if (isButtonX2) {
            robot.leftIntake.setPower(-1);
            robot.rightIntake.setPower(-1);
            telemetry.addData("Button","X2");
            //X is retract, but with full power

        }  else if (isButtonY2) {
            robot.leftIntake.setPower(-0.5);
            robot.rightIntake.setPower(-0.5);
            telemetry.addData("Button","Y2");
            //X is retract, but with full power
        }else {
            telemetry.addData("Button","None");
            robot.leftIntake.setPower(0);
            robot.rightIntake.setPower(0);
        }
        //programming buttons for gamepad 2
        //duckwheel
        //gamepad
        if (isButtonDU2) {
            robot.duckMotor.setPower(speedDuck);
            robot.greenLED.setState(true);
            robot.redLED.setState(false);
            telemetry.addData("Button","DU");
            //A is retract
        } else if (isButtonDD2) {
            robot.duckMotor.setPower(-speedDuck);
            robot.greenLED.setState(true);
            robot.redLED.setState(false);
            telemetry.addData("Button","DD");
            //B is extend
        } else if (isButtonDR2) {
            robot.duckMotor.setPower(1);
            robot.greenLED.setState(true);
            robot.redLED.setState(false);
            telemetry.addData("Button","DR");
            //X is retract, but with full power

        }  else if (isButtonDL2) {
            robot.duckMotor.setPower(-1);
            robot.greenLED.setState(true);
            robot.redLED.setState(false);
            telemetry.addData("Button","DL");
            //X is retract, but with full power
        }else {
            robot.greenLED.setState(false);
            robot.redLED.setState(true);
            telemetry.addData("Button","None");
            robot.duckMotor.setPower(0);
        }
        //programming buttons for gamepad 1
        //duck wheel
        //gamepad
        if (isButtonDU1) {
            robot.duckMotor.setPower(speedDuck);
            robot.greenLED.setState(true);
            robot.redLED.setState(false);
            telemetry.addData("Button","DU");
            //A is retract
        } else if (isButtonDD1) {
            robot.duckMotor.setPower(-speedDuck);
            robot.greenLED.setState(true);
            robot.redLED.setState(false);
            telemetry.addData("Button","DD");
            //B is extend
        } else if (isButtonDR1) {
            robot.duckMotor.setPower(1);
            robot.greenLED.setState(true);
            robot.redLED.setState(false);
            telemetry.addData("Button","DR");
            //X is retract, but with full power

        }  else if (isButtonDL1) {
            robot.duckMotor.setPower(-1);
            robot.greenLED.setState(true);
            robot.redLED.setState(false);
            telemetry.addData("Button","DL");
            //X is retract, but with full power
        }else {
            robot.greenLED.setState(false);
            robot.redLED.setState(true);
            telemetry.addData("Button","None");
            robot.duckMotor.setPower(0);
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        telemetry.addData("Say", "Good Job Team! We have STOPPED!!");

    }


}