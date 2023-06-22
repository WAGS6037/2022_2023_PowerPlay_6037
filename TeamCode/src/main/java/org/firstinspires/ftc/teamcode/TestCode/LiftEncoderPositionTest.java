package org.firstinspires.ftc.teamcode.TestCode;

        import com.qualcomm.robotcore.eventloop.opmode.OpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.util.ElapsedTime;

        import org.firstinspires.ftc.teamcode.HardwareMap.HardwareMap_CompetitionBot;

// name this opMode and determine a group
@TeleOp(name="LiftEncoder POSITION Test", group="Test?")
public class LiftEncoderPositionTest extends OpMode{

    /* Declare OpMode members. */
    HardwareMap_CompetitionBot robot       = new  HardwareMap_CompetitionBot();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {

        /* Initialize the har5dware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

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

        boolean isButtonB = gamepad2.b;
        boolean isButtonA = gamepad2.a;
        boolean isButtonX = gamepad2.x;
        boolean isButtonY = gamepad2.y;

        final int down = 0;
        final int level1 = -4530;
        //final int level2 = 4752;

        if (isButtonA) {
            liftUpPosition(level1, 1.0);
            telemetry.addData("Button","A");
            //A is retract
        } else if (isButtonB) {
            //liftUpPosition(level2, 0.3);
            telemetry.addData("Button","B");
            //B is extend
        } else if (isButtonX) {
            liftUpPosition(down, 1.0);
            telemetry.addData("Button", "B");
            //B is extend
        }else {
            telemetry.addData("Button","None");
            robot.slideSystem.setPower(0);
        }
        telemetry.addData("Lift Position",String.format("%7d", robot.slideSystem.getCurrentPosition()));
        telemetry.update();

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        telemetry.addData("Say", "Good Job Team! We have STOPPED!!");

    }

    public void liftUpPosition(int position, double liftSpeed) {

        robot.slideSystem.setTargetPosition(position);
        robot.slideSystem.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.slideSystem.setPower(Math.abs(liftSpeed));

    }

}
