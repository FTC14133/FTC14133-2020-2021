// simple autonomous program that drives bot forward 2 seconds then ends.

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.hardware.DcMotor;

// below is the Annotation that registers this OpMode with the FtcRobotController app.
// @Autonomous classifies the OpMode as autonomous, name is the OpMode title and the
// optional group places the OpMode into the Exercises group.
// uncomment the @Disable annotation to remove the OpMode from the OpMode list.

@Autonomous(name="Test Move", group="Exercises")
//@Disabled
public class TestMove extends LinearOpMode
{
    DcMotor leftback;
    DcMotor rightback;
    DcMotor leftfront;
    DcMotor rightfront;

    // called when init button is  pressed.

    @Override
    public void runOpMode() throws InterruptedException
    {
        leftback = hardwareMap.dcMotor.get("left_back");
        rightback = hardwareMap.dcMotor.get("right_back");
        leftfront = hardwareMap.dcMotor.get("left_front");
        rightfront = hardwareMap.dcMotor.get("right_front");

        leftback.setDirection(DcMotor.Direction.REVERSE);
        leftfront.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Mode", "waiting");
        telemetry.update();

        // wait for start button.

        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        // set both motors to 25% power.
        // forward

        leftback.setPower(0.25);
        rightback.setPower(0.25);
        leftfront.setPower(0.25);
        rightfront.setPower(0.25);

        sleep(2000);        // wait for 2 seconds.

        // set motor power to zero to stop motors.

        leftback.setPower(0.0);
        rightback.setPower(0.0);
        leftfront.setPower(0.0);
        rightfront.setPower(0.0);

        sleep(1000);

        // reverse

        leftback.setPower(-0.25);
        rightback.setPower(-0.25);
        leftfront.setPower(-0.25);
        rightfront.setPower(-0.25);

        sleep(2000);

        leftback.setPower(0.0);
        rightback.setPower(0.0);
        leftfront.setPower(0.0);
        rightfront.setPower(0.0);

        sleep(1000);

        // strafe left

        leftback.setPower(0.25);
        rightback.setPower(-0.25);
        leftfront.setPower(-0.25);
        rightfront.setPower(0.25);

        sleep(1000);

        leftback.setPower(0.0);
        rightback.setPower(0.0);
        leftfront.setPower(0.0);
        rightfront.setPower(0.0);

        sleep(1000);

        // strafe right

        leftback.setPower(-0.25);
        rightback.setPower(0.25);
        leftfront.setPower(0.25);
        rightfront.setPower(-0.25);

        sleep(2000);

        leftback.setPower(0.0);
        rightback.setPower(0.0);
        leftfront.setPower(0.0);
        rightfront.setPower(0.0);

        sleep(1000);

        // rotate left

        leftback.setPower(-0.25);
        rightback.setPower(0.25);
        leftfront.setPower(-0.25);
        rightfront.setPower(0.25);

        sleep(2000);

        leftback.setPower(0.0);
        rightback.setPower(0.0);
        leftfront.setPower(0.0);
        rightfront.setPower(0.0);

        sleep(1000);

        // rotate right

        leftback.setPower(0.25);
        rightback.setPower(-0.25);
        leftfront.setPower(0.25);
        rightfront.setPower(-0.25);

        sleep(2000);

        leftback.setPower(0.0);
        rightback.setPower(0.0);
        leftfront.setPower(0.0);
        rightfront.setPower(0.0);

    }
}