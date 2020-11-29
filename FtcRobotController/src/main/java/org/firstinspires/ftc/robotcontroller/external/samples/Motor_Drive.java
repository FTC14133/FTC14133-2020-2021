
package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Basic: Iterative OpMode", group="Iterative Opmode")
@Disabled
public class Motor_Drive extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftback = null;
    private DcMotor rightback = null;
    private DcMotor leftfront = null;
    private DcMotor rightfront = null;
    HardwarePushbot robot = new HardwarePushbot();

    public void init() {
        leftfront = hardwareMap.get(DcMotor.class, "left_drive");
        rightfront = hardwareMap.get(DcMotor.class, "right_drive");
        leftback  = hardwareMap.get(DcMotor.class, "left_drive");
        rightback = hardwareMap.get(DcMotor.class, "right_drive");

        leftfront.setDirection(DcMotor.Direction.FORWARD);
        rightfront.setDirection(DcMotor.Direction.REVERSE);
        leftback.setDirection(DcMotor.Direction.FORWARD);
        rightback.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double leftPowerY;
        double leftPowerX;
        double rightPowerX;
        double NormScaling;
        double leftfront;
        double rightback;
        double leftback;
        double rightfront;

        leftPowerY  = -gamepad1.left_stick_y ;
        leftPowerX  = gamepad1.left_stick_x ;
        rightPowerX = gamepad1.right_stick_x ;

        leftfront = leftPowerY + leftPowerX + rightPowerX;
        rightfront = leftPowerY - leftPowerX - rightPowerX;
        leftback = leftPowerY + leftPowerX - rightPowerX;
        rightback = leftPowerY - leftPowerX + rightPowerX;

        NormScaling = Math.max(Math.max(leftfront, rightfront), Math.max(leftback, rightback));

        leftfront = leftfront/=NormScaling;
        rightfront = rightfront/=NormScaling;
        leftback = leftback/=NormScaling;
        rightback = rightback/=NormScaling;
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
