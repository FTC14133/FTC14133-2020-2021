import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

public class FTC_14133_2021 extends OpMode {
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


    public void init_loop() {
    }


    public void start() {
    }


    public void loop() {
        double leftbackPower;
        double rightbackPower;
        double leftfrontPower;
        double rightfrontPower;

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        // I DONT KNOW WHAT THIS DOES
        double drive = -gamepad1.left_stick_y;
        double turn  =  gamepad1.right_stick_x;
        leftbackPower = Range.clip(drive + turn, -1.0, 1.0) ;
        rightbackPower = Range.clip(drive - turn, -1.0, 1.0) ;
        leftfrontPower = Range.clip(drive + turn, -1.0, 1.0) ;
        rightbackPower = Range.clip(drive - turn, -1.0, 1.0) ;

        Servo Claw = null;

        if(gamepad1.y) {
            // move to 0 degrees.
            Claw.setPosition(0);
        } else if (gamepad1.x) {
            // move to 90 degrees.
            Claw.setPosition(0.5);
        } else if (gamepad1.a) {
            // move to 180 degrees.
            Claw.setPosition(1);
        }
    }
}
