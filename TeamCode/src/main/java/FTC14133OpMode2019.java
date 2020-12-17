/**public class FTC14133OpMode2019 {
}
package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "AutoMoob2 (Blocks to Java)", group = "")
public class AutoMoob2 extends LinearOpMode {

    private DcMotor FrontLeft;
    private DcMotor BackLeft;
    private DcMotor Elevator;
    private DcMotor FrontRight;
    private DcMotor BackRight;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
/**    @Override
    public void runOpMode() {
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        Elevator = hardwareMap.get(DcMotor.class, "Elevator");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");

        // Put initialization blocks here.
        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        IntakeDown();
        sleep(1500);
        Elevator.setPower(0);
        Elevator.setPower(-0.5);
        sleep(1500);
        Elevator.setPower(0);
        Driveforward();
        sleep(1500);
        Strafeleft();
        sleep(2000);
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);
    }

    /**
     * Describe this function...
     */
/**    private void Driveforward() {
        FrontLeft.setPower(-0.5);
        FrontRight.setPower(-0.5);
        BackLeft.setPower(-0.5);
        BackRight.setPower(-0.5);
    }

    /**
     * Describe this function...
     */
/**    private void Drivebackward() {
        FrontLeft.setPower(0.5);
        FrontRight.setPower(0.5);
        BackLeft.setPower(0.5);
        BackRight.setPower(0.5);
    }

    /**
     * Describe this function...
     */
/**    private void Strafeleft() {
        FrontLeft.setPower(0.5);
        FrontRight.setPower(-0.5);
        BackLeft.setPower(-0.5);
        BackRight.setPower(0.5);
    }

    /**
     * Describe this function...
     */
/**    private void Straferight() {
        FrontLeft.setPower(-0.5);
        FrontRight.setPower(0.5);
        BackLeft.setPower(0.5);
        BackRight.setPower(-0.5);
    }

    /**
     * Describe this function...
     */
 /**   private void Turnleft() {
        FrontLeft.setPower(-0.5);
        FrontRight.setPower(0.5);
        BackLeft.setPower(-0.5);
        BackRight.setPower(0.5);
    }

    /**
     * Describe this function...
     */
/**    private void Turnright() {
        FrontLeft.setPower(0.5);
        FrontRight.setPower(-0.5);
        BackLeft.setPower(0.5);
        BackRight.setPower(-0.5);
    }

    /**
     * Describe this function...
     */
 /**   private void IntakeDown() {
        Elevator.setPower(1);
    }
}
  */