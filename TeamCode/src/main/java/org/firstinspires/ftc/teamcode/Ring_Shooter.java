package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp(name="Basic: Iterative OpMode", group="Iterative Opmode")
public class Ring_Shooter extends OpMode{
    private DcMotor Shooter = null;
    double ShooterPower = 1;             // mayhaps
    private DcMotor intake = null;          // Sets the variable of the intake
    private DcMotor conveyor = null;          // Sets the variable of the conveyor

    @Override
    public void init() {
        Shooter = hardwareMap.get(DcMotor.class, "Shooter");
        Shooter.setDirection(DcMotor.Direction.FORWARD);
        intake = hardwareMap.get(DcMotor.class, "intake");
        conveyor = hardwareMap.get(DcMotor.class, "conveyor");
    }

    public void init_loop() {
    }


    public void start(){
    }

    @Override
    public void loop() {

        if (gamepad2.left_stick_y < 0) {
            ShooterPower += 0.01;
        }

        if (gamepad2.left_stick_y > 0) {
            ShooterPower -= 0.01;
        }

        if (ShooterPower > 1) {
            ShooterPower = 1;
        }

        if (ShooterPower < 0) {
            ShooterPower = 0;
        }

        if (gamepad2.b) {
            Shooter.setPower(ShooterPower);            // This Controls the shooter
            intake.setPower(1);
            conveyor.setPower(1);
        }
        else {
            Shooter.setPower(0);    // This tells the program to set the Intake, Long Arm, and Shooter
            intake.setPower(0);
            conveyor.setPower(0);                        //to turn them off when not being used
        }
    }
}

