package org.firstinspires.ftc.teamcode;
// https://first-tech-challenge.github.io/SkyStone/  This is the link to ALL metered of FTC
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="FTC 14133 2021", group="Iterative Opmode")
public class Claw extends OpMode {

    Servo Claw = null;          // Sets the variable of the Claw
    boolean clawstate = false;          // Sets the variable of the clawstate
    boolean toggle = true;          // Sets the variable of the toggle



    public void init() {
        Claw = hardwareMap.get(Servo.class, "Claw");

        Claw.setPosition(0);

    }



    public void init_loop() {
    }


    public void start() {
    }


    public void loop() {


        //Claw

        if (toggle && gamepad2.y) {  // Only execute once per Button push
            toggle = false;  // Prevents this section of code from being called again until the Button is released and re-pressed
            if (clawstate) {  // Decide which way to set the motor this time through (or use this as a motor value instead)
                clawstate = false;
                Claw.setPosition(1);
            } else {
                clawstate = true;
                Claw.setPosition(0);
            }
        } else if (gamepad2.y == false) {
            toggle = true; // Button has been released, so this allows a re-press to activate the code above.
        }


        }
    }