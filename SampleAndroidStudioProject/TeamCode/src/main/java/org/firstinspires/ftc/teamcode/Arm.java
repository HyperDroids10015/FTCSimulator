package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Arm {
    public enum ClawPosition {
        OPEN,
        CLOSED,
    }
    public enum WristPosition {
        UP,
        MIDDLE,
        DOWN,
    }
    public WristPosition wrist = WristPosition.MIDDLE;
    public SimpleServo wrist_servo = null;

    public ClawPosition claw = ClawPosition.CLOSED;
    public SimpleServo claw_servo = null;


    public Arm(HardwareMap hw) {
        claw_servo = new SimpleServo(hw, "claw", 0, 360, AngleUnit.DEGREES);
        wrist_servo = new SimpleServo(hw, "wrist", 0, 360, AngleUnit.DEGREES);
    }

    public void humanInputs(GamepadEx operator, double time) {
        if (operator.wasJustPressed(GamepadKeys.Button.X)) {
            toggleClaw();
        }
        if (operator.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            if (wrist == WristPosition.DOWN){
                wrist = WristPosition.MIDDLE;
            } else {
                wrist = WristPosition.UP;
            }
        }
        if (operator.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            if (wrist == WristPosition.UP){
                wrist = WristPosition.MIDDLE;
            } else {
                wrist = WristPosition.DOWN;
            }
        }
    }

    private void toggleClaw() {
        if (claw == ClawPosition.OPEN){
            claw = ClawPosition.CLOSED;
        } else {
            claw = ClawPosition.OPEN;
        }
    }


    public void loop() {}
    public void read_sensors() {}

}
