package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ArmMove;
import frc.robot.commands.ArmMoveDown;
import frc.robot.commands.ArmStop;
/*import frc.robot.commands.IntakeInLeft;
import frc.robot.commands.IntakeInRight;
import frc.robot.commands.ShooterLeft;
import frc.robot.commands.ShooterRight;*/
import frc.robot.commands.ShintakeOn;

public class OI {
    public Joystick j;
    public Joystick joy;
    /*private JoystickButton leftShintakeIn;
    private JoystickButton rightShintakeIn;
    private JoystickButton leftShintakeOut;
    private JoystickButton rightShintakeOut;*/
    private JoystickButton bothShintakeIn;
    private JoystickButton bothShintakeOut;
    private JoystickButton Defense;
    /*private JoystickButton ArmUp;
    private JoystickButton ArmDown;
    private JoystickButton ArmMid;
    private JoystickButton ArmStop;

    private JoystickButton bothShintakeIn2;
    private JoystickButton bothShintakeOut2;
    private JoystickButton Defense2;*/
    private JoystickButton ArmUp2;
    private JoystickButton ArmDown2;
    //private JoystickButton ArmMid2;
    private JoystickButton ArmStop2;

    

    public OI() {
        j = new Joystick(0);
        joy = new Joystick(1);
        /*leftShintakeIn = new JoystickButton(j, 5);
        leftShintakeIn.whileHeld(new IntakeInLeft());

        rightShintakeIn = new JoystickButton(j, 6);
        rightShintakeIn.whileHeld(new IntakeInRight());

        leftShintakeOut = new JoystickButton(j, 3);
        leftShintakeOut.whileHeld(new ShooterLeft());

        rightShintakeOut = new JoystickButton(j, 4);
        rightShintakeOut.whileHeld(new ShooterRight());*/

        bothShintakeIn = new JoystickButton(j, 2);
        bothShintakeIn.whileHeld(new ShintakeOn(-0.5));

        bothShintakeOut = new JoystickButton(j, 1);
        bothShintakeOut.whileHeld(new ShintakeOn(0.9));

        /*ArmUp = new JoystickButton(j, 8);
        ArmUp.whenPressed(new ArmMove(0.4));

        ArmDown = new JoystickButton(j, 7);
        ArmDown.whenPressed(new ArmMoveDown(-0.5));

        ArmMid = new JoystickButton(j, 9);
        ArmMid.whileHeld(new ArmMove(0.2));*/

        Defense = new JoystickButton(j, 3);
        Defense.whileHeld(new ShintakeOn(0.3));

        // ArmStop = new JoystickButton(j, 11);
        // ArmStop.whenPressed(new ArmStop(0));

        // Split Drive:

        /*bothShintakeIn2 = new JoystickButton(joy, 6);
        bothShintakeIn2.whileHeld(new ShintakeOn(-0.5));

        bothShintakeOut2 = new JoystickButton(joy, 8);
        bothShintakeOut2.whileHeld(new ShintakeOn(0.9));*/

        ArmUp2 = new JoystickButton(joy, 6);
        ArmUp2.whenPressed(new ArmMove(0.4));

        ArmDown2 = new JoystickButton(joy, 5);
        ArmDown2.whenPressed(new ArmMoveDown(-0.4));

        // ArmMid2 = new JoystickButton(joy, 1);
        // ArmMid2.whileHeld(new ArmMove(0.2));

        // Defense2 = new JoystickButton(joy, 2);
        // Defense2.whileHeld(new ShintakeOn(0.5));

        ArmStop2 = new JoystickButton(joy, 3);
        ArmStop2.whenPressed(new ArmStop(0));

        

        

    }
}
