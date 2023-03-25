// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.ColorConstants.*;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AddressableLEDs;

public class LEDs extends SubsystemBase {
    private AddressableLEDs leds = new AddressableLEDs(0, 53);
    private int count = 30;
    private final SendableChooser<Color8Bit> color_selector = new SendableChooser<>();

    /** Creates a new AdressableLEDs. */
    public LEDs() {
        leds.start();
        leds.setColor(RED);
        leds.commitColor();

        color_selector.addOption("BLACK", BLACK);
        color_selector.addOption("PURPLE", PURPLE);
        color_selector.addOption("ORANGE", ORANGE);
        color_selector.addOption("YELLOW", YELLOW);
        color_selector.addOption("GREEN", GREEN);
        color_selector.addOption("BLUE", BLUE);
        color_selector.setDefaultOption("RED", RED);
        color_selector.addOption("RAINBOW", BLACK);
        SmartDashboard.putData("LEDS", color_selector);
    }

    public void setGamePieceColor(boolean isYellow) {
        if (isYellow) {
            leds.setColor(YELLOW);
        } else {
            leds.setColor(PURPLE);
        }
        leds.commitColor();
    }

    public void setColor(Color8Bit color, int index) {
        leds.setColor(color, index);
    }

    public void commitColor() {
        leds.commitColor();
    }

    public void fillColor(Color8Bit color) {
        leds.setColor(color);
    }

    @Override
    public void periodic() {
        if (color_selector.getSelected() != null) {
            leds.setColor(color_selector.getSelected());
            leds.commitColor();
        }
    }
}