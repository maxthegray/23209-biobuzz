package org.firstinspires.ftc.teamcode.tests.flowerdash;

import org.json.JSONObject;

// something the website (or the gamepad) wants the opmode to do. the opmode loop is the only thing that acts on these
public final class Command {
    public final String name;
    public final JSONObject args;

    public Command(String name, JSONObject args) {
        this.name = name;
        this.args = args == null ? new JSONObject() : args;
    }

    public Command(String name) {
        this(name, null);
    }
}
