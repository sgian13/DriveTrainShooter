package org.firstinspires.ftc.teamcode;

public class Edge {
    private boolean last = false;
    public boolean pressed(boolean now) {
        boolean result = now && !last;
        last = now;
        return result;
    }
}