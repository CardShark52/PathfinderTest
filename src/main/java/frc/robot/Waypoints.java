package frc.robot;

import jaci.pathfinder.*;

public final class Waypoints {
    public static final Waypoint[] defaultAuto = new Waypoint[] {
        new Waypoint(0, 0, 0),  // Waypoint @ x=0, y=0,   exit angle=0 radians
        new Waypoint(2, 2, 0),                        // Waypoint @ x=-2, y=-2, exit angle=0 radians
        new Waypoint(4, 1, Pathfinder.d2r(-45))     // Waypoint @ x=-4, y=-1, exit angle=-45 degrees
    };

    public static final Waypoint[] longerAuto = new Waypoint[] {
        new Waypoint(0.0, 0.0, Pathfinder.d2r(0)),
        new Waypoint(1.5, 1.0, Pathfinder.d2r(0)),
        new Waypoint(2.5, 1.0, Pathfinder.d2r(0)),
        new Waypoint(4.0, 2.0, Pathfinder.d2r(0)),
        new Waypoint(5.0, 2.0, Pathfinder.d2r(0)),
    };

    public static final Waypoint[] auto3 = new Waypoint[] {
        new Waypoint(0.0, 0.0, Pathfinder.d2r(0)),
        new Waypoint(1.5, 1.0, Pathfinder.d2r(0)),
        new Waypoint(2.5, 1.0, Pathfinder.d2r(0)),
        new Waypoint(4.0, 2.0, Pathfinder.d2r(0)),
        new Waypoint(5.0, 2.0, Pathfinder.d2r(0)),
    };

    public static final Waypoint[] newPath = new Waypoint[] {
        new Waypoint(0.0, 0.0, Pathfinder.d2r(0.0)),
        new Waypoint(5.0, 6.0, Pathfinder.d2r(90.0)),
    };

    public static final Waypoint[] circle = new Waypoint[] {
        new Waypoint(2.0, 2.0, Pathfinder.d2r(0.0)),
        new Waypoint(4.0, 4.0, Pathfinder.d2r(90.0)),
        new Waypoint(2.0, 6.0, Pathfinder.d2r(180.0)),
        new Waypoint(0.0, 4.0, Pathfinder.d2r(-90.0)),
        new Waypoint(2.0, 4.0, Pathfinder.d2r(0.0)),
    };
}