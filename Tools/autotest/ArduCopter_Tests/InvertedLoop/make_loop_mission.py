#!/usr/bin/env python3
"""Emit a QGC WPL 110 mission for the inverted vertical loop.

    ./make_loop_mission.py <home_lat> <home_lon> [home_alt_amsl] [bearing_deg]

bearing rotates the whole layout about home, so point it at whichever way you
have 150 m of clear space.  0 = due north.
"""
import math
import sys

R_EARTH = 6378137.0
TAKEOFF_ALT = 30.0     # m, relative
TRANSIT = 100.0        # m from home to the loop entry
CHORD = 40.0           # m, loop chord -> radius 20
H = 85.0               # m, loop centre altitude (relative); loop spans H +/- 20


def offset(lat, lon, north, east):
    dlat = north / R_EARTH
    dlon = east / (R_EARTH * math.cos(math.radians(lat)))
    return lat + math.degrees(dlat), lon + math.degrees(dlon)


def main():
    if len(sys.argv) < 3:
        sys.exit(__doc__)
    lat, lon = float(sys.argv[1]), float(sys.argv[2])
    amsl = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
    brg = math.radians(float(sys.argv[4]) if len(sys.argv) > 4 else 0.0)

    def at(along, alt):
        n, e = along * math.cos(brg), along * math.sin(brg)
        la, lo = offset(lat, lon, n, e)
        return la, lo, alt

    # seq, frame, cmd, p1, p2, along-track distance, alt
    items = [
        (1, 3, 22, 0, 0, None, TAKEOFF_ALT),          # NAV_TAKEOFF
        (2, 3, 16, 0, 0, TRANSIT, TAKEOFF_ALT),       # transit out
        (3, 3, 16, 0, 0, TRANSIT, H),                 # vertical climb -> tangential entry
        (4, 3, 36, 180, 90, TRANSIT + CHORD, H),      # ARC over the top
        (5, 3, 36, -180, 90, TRANSIT, H),             # ARC under the bottom
        (6, 3, 20, 0, 0, None, 0.0),                  # RTL
    ]
    out = ["QGC WPL 110"]
    out.append("0\t1\t0\t16\t0\t0\t0\t0\t%.8f\t%.8f\t%.6f\t1" % (lat, lon, amsl))
    for seq, frame, cmd, p1, p2, along, alt in items:
        if along is None:
            la, lo = 0.0, 0.0
        else:
            la, lo, _ = at(along, alt)
        out.append("%d\t0\t%d\t%d\t%.6f\t%.6f\t0\t0\t%.8f\t%.8f\t%.6f\t1"
                   % (seq, frame, cmd, p1, p2, la, lo, alt))
    print("\n".join(out))


if __name__ == '__main__':
    main()
