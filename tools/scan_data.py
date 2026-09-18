"""Loading and structuring scanner captures.

The firmware emits one CSV line per reading:

    pan, tilt, distance_cm, temperature_c, valid

Pan is a TRUE BEARING (0-180, 90 straight ahead), not a servo command --
servo_write applies the +15 offset internally. Captures taken before
2026-09-14 are in command units and are 15 degrees out; don't mix them.
"""

from dataclasses import dataclass
import math



# A dropped byte can splice two numbers into one plausible-looking integer
# (a real capture produced pan=1742). Range-checking every field is the
# only thing separating that from a legitimate reading.
PAN_RANGE = (0, 180)
TILT_RANGE = (80, 130)
DIST_RANGE = (10, 800)   # TF-Luna's own limits, per the datasheet


@dataclass
class Reading:
    pan: int
    tilt: int
    dist: int
    temp: int
    valid: bool


@dataclass
class Sweep:
    """One pass of the head in a single direction."""
    readings: list
    rising: bool          # True if pan increased through the sweep

    def __len__(self):
        return len(self.readings)

    @property
    def tilt(self):
        return self.readings[0].tilt



def to_xy(reading):
    angle=math.radians(reading.pan)
    x=reading.dist*math.cos(angle)
    y=reading.dist*math.sin(angle)

    return x,y

def load(path):
    """Parse a capture. Returns (readings, rejected_count).

    Rejects anything that fails a range check rather than trusting the
    line to be well-formed -- see PAN_RANGE above for why.
    """
    readings, rejected = [], 0

    for line in open(path):
        fields = line.strip().split(',')
        if len(fields) != 5:
            rejected += 1
            continue
        try:
            pan, tilt, dist, temp, valid = (int(f) for f in fields)
        except ValueError:
            rejected += 1
            continue

        if not (PAN_RANGE[0] <= pan <= PAN_RANGE[1]
                and TILT_RANGE[0] <= tilt <= TILT_RANGE[1]
                and DIST_RANGE[0] <= dist <= DIST_RANGE[1]):
            rejected += 1
            continue

        readings.append(Reading(pan, tilt, dist, temp, bool(valid)))

    return readings, rejected


def split_sweeps(readings, min_length=30):
    """Group readings into single-direction passes.

    A sweep boundary is where the pan step reverses sign. Direction
    matters downstream: the 50 ms dwell leaves about 2 degrees of
    direction-dependent lag, so a rising sweep and a falling sweep
    disagree slightly about where the same edge sits.

    Short fragments (the partial sweep at the start of a capture, or
    whatever was mid-pass when Ctrl-C landed) are dropped.
    """
    if not readings:
        return []

    sweeps, current = [], [readings[0]]

    for r in readings[1:]:
        if len(current) >= 2:
            prev_step = current[-1].pan - current[-2].pan
            next_step = r.pan - current[-1].pan
            reversed_ = (prev_step != 0 and next_step != 0
                         and (prev_step > 0) != (next_step > 0))
            if reversed_:
                sweeps.append(current)
                current = []
        current.append(r)
    sweeps.append(current)

    out = []
    for s in sweeps:
        if len(s) < min_length:
            continue
        out.append(Sweep(readings=s, rising=s[-1].pan > s[0].pan))
    return out


def summary(path):
    """One-line health check on a capture."""
    readings, rejected = load(path)
    sweeps = split_sweeps(readings)
    pans = [r.pan for r in readings]
    tilts = sorted({r.tilt for r in readings})
    invalid = sum(1 for r in readings if not r.valid)
    jumps = sum(1 for i in range(1, len(readings))
                if abs(readings[i].pan - readings[i - 1].pan) > 4)

    return (f"{path}\n"
            f"  {len(readings)} readings, {rejected} rejected, "
            f"{invalid} flagged invalid\n"
            f"  pan {min(pans)}-{max(pans)}, tilt {tilts}\n"
            f"  {len(sweeps)} sweeps, {jumps} pan discontinuities")


if __name__ == '__main__':
    import sys
    for path in sys.argv[1:]:
        print(summary(path))
