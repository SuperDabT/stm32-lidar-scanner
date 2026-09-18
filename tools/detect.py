"""Detection against a background baseline.

Fill in the two functions marked TODO. The evaluation harness below is
plumbing -- it just runs your detector against the three static captures
and prints what it found.

Run:  python3 detect.py
"""

from scan_data import load, split_sweeps, to_xy
import statistics
import math

# ---------------------------------------------------------------- step 2




def background_profile(readings):
    groups={}
    for reading in readings:
        if reading.pan not in groups:
            groups[reading.pan]=[]
        groups[reading.pan].append(reading.dist)
    fresher_readings={}
    for angle, pile in groups.items():
        fresher_readings[angle]=statistics.median(pile)
    return fresher_readings
        

    
    """One expected distance per bearing, from an empty-room capture.

    readings: the full list from load() on empty_room.csv. Each bearing
    appears ~24 times (once per sweep).

    Returns: dict mapping bearing -> expected distance in cm.

    Decide: mean or median across the repeats at each bearing? Consider
    what a bad reading looks like here -- mostly wall at ~230cm, with the
    occasional outlier from a stray reflection or a mid-move sample.
    """
    # TODO
    raise NotImplementedError


# ---------------------------------------------------------------- step 3

def detect(sweep, background, min_drop_cm, min_width):
    found=[]
    box=[]
    for reading in sweep.readings:
        if  reading.dist<background[reading.pan]-min_drop_cm:
            box.append(reading)
                
        else:
            if len(box)>=min_width:
                found.append(box)
            
            box=[]
    if len(box)>=min_width:
        found.append(box)
    return found

def keep_person_sized(clusters,min_cm,max_cm):
    person=[]
    for cluster in clusters:
        if object_width(cluster)>=min_cm and object_width(cluster)<=max_cm:
            person.append(cluster)
    return person

    """Find objects in one sweep that weren't in the background.

    sweep:       a Sweep from split_sweeps()
    background:  dict from background_profile()
    min_drop_cm: how much closer than background counts as "something"
    min_width:   how many consecutive bearings before you believe it

    Returns: list of clusters. Each cluster is a list of Readings.

    Three things this has to handle, all visible in the captures:

      1. A bearing may be missing from the background dict (the sweep
         covered a bearing the baseline didn't). Skip those.
      2. Edges are gradual. In person_15.csv the profile runs
         260, 250, 241, 170, 170, 170, 172, 174, 230 -- the readings
         either side of the core are partial hits where the beam
         straddles the target. Your threshold decides whether they're in.
      3. A run shorter than min_width is noise, not an object.

    Suggested starting values: min_drop_cm=40, min_width=3. Both are
    guesses -- the whole point of the captures is to replace them with
    measurements.
    """
    # TODO
    raise NotImplementedError


# ---------------------------------------------------------------- step 4

def cluster_bearing(cluster):
    """Where is this cluster? Centroid of the bearings it spans."""
    return sum(r.pan for r in cluster) / len(cluster)


def cluster_range(cluster):
    """How far away? Median distance across the cluster."""
    dists = sorted(r.dist for r in cluster)
    return dists[len(dists) // 2]

def object_width(cluster):
    width=math.dist((to_xy(cluster[-1])),(to_xy(cluster[0])))
    return width

def evaluate(path, background, expected_bearing=None, **kwargs):
    """Run detect() over every sweep in a capture and report."""
    readings, _ = load(path)
    sweeps = split_sweeps(readings)

    found = []
    for sweep in sweeps:
        for cluster in detect(sweep, background, **kwargs):
            found.append((cluster_bearing(cluster),
                          cluster_range(cluster),
                          len(cluster),
                          sweep.rising))

    name = path.split('/')[-1]
    print(f"\n{name}  ({len(sweeps)} sweeps)")

    if not found:
        print("  no detections")
    else:
        bearings = [f[0] for f in found]
        widths = [f[2] for f in found]
        print(f"  {len(found)} detections across {len(sweeps)} sweeps")
        print(f"  bearing: {min(bearings):.0f} to {max(bearings):.0f}, "
              f"mean {sum(bearings)/len(bearings):.1f}")
        print(f"  width:   {min(widths)} to {max(widths)} bearings")

    if expected_bearing is not None:
        if not found:
            print(f"  MISS -- expected something near {expected_bearing}")
        else:
            err = sum(b for b in bearings) / len(bearings) - expected_bearing
            print(f"  expected ~{expected_bearing}, off by {err:+.1f}")

    return found


if __name__ == '__main__':
    UP = '/mnt/user-data/uploads/'
    params = dict(min_drop_cm=40, min_width=3)

    empty, _ = load(UP + 'empty_room.csv')
    bg = background_profile(empty)

    evaluate(UP + 'person_15.csv', bg, expected_bearing=19, **params)
    evaluate(UP + 'person_95.csv', bg, expected_bearing=96, **params)
    evaluate(UP + 'empty_room.csv', bg, **params)   # false-positive check
