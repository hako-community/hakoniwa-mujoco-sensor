#!/usr/bin/env python3
"""Declare a sensor's SHM channel in a hakoniwa pdudef (issue #5).

Adding a third radar used to be a manual edit in several places: a reader block
and a writer block for every robot in the pdudef -- four near-identical JSON
objects for one sensor on a two-aircraft config -- plus the channel number
repeated on the launcher command line as A2_PDU_MAP="name=ch".

The bridge now derives its mapping from the pdudef (runtime/pdudef_channels.hpp),
so the command line copy is gone. This script removes the other half: it writes
the blocks, picks a free channel id, and keeps every robot consistent.

    # add a rear radar to both aircraft, auto-assigning the channel
    ./add_sensor_channel.py config2/webavatar-2-radar2.json radar_points_rear

    # a specific channel, one robot only
    ./add_sensor_channel.py pdudef.json radar_points_left --channel 22 --robot Drone

Idempotent: re-running reports "already declared" and changes nothing, unless
--channel asks for a different id, which is an explicit retarget.

Channel ids must agree across robots. The master lays out SHM per robot, but the
bridge is told one channel per pdu_name and publishes to whichever robot it runs
for, so a name that means ch21 on Drone and ch22 on Drone1 sends one aircraft's
radar into the wrong slot. Auto-assignment therefore picks an id free on ALL
selected robots rather than per robot.
"""

import argparse
import json
import sys

# PointCloud2 at the size the existing sensor channels use. Both radar and 3D
# lidar publish PointCloud2, so this default covers the case the issue is about.
DEFAULT_TYPE = "sensor_msgs/PointCloud2"
DEFAULT_SIZE = 177424

PDU_LISTS = ("shm_pdu_readers", "shm_pdu_writers")


def used_channel_ids(robot):
    ids = set()
    for key in PDU_LISTS:
        for entry in robot.get(key, []):
            if "channel_id" in entry:
                ids.add(entry["channel_id"])
    return ids


def find_existing(robot, org_name):
    """Return the first entry for org_name in this robot, or None."""
    for key in PDU_LISTS:
        for entry in robot.get(key, []):
            if entry.get("org_name") == org_name:
                return entry
    return None


def make_entry(robot_name, org_name, channel_id, pdu_type, pdu_size):
    # Field order and contents mirror the entries already in these files, so a
    # diff shows one added block rather than a reformat.
    return {
        "type": pdu_type,
        "org_name": org_name,
        "name": f"{robot_name}_{org_name}",
        "channel_id": channel_id,
        "pdu_size": pdu_size,
        "write_cycle": 1,
        "method_type": "SHM",
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("pdudef", help="pdudef JSON to edit in place")
    ap.add_argument("org_name", help='channel name, e.g. "radar_points_rear"')
    ap.add_argument("--channel", type=int, default=None,
                    help="channel id (default: lowest free id >= 16, common to all robots)")
    ap.add_argument("--pdu-size", type=int, default=DEFAULT_SIZE,
                    help=f"declared channel width in bytes (default {DEFAULT_SIZE})")
    ap.add_argument("--type", default=DEFAULT_TYPE,
                    help=f"PDU type (default {DEFAULT_TYPE})")
    ap.add_argument("--robot", action="append", default=None,
                    help="restrict to this robot (repeatable; default: every robot)")
    ap.add_argument("--dry-run", action="store_true",
                    help="print what would change without writing")
    args = ap.parse_args()

    with open(args.pdudef) as f:
        doc = json.load(f)

    robots = doc.get("robots")
    if not isinstance(robots, list):
        sys.exit(f"{args.pdudef}: no \"robots\" array")

    targets = [r for r in robots if args.robot is None or r.get("name") in args.robot]
    if not targets:
        sys.exit(f"no matching robot (have: {[r.get('name') for r in robots]})")

    # Report an existing declaration before touching anything, so a re-run is a
    # no-op rather than a duplicate block.
    existing = {r["name"]: find_existing(r, args.org_name) for r in targets}
    already = {n: e for n, e in existing.items() if e is not None}
    if already and args.channel is None:
        for name, entry in already.items():
            print(f"{name}: '{args.org_name}' already declared on ch{entry['channel_id']}")
        if len(already) == len(targets):
            print("nothing to do")
            return 0

    if args.channel is not None:
        channel_id = args.channel
        clashes = [r["name"] for r in targets
                   if channel_id in used_channel_ids(r)
                   and (existing[r["name"]] or {}).get("channel_id") != channel_id]
        if clashes:
            sys.exit(f"ch{channel_id} is already used by: {', '.join(clashes)}")
    else:
        # One id free on every selected robot -- see the note in the docstring.
        taken = set()
        for r in targets:
            taken |= used_channel_ids(r)
        channel_id = next(c for c in range(16, 1024) if c not in taken)

    changes = []
    for robot in targets:
        rname = robot.get("name", "?")
        for key in PDU_LISTS:
            lst = robot.setdefault(key, [])
            hit = next((e for e in lst if e.get("org_name") == args.org_name), None)
            if hit is not None:
                if hit.get("channel_id") != channel_id:
                    changes.append(f"{rname}.{key}: {args.org_name} ch{hit['channel_id']}"
                                   f" -> ch{channel_id}")
                    hit["channel_id"] = channel_id
                    hit["pdu_size"] = args.pdu_size
                continue
            lst.append(make_entry(rname, args.org_name, channel_id, args.type, args.pdu_size))
            changes.append(f"{rname}.{key}: + {args.org_name} ch{channel_id} "
                           f"size={args.pdu_size}")

    if not changes:
        print("nothing to do")
        return 0

    for c in changes:
        print(c)

    if args.dry_run:
        print("(dry run -- nothing written)")
        return 0

    with open(args.pdudef, "w") as f:
        json.dump(doc, f, indent=2, ensure_ascii=False)
        f.write("\n")

    print(f"\nwrote {args.pdudef}")
    print(f"the bridge picks this up via A2_PDUDEF={args.pdudef}; no A2_PDU_MAP needed.")
    print("give the sensor pdu_name=\"%s\" in the A-2 manifest." % args.org_name)
    print("NOTE: the channel layout changed -- drop /var/lib/hakoniwa/mmap/*.bin "
          "before the next run or the master keeps the old layout.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
