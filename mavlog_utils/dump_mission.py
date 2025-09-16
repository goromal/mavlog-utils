#!/usr/bin/env python

import sys
import numpy as np
from datetime import datetime, timedelta, timezone
from pymavlink.DFReader import DFReader_binary
from pymavlink import mavutil, mavwp
import progressbar

helpstr = """
mav-dump-mission LOGNAME.bin

Dump mission waypoints from a mavlink .bin logfile.
"""


def frame_str(f):
    if f == mavutil.mavlink.MAV_FRAME_GLOBAL:
        return "Global"
    elif f == mavutil.mavlink.MAV_FRAME_LOCAL_NED:
        return "LocalNED"
    elif f == mavutil.mavlink.MAV_FRAME_MISSION:
        return "Mission"
    elif f == mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT:
        return "RelAlt"
    else:
        return f"?({f})"


def mission_cmd_str(wp):
    if wp["command"] == mavutil.mavlink.MAV_CMD_NAV_WAYPOINT:
        return f"[{wp['seq']:3d} / {wp['tot']:3d}] WAYPOINT:\t\tLat={wp['x']:.4f}\t\tLong={wp['y']:.4f}\t\tAlt={wp['z']:.4f}"
    elif wp["command"] == mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM:
        return f"[{wp['seq']:3d} / {wp['tot']:3d}] LOITER UNLIM:\tLat={wp['x']:.4f}\t\tLong={wp['y']:.4f}\t\tAlt={wp['z']:.4f}"
    elif wp["command"] == mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS:
        return f"[{wp['seq']:3d} / {wp['tot']:3d}] LOITER TURNS:\tLat={wp['x']:.4f}\t\tLong={wp['y']:.4f}\\ttAlt={wp['z']:.4f}"
    elif wp["command"] == mavutil.mavlink.MAV_CMD_DO_JUMP:
        return f"[{wp['seq']:3d} / {wp['tot']:3d}] DO JUMP:\tSeqNum={wp['param1']:.4f}\tRepeat={wp['param2']}"
    elif wp["command"] == mavutil.mavlink.MAV_CMD_NAV_LAND:
        return f"[{wp['seq']:3d} / {wp['tot']:3d}] LAND:\t\t\tLat={wp['x']:.4f}\t\tLong={wp['y']:.4f}\t\tAlt={wp['z']:.4f}"
    elif wp["command"] == mavutil.mavlink.MAV_CMD_DO_LAND_START:
        return f"[{wp['seq']:3d} / {wp['tot']:3d}] DO LAND START"
    else:
        return f"Unsupported command ID: {wp['command']}"


def gps_time_to_datetime(week, ms):
    gps_epoch = datetime(1980, 1, 6, tzinfo=timezone.utc)
    return gps_epoch + timedelta(weeks=week, milliseconds=ms)

def make_timeus_converter(anchors):
    """
    Given a list of (timeus, datetime) anchors, return a fast function
    that maps an array/list of timeus values to a list of datetimes (UTC).
    
    Uses numpy for vectorized interpolation and extrapolation.
    """
    anchors = sorted(anchors, key=lambda x: x[0])
    timeus = np.array([a[0] for a in anchors], dtype=np.int64)
    # Convert datetimes → microseconds since epoch (UTC)
    epoch = datetime(1970, 1, 1, tzinfo=timezone.utc)
    datetimes_us = np.array(
        [(a[1] - epoch).total_seconds() * 1e6 for a in anchors],
        dtype=np.float64
    )

    def convert(timeus_list):
        arr = np.asarray(timeus_list, dtype=np.int64)

        # Use numpy.interp for values inside [timeus[0], timeus[-1]]
        interp_us = np.interp(arr, timeus, datetimes_us)

        # Extrapolate below first anchor
        mask_low = arr < timeus[0]
        if mask_low.any():
            slope = (datetimes_us[1] - datetimes_us[0]) / (timeus[1] - timeus[0])
            interp_us[mask_low] = datetimes_us[0] + (arr[mask_low] - timeus[0]) * slope

        # Extrapolate above last anchor
        mask_high = arr > timeus[-1]
        if mask_high.any():
            slope = (datetimes_us[-1] - datetimes_us[-2]) / (timeus[-1] - timeus[-2])
            interp_us[mask_high] = datetimes_us[-1] + (arr[mask_high] - timeus[-1]) * slope

        # Convert back to datetime objects
        return [epoch + timedelta(microseconds=us) for us in interp_us]

    return convert


def main():
    if len(sys.argv) < 2:
        print("ERROR: No mavlink logfile provided.")
        print(helpstr)
        exit()
    elif sys.argv[1] == "-h" or sys.argv[1] == "--help":
        print(helpstr)
        exit()

    logfile = sys.argv[1]
    reader_pbar = progressbar.ProgressBar(max_value=100, redirect_stdout=True)
    reader = DFReader_binary(
        logfile, zero_time_base=True, progress_callback=reader_pbar.update
    )
    reader_pbar.finish()
    reader.rewind()

    types = set(["GPS", "CMD", "MISSION_ITEM_INT", "MISSION_ITEM"])
    timeus = []
    waypoints = []
    time_anchors = []
    while True:
        m = reader.recv_match(type=types)
        if m is None:
            break
        t = m._timestamp
        
        if m.get_type() == "GPS":
            try:
                if m.GWk <= 0 or m.GMS <= 0:
                    continue
            except:
                continue
            dt = gps_time_to_datetime(m.GWk, m.GMS)
            time_anchors.append((t, dt))
            continue
        
        elif m.get_type() == "CMD":
            try:
                frame = m.Frame
            except AttributeError:
                print("Warning: assuming frame is GLOBAL_RELATIVE_ALT")
                frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
            w = {
                "command": m.CId,
                "frame": frame,
                "seq": m.CNum,
                "tot": m.CTot,
                "x": m.Lat,
                "y": m.Lng,
                "z": m.Alt,
                "param1": m.Prm1,
                "param2": m.Prm2,
                "param3": m.Prm3,
                "param4": m.Prm4,
            }

        elif m.get_type() == "MISSION_ITEM_INT":
            w = {
                "command": m.command,
                "frame": m.frame,
                "seq": m.seq,
                "tot": 0,
                "x": m.x * 1.0e-7,
                "y": m.y * 1.0e-7,
                "z": m.z,
                "param1": m.param1,
                "param2": m.param2,
                "param3": m.param3,
                "param4": m.param4,
            }

        elif m.get_type() == "MISSION_ITEM":
            w = {
                "command": m.command,
                "frame": m.frame,
                "seq": m.seq,
                "tot": 0,
                "x": m.x,
                "y": m.y,
                "z": m.z,
                "param1": m.param1,
                "param2": m.param2,
                "param3": m.param3,
                "param4": m.param4,
            }

        timeus.append(t)
        waypoints.append(w)

    timeus_to_dt = make_timeus_converter(time_anchors)
    datetimes = timeus_to_dt(timeus)

    print()
    for t, w in zip(datetimes, waypoints):
        print(f"({t}) [Frame={frame_str(w['frame'])}] {mission_cmd_str(w)}")


if __name__ == "__main__":
    main()
