#!/usr/bin/env python

import sys
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
    if wp.command == mavutil.mavlink.MAV_CMD_NAV_WAYPOINT:
        return f"WAYPOINT:\tLat={wp.x:.4f}\tLong={wp.y:.4f}\tAlt={wp.z:.4f}"
    elif wp.command == mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM:
        return f"LOITER UNLIM:\tLat={wp.x:.4f}\tLong={wp.y:.4f}\tAlt={wp.z:.4f}"
    elif wp.command == mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS:
        return f"LOITER TURNS:\tLat={wp.x:.4f}\tLong={wp.y:.4f}\tAlt={wp.z:.4f}"
    elif wp.command == mavutil.mavlink.MAV_CMD_DO_JUMP:
        return f"DO JUMP:\tSeqNum={wp.param1:.4f}\tRepeat={wp.param2}"
    else:
        return f"Unsupported command ID: {wp.command}"

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
    reader = DFReader_binary(logfile, zero_time_base=True, progress_callback=reader_pbar.update)
    reader_pbar.finish()
    reader.rewind()

    types = set(['CMD','MISSION_ITEM_INT'])
    wp = mavwp.MAVWPLoader()
    times = []
    while True:
        m = reader.recv_match(type=types)
        if m is None:
            break
        t = m._timestamp
        if m.get_type() == 'CMD':
            try:
                frame = m.Frame
            except AttributeError:
                print("Warning: assuming frame is GLOBAL_RELATIVE_ALT")
                frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
            m = mavutil.mavlink.MAVLink_mission_item_message(0,
                                                             0,
                                                             m.CNum,
                                                             frame,
                                                             m.CId,
                                                             0, 1,
                                                             m.Prm1, m.Prm2, m.Prm3, m.Prm4,
                                                             m.Lat, m.Lng, m.Alt)

        if m.get_type() == 'MISSION_ITEM_INT':
            m = mavutil.mavlink.MAVLink_mission_item_message(m.target_system,
                                                             m.target_component,
                                                             m.seq,
                                                             m.frame,
                                                             m.command,
                                                             m.current,
                                                             m.autocontinue,
                                                             m.param1,
                                                             m.param2,
                                                             m.param3,
                                                             m.param4,
                                                             m.x*1.0e-7,
                                                             m.y*1.0e-7,
                                                             m.z)
        if m.current >= 2:
            continue

        while m.seq > wp.count():
            print("Adding dummy WP %u" % wp.count())
            wp.set(m, wp.count())
        wp.set(m, m.seq)
        times.append(t)

    print()
    for t, i in zip(times, range(wp.count())):
        w = wp.wp(i)
        print(f"({t:.2f}) [Seq={w.seq}, Curr={w.current}, Frame={frame_str(w.frame)}] {mission_cmd_str(w)}")
        # print("%f\t%u\t%u\t%u\t%u\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%u" % (
        #     t, w.seq, w.current, w.frame, w.command,
        #     w.param1, w.param2, w.param3, w.param4,
        #     w.x, w.y, w.z, w.autocontinue))

if __name__ == "__main__":
    main()
