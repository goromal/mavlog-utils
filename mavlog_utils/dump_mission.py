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
Detects and highlights mission uploads, especially those occurring in AUTO mode.
Also shows SET_POSITION_TARGET_GLOBAL_INT commands for direct position control.
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

def get_seq_num(wp):
    return wp['seq']

def get_lat_lon_tuple(wp):
    return (wp['x'], wp['y']) if wp['command'] == mavutil.mavlink.MAV_CMD_NAV_WAYPOINT or wp['command'] == mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM else (np.inf, np.inf)

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
        return [epoch + timedelta(microseconds=us) if np.isfinite(us) else epoch for us in interp_us]
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
    
    # Add POSITION_TARGET tracking and MODE tracking
    types = set(["GPS", "CMD", "MISSION_ITEM_INT", "MISSION_ITEM", "MODE", "HEARTBEAT", 
                "MISSION_COUNT", "MISSION_ACK", "SET_POSITION_TARGET_GLOBAL_INT"])
    timeus = []
    waypoints = []
    time_anchors = []
    
    # Track vehicle mode and mission uploads
    mode_changes = []
    mission_events = []
    position_targets = []
    current_mode = "UNKNOWN"
    mission_upload_in_progress = False
    mission_items_expected = 0
    mission_items_received = 0
    
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
        
        # Track flight mode changes
        elif m.get_type() == "MODE":
            try:
                mode_name = m.Mode
                if current_mode != mode_name:
                    mode_changes.append((t, mode_name))
                    current_mode = mode_name
            except AttributeError:
                pass
        
        # Alternative mode detection using HEARTBEAT
        elif m.get_type() == "HEARTBEAT":
            try:
                mode_name = mavutil.mode_string_v10(m.base_mode, m.custom_mode)
                if current_mode != mode_name:
                    mode_changes.append((t, mode_name))
                    current_mode = mode_name
            except AttributeError:
                pass
        
        # Mission upload start
        elif m.get_type() == "MISSION_COUNT":
            mission_upload_in_progress = True
            mission_items_expected = m.count
            mission_items_received = 0
            mission_events.append((t, "UPLOAD_START", current_mode, m.count))
        
        # Mission upload end
        elif m.get_type() == "MISSION_ACK":
            if mission_upload_in_progress:
                mission_upload_in_progress = False
                status = "SUCCESS" if m.type == 0 else f"FAILED (code: {m.type})"
                mission_events.append((t, "UPLOAD_END", current_mode, status))
        
        # Mission item received during upload
        elif m.get_type() in ["MISSION_ITEM", "MISSION_ITEM_INT"] and mission_upload_in_progress:
            mission_items_received += 1
            if mission_items_received == mission_items_expected:
                mission_upload_in_progress = False
                mission_events.append((t, "UPLOAD_COMPLETE", current_mode, mission_items_expected))
        
        # Track SET_POSITION_TARGET_GLOBAL_INT commands
        elif m.get_type() == "SET_POSITION_TARGET_GLOBAL_INT":
            # Extract the relevant fields
            pos_target = {
                "time_us": t,
                "lat": m.lat_int * 1.0e-7,  # Convert from int to float degrees
                "lon": m.lon_int * 1.0e-7,
                "alt": m.alt,
                "type_mask": m.type_mask,
                "coordinate_frame": m.coordinate_frame,
                "vx": m.vx,
                "vy": m.vy, 
                "vz": m.vz,
                "afx": m.afx,
                "afy": m.afy,
                "afz": m.afz,
                "yaw": m.yaw,
                "yaw_rate": m.yaw_rate,
                "mode": current_mode
            }
            position_targets.append(pos_target)
        
        # Process waypoint data
        if m.get_type() == "CMD":
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
                "mode": current_mode,  # Save current mode with waypoint
            }
            timeus.append(t)
            waypoints.append(w)
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
                "mode": current_mode,  # Save current mode with waypoint
            }
            timeus.append(t)
            waypoints.append(w)
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
                "mode": current_mode,  # Save current mode with waypoint
            }
            timeus.append(t)
            waypoints.append(w)
    
    timeus_to_dt = make_timeus_converter(time_anchors)
    datetimes = timeus_to_dt(timeus)
    
    # Print mode changes
    print("\n=== FLIGHT MODE CHANGES ===")
    for t, mode in mode_changes:
        dt = timeus_to_dt([t])[0]
        print(f"{dt}: Mode changed to {mode}")
    
    # Print mission upload events
    print("\n=== MISSION UPLOAD EVENTS ===")
    for t, event_type, mode, details in mission_events:
        dt = timeus_to_dt([t])[0]
        if event_type == "UPLOAD_START":
            print(f"{dt}: Mission upload started with {details} items while in {mode} mode")
        elif event_type == "UPLOAD_COMPLETE":
            print(f"{dt}: Mission upload completed with {details} items while in {mode} mode")
        elif event_type == "UPLOAD_END":
            print(f"{dt}: Mission upload ended with status: {details} while in {mode} mode")
    
    # Print direct position target commands
    print("\n=== POSITION TARGET COMMANDS ===")
    if position_targets:
        for pt in position_targets:
            dt = timeus_to_dt([pt["time_us"]])[0]
            
            # Determine what type of command it is based on the bitmask
            cmd_type = []
            if not (pt["type_mask"] & 0x007):  # Position setpoint
                cmd_type.append("POS")
            if not (pt["type_mask"] & 0x038):  # Velocity setpoint
                cmd_type.append("VEL")
            if not (pt["type_mask"] & 0x1C0):  # Acceleration setpoint
                cmd_type.append("ACC")
            if not (pt["type_mask"] & 0x400):  # Yaw setpoint
                cmd_type.append("YAW")
            if not (pt["type_mask"] & 0x800):  # Yaw rate setpoint
                cmd_type.append("YAWRATE")
            
            cmd_type_str = "+".join(cmd_type) if cmd_type else "UNKNOWN"
            
            # Format the output based on what's being commanded
            output_parts = [f"({dt}) [{pt['mode']}] {cmd_type_str} TARGET:"]
            
            if not (pt["type_mask"] & 0x007):
                output_parts.append(f"Lat={pt['lat']:.6f} Lon={pt['lon']:.6f} Alt={pt['alt']:.2f}")
            
            if not (pt["type_mask"] & 0x038):
                output_parts.append(f"Vx={pt['vx']:.2f} Vy={pt['vy']:.2f} Vz={pt['vz']:.2f}")
            
            if not (pt["type_mask"] & 0x1C0):
                output_parts.append(f"Ax={pt['afx']:.2f} Ay={pt['afy']:.2f} Az={pt['afz']:.2f}")
            
            if not (pt["type_mask"] & 0x400):
                output_parts.append(f"Yaw={pt['yaw']:.2f}")
                
            if not (pt["type_mask"] & 0x800):
                output_parts.append(f"YawRate={pt['yaw_rate']:.2f}")
                
            print(" ".join(output_parts))
    else:
        print("No position target commands found in log.")
    
    # Print waypoints with enhanced upload detection
    print("\n=== MISSION WAYPOINTS ===")
    prev_seq1_lat, prev_seq1_lon = 0.0, 0.0
    for t, w in zip(datetimes, waypoints):
        probable_upload = False
        # Check for new mission uploads (when waypoint 1 changes)
        if get_seq_num(w) == 1:
            cur_seq1_lat, cur_seq1_lon = get_lat_lon_tuple(w)
            if (cur_seq1_lat != prev_seq1_lat or cur_seq1_lon != prev_seq1_lon) and not (prev_seq1_lat == 0.0 and prev_seq1_lon == 0.0):
                probable_upload = True
            prev_seq1_lat = cur_seq1_lat
            prev_seq1_lon = cur_seq1_lon
        
        # Enhanced notification for AUTO mode uploads
        upload_marker = ""
        if probable_upload:
            if w['mode'] == "AUTO":
                upload_marker = " <AUTO MODE UPLOAD!>"
            else:
                upload_marker = " <UPLOAD>"
        
        print(f"({t}) [Mode={w['mode']}] [Frame={frame_str(w['frame'])}] {mission_cmd_str(w)}{upload_marker}")

if __name__ == "__main__":
    main()
