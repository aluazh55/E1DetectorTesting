import os
import sys

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("Please declare environment variable 'SUMO_HOME'")

import traci

Sumo_config = [
    'sumo-gui',
    '-c', 'net2.sumocfg',
    '--step-length', '0.1',
    '--delay', '50'
]

traci.start(Sumo_config)

# Constants
V_CONST = 13.89
D = 272.16
DETECTOR_ID = "E2J1"
T_ACCEL = 10.0

waiting_tracker = {}  # { veh_id: start_idle_time }
seen_on_detector = set()  # vehicles currently on the detector
logged_vehicles = set()  # vehicles already logged this pass

# We keep the file for a permanent record, but print() handles the terminal
log_file = open("simulation_log.txt", "w")


def calculate_reach_time(wait_time, veh_id, is_flowing, current_step):
    """Calculates reach time and formats the detailed math string."""
    if is_flowing:
        t_reach = D / V_CONST
        status = "FLOWING"
        math_log = f"T_reach = {D}m / {V_CONST}m/s = {t_reach:.2f}s"
    else:
        d_accel = (V_CONST * T_ACCEL) / 2
        d_remaining = D - d_accel
        t_const = d_remaining / V_CONST
        t_reach = T_ACCEL + t_const
        status = "STOPPED"
        math_log = (
            f"T_reach = {T_ACCEL}s (accel) + "
            f"({D}m - ({V_CONST}m/s * {T_ACCEL}s)/2) / {V_CONST}m/s = "
            f"{T_ACCEL}s + ({d_remaining:.2f}m / {V_CONST}m/s) = {t_reach:.2f}s"
        )

    full_log = f"[Step: {current_step:.1f}s] Vehicle: {veh_id} | Result: {status} ({wait_time:.1f}s waited) | {math_log}"
    return t_reach, full_log


def log_vehicle(veh_id, wait_time, is_flowing, current_step):
    """Prints to terminal in real-time and writes to file."""
    reach_time, log_entry = calculate_reach_time(wait_time, veh_id, is_flowing, current_step)

    # print with flush=True ensures the text appears in your terminal immediately
    print(log_entry, flush=True)

    log_file.write(log_entry + "\n")
    logged_vehicles.add(veh_id)
    return reach_time


print(f"--- LIVE MONITORING STARTED ---")
print(f"Detector: {DETECTOR_ID} | Distance: {D}m | V_const: {V_CONST}m/s\n")

while traci.simulation.getMinExpectedNumber() > 0:
    traci.simulationStep()
    current_time = traci.simulation.getTime()

    vehs_on_det = set(traci.lanearea.getLastStepVehicleIDs(DETECTOR_ID))

    # --- Vehicles that just LEFT the detector ---
    just_left = seen_on_detector - vehs_on_det
    for veh_id in just_left:
        if veh_id not in logged_vehicles:
            log_vehicle(veh_id, 0.0, is_flowing=True, current_step=current_time)

        # Cleanup for re-entry
        waiting_tracker.pop(veh_id, None)
        logged_vehicles.discard(veh_id)

    seen_on_detector = vehs_on_det.copy()

    # --- Monitor vehicles currently on the detector ---
    for veh_id in vehs_on_det:
        if veh_id in logged_vehicles:
            continue

        speed = traci.vehicle.getSpeed(veh_id)

        # Vehicle stops
        if speed < 0.1:
            if veh_id not in waiting_tracker:
                waiting_tracker[veh_id] = current_time

        # Vehicle starts moving after a stop
        elif speed > 0.1 and veh_id in waiting_tracker:
            total_wait = current_time - waiting_tracker[veh_id]
            log_vehicle(veh_id, total_wait, is_flowing=False, current_step=current_time)
            del waiting_tracker[veh_id]

    # Cleanup vanished vehicles
    all_vehs = set(traci.vehicle.getIDList())
    for vid in list(waiting_tracker.keys()):
        if vid not in all_vehs:
            del waiting_tracker[vid]

log_file.close()
traci.close()
print("\n--- SIMULATION FINISHED ---")