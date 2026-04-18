import os
import sys
import traci

# --- SUMO Configuration ---
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("Please declare environment variable 'SUMO_HOME'")

Sumo_config = [
    'sumo-gui', '-c', 'net3.sumocfg',
    '--step-length', '0.1', '--delay', '50'
]
traci.start(Sumo_config)

# --- Constants ---
V_CONST = 13.89
D_12 = 272.16
D_23 = 269.84
T_ACCEL = 5.0
DIST_CAR = 7.0
T_DELAY = 2.0
SAFETY_GAP = 1.5

DET_J1 = "E2J1"
DET_J2 = "E2J2"
DET_J3 = "E2J3"

# --- State Trackers ---
waiting_tracker_j1 = {}
entry_time_tracker_j1 = {}
seen_on_detector_j1 = set()

waiting_tracker_j2 = {}
entry_time_tracker_j2 = {}
seen_on_detector_j2 = set()

logged_vehicles = set()
log_file = open("simulation_log.txt", "w")

# --- FIX: REPLACED SINGLE GLOBAL WITH DUAL COUNTERS ---
last_j2_native_j3_time = 0.0  # Lead group already at J2
last_j1_origin_j3_time = 0.0  # Vehicles still en-route from J1


def calculate_green_wave_plan(veh_id, departure_time, start_at_j1=True):
    global last_j2_native_j3_time, last_j1_origin_j3_time

    N2 = traci.lanearea.getLastStepHaltingNumber(DET_J2)
    N3 = traci.lanearea.getLastStepHaltingNumber(DET_J3)
    d_accel = (V_CONST * T_ACCEL) / 2

    if start_at_j1:
        t_reach_12 = T_ACCEL + (D_12 - (N2 * DIST_CAR) - d_accel) / V_CONST - (N2 * T_DELAY)
        tlc_j2_green = departure_time + t_reach_12
        t_reach_23 = (D_23 - (N3 * DIST_CAR)) / V_CONST - (N3 * T_DELAY)
        ideal_j3_arrival = tlc_j2_green + t_reach_23

        # Must respect BOTH streams — merge behind whichever is later
        actual_j3_green = max(
            ideal_j3_arrival,
            last_j1_origin_j3_time + SAFETY_GAP,
            last_j2_native_j3_time + SAFETY_GAP   # ← respect lead group
        )
        last_j1_origin_j3_time = actual_j3_green

        path_info = f"Full Path | J2 Green: {tlc_j2_green:.2f}s"
    else:
        t_reach_23 = T_ACCEL + (D_23 - (N3 * DIST_CAR) - d_accel) / V_CONST - (N3 * T_DELAY)
        ideal_j3_arrival = departure_time + t_reach_23

        # J2-native: independent lead group, ignores J1-origin counter
        actual_j3_green = max(ideal_j3_arrival, last_j2_native_j3_time + SAFETY_GAP)
        last_j2_native_j3_time = actual_j3_green
        path_info = "Starts at E1/J2"

    adjustment_note = "(Adjusted for Traffic)" if actual_j3_green > ideal_j3_arrival else ""

    log_entry = (
        f"\n[Dep. Time: {departure_time:.1f}s] Vehicle: {veh_id} ({path_info})\n"
        f"  >> Ideal J3 Arrival: {ideal_j3_arrival:.2f}s\n"
        f"  >> ACTUAL J3 GREEN:  {actual_j3_green:.2f}s {adjustment_note}\n"
    )
    return log_entry

def log_event(veh_id, departure_time, start_at_j1=True):
    entry = calculate_green_wave_plan(veh_id, departure_time, start_at_j1)
    print(entry, flush=True)
    log_file.write(entry + "\n")
    logged_vehicles.add(veh_id)


# --- Main Loop ---
print("--- LIVE GREEN WAVE MONITORING STARTED (DUAL-STREAM MODE) ---")

while traci.simulation.getMinExpectedNumber() > 0:
    traci.simulationStep()
    current_time = traci.simulation.getTime()

    # Process J2 (Natives) first
    vehs_on_det_j2 = set(traci.lanearea.getLastStepVehicleIDs(DET_J2))
    for veh_id in (vehs_on_det_j2 - seen_on_detector_j2):
        if veh_id not in logged_vehicles and veh_id not in entry_time_tracker_j1:
            entry_time_tracker_j2[veh_id] = current_time

    for veh_id in (seen_on_detector_j2 - vehs_on_det_j2):
        if veh_id in entry_time_tracker_j2 and veh_id not in logged_vehicles:
            log_event(veh_id, entry_time_tracker_j2.pop(veh_id), start_at_j1=False)

    for veh_id in vehs_on_det_j2:
        if veh_id in logged_vehicles or veh_id in entry_time_tracker_j1: continue
        speed = traci.vehicle.getSpeed(veh_id)
        if speed < 0.1:
            waiting_tracker_j2.setdefault(veh_id, current_time)
        elif speed > 0.1 and veh_id in waiting_tracker_j2:
            log_event(veh_id, current_time, start_at_j1=False)
            waiting_tracker_j2.pop(veh_id)
    seen_on_detector_j2 = vehs_on_det_j2.copy()

    # Process J1 (Incoming)
    vehs_on_det_j1 = set(traci.lanearea.getLastStepVehicleIDs(DET_J1))
    for veh_id in (vehs_on_det_j1 - seen_on_detector_j1):
        entry_time_tracker_j1[veh_id] = current_time

    for veh_id in (seen_on_detector_j1 - vehs_on_det_j1):
        if veh_id not in logged_vehicles:
            log_event(veh_id, entry_time_tracker_j1.get(veh_id, current_time), start_at_j1=True)
        entry_time_tracker_j1.pop(veh_id, None)

    for veh_id in vehs_on_det_j1:
        if veh_id in logged_vehicles: continue
        speed = traci.vehicle.getSpeed(veh_id)
        if speed < 0.1:
            waiting_tracker_j1.setdefault(veh_id, current_time)
        elif speed > 0.1 and veh_id in waiting_tracker_j1:
            log_event(veh_id, current_time, start_at_j1=True)
            waiting_tracker_j1.pop(veh_id)
    seen_on_detector_j1 = vehs_on_det_j1.copy()

    # Cleanup
    all_vehs = set(traci.vehicle.getIDList())
    for d in [waiting_tracker_j1, entry_time_tracker_j1, waiting_tracker_j2, entry_time_tracker_j2]:
        for vid in list(d.keys()):
            if vid not in all_vehs: del d[vid]

log_file.close()
traci.close()
print("\n--- SIMULATION FINISHED ---")