import os
import sys
import traci

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
V_CONST  = 13.89
D_12     = 272.16
D_23     = 269.84
T_ACCEL  = 10.0
DIST_CAR = 7.0
T_DELAY  = 2.0

DET_J1 = "E2J1"
DET_J2 = "E2J2"
DET_J3 = "E2J3"

# --- State ---
waiting_tracker   = {}   # veh_id -> time stopped
entry_time_tracker = {}  # FIX 1: record when vehicle first appears on J1 det
seen_on_detector  = set()
logged_vehicles   = set()

log_file = open("simulation_log.txt", "w")


def calculate_green_wave_plan(veh_id, departure_time):
    """
    Given the simulation time the vehicle departs J1,
    compute when J2 and J3 lights must turn green.
    """
    N2 = traci.lanearea.getLastStepHaltingNumber(DET_J2)
    N3 = traci.lanearea.getLastStepHaltingNumber(DET_J3)

    d_accel = (V_CONST * T_ACCEL) / 2   # distance covered while accelerating

    # J2: vehicle accelerates for T_ACCEL s, then cruises; subtract queue clearance time
    t_reach_12  = T_ACCEL + (D_12 - (N2 * DIST_CAR) - d_accel) / V_CONST - (N2 * T_DELAY)
    tlc_j2_green = departure_time + t_reach_12

    # J3: vehicle departs J2 at tlc_j2_green, cruises D_23 minus queue
    t_reach_23  = (D_23 - (N3 * DIST_CAR)) / V_CONST - (N3 * T_DELAY)
    tlc_j3_green = tlc_j2_green + t_reach_23

    # FIX 3: use T_DELAY variable in the log string (not hardcoded 2)
    log_entry = (
        f"\n[Departure: {departure_time:.1f}s] Vehicle: {veh_id}\n"
        f"  N2={N2} queued cars at J2, N3={N3} at J3\n"
        f"  >> J2 green at: {departure_time:.1f} + {T_ACCEL} + "
        f"({D_12} - {N2 * DIST_CAR} - {d_accel:.1f})/{V_CONST} "
        f"- {N2} x {T_DELAY} = {tlc_j2_green:.2f}s\n"
        f"  >> J3 green at: {tlc_j2_green:.2f} + "
        f"({D_23} - {N3 * DIST_CAR})/{V_CONST} "
        f"- {N3} x {T_DELAY} = {tlc_j3_green:.2f}s\n"
    )
    return log_entry


def log_event(veh_id, departure_time):
    entry = calculate_green_wave_plan(veh_id, departure_time)
    print(entry, flush=True)
    log_file.write(entry + "\n")
    logged_vehicles.add(veh_id)


# --- Main Loop ---
print("--- LIVE GREEN WAVE MONITORING STARTED ---")

while traci.simulation.getMinExpectedNumber() > 0:
    traci.simulationStep()
    current_time = traci.simulation.getTime()

    vehs_on_det = set(traci.lanearea.getLastStepVehicleIDs(DET_J1))

    # FIX 1: record entry time the moment a vehicle first appears on J1
    for veh_id in vehs_on_det - seen_on_detector:
        if veh_id not in entry_time_tracker:
            entry_time_tracker[veh_id] = current_time

    # --- Trigger A: Flowing through (left without stopping) ---
    just_left = seen_on_detector - vehs_on_det
    for veh_id in just_left:
        if veh_id not in logged_vehicles:
            # FIX 1: use recorded entry time, not current_time (which is one step late)
            departure_time = entry_time_tracker.get(veh_id, current_time)
            log_event(veh_id, departure_time)
        waiting_tracker.pop(veh_id, None)
        entry_time_tracker.pop(veh_id, None)
        # FIX 2: do NOT discard from logged_vehicles — prevents re-logging on re-entry
        # logged_vehicles intentionally kept; clear only if you want per-cycle logging

    seen_on_detector = vehs_on_det.copy()

    # --- Trigger B: Stop-and-Go (fires the moment speed recovers) ---
    for veh_id in vehs_on_det:
        if veh_id in logged_vehicles:
            continue

        speed = traci.vehicle.getSpeed(veh_id)

        if speed < 0.1:
            if veh_id not in waiting_tracker:
                waiting_tracker[veh_id] = current_time  # record stop time

        elif speed > 0.1 and veh_id in waiting_tracker:
            # Departure time = NOW (the step the vehicle starts moving)
            log_event(veh_id, current_time)
            del waiting_tracker[veh_id]

    # Clean up vehicles that have vanished from the simulation
    all_vehs = set(traci.vehicle.getIDList())
    for vid in list(waiting_tracker.keys()):
        if vid not in all_vehs:
            del waiting_tracker[vid]
    for vid in list(entry_time_tracker.keys()):
        if vid not in all_vehs:
            del entry_time_tracker[vid]

log_file.close()
traci.close()
print("\n--- SIMULATION FINISHED ---")