import os
import sys
import traci

# --- SUMO Environment Setup ---
# --- Машины логгируются правильно, но сама логика срадает, потому что машины с первого перексретка прибывают на 3 намного
# раньше чем со второгоБ что невозможно---

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
V_CONST = 13.89  # Target cruise speed (m/s)
D_12 = 272.16  # Distance J1 to J2
D_23 = 269.84  # Distance J2 to J3
T_ACCEL = 5.0  # Time to reach V_CONST
DIST_CAR = 7.0  # Average space occupied by a car in queue
T_DELAY = 2.0  # Time delay per vehicle to start moving

DET_J1 = "E2J1"
DET_J2 = "E2J2"
DET_J3 = "E2J3"

# --- State Management ---
# J1 Flow Tracking
waiting_tracker_j1 = {}  # veh_id -> time stopped at J1
entry_time_tracker_j1 = {}  # veh_id -> time entered J1 detector
seen_on_detector_j1 = set()

# J2 Flow Tracking (For vehicles starting at E1)
waiting_tracker_j2 = {}  # veh_id -> time stopped at J2
entry_time_tracker_j2 = {}  # veh_id -> time entered J2 detector
seen_on_detector_j2 = set()

logged_vehicles = set()
log_file = open("simulation_log.txt", "w")


def calculate_green_wave_plan(veh_id, departure_time, start_at_j1=True):
    """
    Computes Green Wave timings.
    If start_at_j1 is False, it assumes the vehicle starts at E1/J2.
    """
    N2 = traci.lanearea.getLastStepHaltingNumber(DET_J2)
    N3 = traci.lanearea.getLastStepHaltingNumber(DET_J3)
    d_accel = (V_CONST * T_ACCEL) / 2  # Distance covered during acceleration

    if start_at_j1:
        # Full path calculation (J1 -> J2 -> J3)
        t_reach_12 = T_ACCEL + (D_12 - (N2 * DIST_CAR) - d_accel) / V_CONST - (N2 * T_DELAY)
        tlc_j2_green = departure_time + t_reach_12

        t_reach_23 = (D_23 - (N3 * DIST_CAR)) / V_CONST - (N3 * T_DELAY)
        tlc_j3_green = tlc_j2_green + t_reach_23

        log_entry = (
            f"\n[Departure: {departure_time:.1f}s] Vehicle: {veh_id} (Full Path)\n"
            f"  N2={N2} queued at J2, N3={N3} at J3\n"
            f"  >> J2 green at: {departure_time:.1f} + {T_ACCEL} + "
            f"({D_12} - {N2 * DIST_CAR} - {d_accel:.1f})/{V_CONST} "
            f"+ {N2} x {T_DELAY} = {tlc_j2_green:.2f}s\n"
            f"  >> J3 green at: {tlc_j2_green:.2f} + "
            f"({D_23} - {N3 * DIST_CAR})/{V_CONST} "
            f"- {N3} x {T_DELAY} = {tlc_j3_green:.2f}s\n"
        )
    else:
        # Partial path calculation (J2 -> J3 only)
        # Vehicle must accelerate from stop/slow at J2
        t_reach_23 = T_ACCEL + (D_23 - (N3 * DIST_CAR) - d_accel) / V_CONST + (N3 * T_DELAY)
        tlc_j3_green = departure_time + t_reach_23

        log_entry = (
            f"\n[Departure: {departure_time:.1f}s] Vehicle: {veh_id} (Starts at E1/J2)\n"
            f"  N3={N3} queued at J3\n"
            f"  >> J3 green at: {departure_time:.1f} + {T_ACCEL} + "
            f"({D_23} - {N3 * DIST_CAR} - {d_accel:.1f})/{V_CONST} "
            f"- {N3} x {T_DELAY} = {tlc_j3_green:.2f}s\n"
        )
    return log_entry


def log_event(veh_id, departure_time, start_at_j1=True):
    entry = calculate_green_wave_plan(veh_id, departure_time, start_at_j1)
    print(entry, flush=True)
    log_file.write(entry + "\n")
    logged_vehicles.add(veh_id)


# --- Main Simulation Loop ---
print("--- LIVE GREEN WAVE MONITORING STARTED ---")

while traci.simulation.getMinExpectedNumber() > 0:
    traci.simulationStep()
    current_time = traci.simulation.getTime()

    # --- 1. J1 DETECTOR LOGIC (Full Path Flow) ---
    vehs_on_det_j1 = set(traci.lanearea.getLastStepVehicleIDs(DET_J1))

    # Track new entries
    for veh_id in (vehs_on_det_j1 - seen_on_detector_j1):
        if veh_id not in entry_time_tracker_j1:
            entry_time_tracker_j1[veh_id] = current_time

    # Handle Departure (Flow-through)
    for veh_id in (seen_on_detector_j1 - vehs_on_det_j1):
        if veh_id not in logged_vehicles:
            dep_time = entry_time_tracker_j1.get(veh_id, current_time)
            log_event(veh_id, dep_time, start_at_j1=True)
        entry_time_tracker_j1.pop(veh_id, None)
        waiting_tracker_j1.pop(veh_id, None)

    # Handle Stop-and-Go at J1
    for veh_id in vehs_on_det_j1:
        if veh_id in logged_vehicles: continue
        speed = traci.vehicle.getSpeed(veh_id)
        if speed < 0.1:
            waiting_tracker_j1.setdefault(veh_id, current_time)
        elif speed > 0.1 and veh_id in waiting_tracker_j1:
            log_event(veh_id, current_time, start_at_j1=True)
            waiting_tracker_j1.pop(veh_id)

    seen_on_detector_j1 = vehs_on_det_j1.copy()

    # --- 2. J2 DETECTOR LOGIC (E1 Entries Only) ---
    vehs_on_det_j2 = set(traci.lanearea.getLastStepVehicleIDs(DET_J2))

    # Guard: Only track if NOT coming from J1 and NOT already logged
    for veh_id in (vehs_on_det_j2 - seen_on_detector_j2):
        if veh_id not in logged_vehicles and veh_id not in entry_time_tracker_j1:
            if veh_id not in entry_time_tracker_j2:
                entry_time_tracker_j2[veh_id] = current_time

    # Handle Departure (Flow-through at J2 for E1 entries)
    for veh_id in (seen_on_detector_j2 - vehs_on_det_j2):
        if veh_id in entry_time_tracker_j2 and veh_id not in logged_vehicles:
            dep_time = entry_time_tracker_j2.get(veh_id, current_time)
            log_event(veh_id, dep_time, start_at_j1=False)
        entry_time_tracker_j2.pop(veh_id, None)
        waiting_tracker_j2.pop(veh_id, None)

    # Handle Stop-and-Go at J2 for E1 entries
    for veh_id in vehs_on_det_j2:
        if veh_id in logged_vehicles or veh_id in entry_time_tracker_j1:
            continue
        speed = traci.vehicle.getSpeed(veh_id)
        if speed < 0.1:
            waiting_tracker_j2.setdefault(veh_id, current_time)
        elif speed > 0.1 and veh_id in waiting_tracker_j2:
            log_event(veh_id, current_time, start_at_j1=False)
            waiting_tracker_j2.pop(veh_id)

    seen_on_detector_j2 = vehs_on_det_j2.copy()

    # --- 3. Cleanup ---
    all_vehs = set(traci.vehicle.getIDList())
    for d in [waiting_tracker_j1, entry_time_tracker_j1, waiting_tracker_j2, entry_time_tracker_j2]:
        for vid in list(d.keys()):
            if vid not in all_vehs:
                del d[vid]

log_file.close()
traci.close()
print("\n--- SIMULATION FINISHED ---")