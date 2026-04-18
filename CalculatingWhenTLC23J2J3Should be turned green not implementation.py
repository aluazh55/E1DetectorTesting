import os
import sys
import traci

# Step 1-3: Setup SUMO_HOME and TraCI
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("Please declare environment variable 'SUMO_HOME'")

# Step 4: Define Sumo configuration
Sumo_config = [
    'sumo-gui',
    '-c', 'net3.sumocfg',
    '--step-length', '0.1',
    '--delay', '50'
]

traci.start(Sumo_config)

# --- Configuration & Constants ---
V_CONST = 13.89
D_12 = 272.16  # Distance J1 to J2
D_23 = 269.84  # Update with your J2 to J3 distance from Netedit
T_ACCEL = 10.0
DIST_CAR = 7.0
T_DELAY = 2.0

DET_J1 = "E2J1"
DET_J2 = "E2J2"
DET_J3 = "E2J3"

waiting_tracker = {}
seen_on_detector = set()
logged_vehicles = set()

log_file = open("simulation_log.txt", "w")


# --- Logic Functions ---

def calculate_green_wave_plan(wait_time, veh_id, current_step):
    """Calculates the chain reaction for J2 and J3."""
    # Get N for upcoming junctions
    N2 = traci.lanearea.getLastStepHaltingNumber(DET_J2)
    N3 = traci.lanearea.getLastStepHaltingNumber(DET_J3)

    # 1. Junction 2 Calculation (Leg 1)
    d_accel = (V_CONST * T_ACCEL) / 2
    t_reach_12 = T_ACCEL + (D_12 - (N2 * DIST_CAR) - d_accel) / V_CONST - (N2 * T_DELAY)
    tlc_j2_green = current_step + t_reach_12

    # 2. Junction 3 Calculation (Leg 2)
    # Formula: J2_Green + (Dist23 - N3*7)/V - (N3 * Delay)
    t_reach_23 = (D_23 - (N3 * DIST_CAR)) / V_CONST - (N3 * T_DELAY)
    tlc_j3_green = tlc_j2_green + t_reach_23

    # Format the Log Entry
    log_entry = (
        f"\n[Step: {current_step:.1f}s] Vehicle: {veh_id} DEPARTED J1\n"
        f"  >> J2 Plan: {current_step:.1f} + ({D_12} - {N2 * 7} - {d_accel:.1f})/{V_CONST} - {N2 * 2} = {tlc_j2_green:.2f}s\n"
        f"  >> J3 Plan: {tlc_j2_green:.2f} + ({D_23} - {N3 * 7})/{V_CONST} - {N3 * 2} = {tlc_j3_green:.2f}s\n"
    )
    return log_entry


def log_event(veh_id, wait_time, current_step):
    log_entry = calculate_green_wave_plan(wait_time, veh_id, current_step)
    print(log_entry, flush=True)
    log_file.write(log_entry + "\n")
    logged_vehicles.add(veh_id)


# --- Main Simulation Loop ---
print(f"--- LIVE GREEN WAVE MONITORING STARTED ---")

while traci.simulation.getMinExpectedNumber() > 0:
    traci.simulationStep()
    current_time = traci.simulation.getTime()

    # Get vehicles on the primary monitoring detector
    vehs_on_det = set(traci.lanearea.getLastStepVehicleIDs(DET_J1))

    # --- Trigger 1: Flowing through (Left without stopping) ---
    just_left = seen_on_detector - vehs_on_det
    for veh_id in just_left:
        if veh_id not in logged_vehicles:
            log_event(veh_id, 0.0, current_time)
        # Reset for next possible pass
        waiting_tracker.pop(veh_id, None)
        logged_vehicles.discard(veh_id)

    seen_on_detector = vehs_on_det.copy()

    # --- Trigger 2: Stop-and-Go (Instant detection on movement) ---
    for veh_id in vehs_on_det:
        if veh_id in logged_vehicles:
            continue

        speed = traci.vehicle.getSpeed(veh_id)

        if speed < 0.1:
            if veh_id not in waiting_tracker:
                waiting_tracker[veh_id] = current_time

        elif speed > 0.1 and veh_id in waiting_tracker:
            # THIS IS THE FIX: This triggers the moment speed hits 0.11+
            # so the [Step] will show 53.0s instead of 57.0s
            total_wait = current_time - waiting_tracker[veh_id]
            log_event(veh_id, total_wait, current_time)
            del waiting_tracker[veh_id]

    # Clean up vanished vehicles
    all_vehs = set(traci.vehicle.getIDList())
    for vid in list(waiting_tracker.keys()):
        if vid not in all_vehs:
            del waiting_tracker[vid]

log_file.close()
traci.close()
print("\n--- SIMULATION FINISHED ---")