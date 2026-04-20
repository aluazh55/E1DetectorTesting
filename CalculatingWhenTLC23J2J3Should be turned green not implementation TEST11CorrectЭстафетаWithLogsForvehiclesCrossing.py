import os
import sys
import traci

# --- SUMO Configuration
# черновик перед для test12---
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("Please declare environment variable 'SUMO_HOME'")

Sumo_config = [
    'sumo-gui', '-c', 'net3.sumocfg',
    '--step-length', '0.1', '--delay', '50'
]

# --- Constants & Physics ---
V_CONST = 13.89
D_12 = 272.16
D_23 = 269.84
T_ACCEL = 5.0
DIST_CAR = 5.0
T_DELAY = 2.0
SAFETY_GAP = 1.5
Turn_GreenEarlier_for = 1.5
CLEARANCE_BUFFER = 4.0

# --- Phase Mapping (From your XML) ---
J2_GREEN_PHASE = 2  # rG phase
J3_GREEN_PHASE = 1  # G phase
YELLOW_PHASE = {"J2": 3, "J3": 2}  # Explicit yellow phases

# --- State Trackers ---
pending_green = {"J2": None, "J3": None}
latest_green_release_time = {"J2": 0.0, "J3": 0.0}
green_locked = {"J2": False, "J3": False}

last_j2_native_j3_time = 0.0
last_j1_origin_j3_time = 0.0

entry_time_tracker_j1 = {}
entry_time_tracker_j2 = {}
logged_vehicles = set()
seen_on_detector_j1 = set()
seen_on_detector_j2 = set()
last_lane_tracker = {}

j3_last_phase = None
j3_phase_start_time = 0.0

DET_J1, DET_J2, DET_J3 = "E2J1", "E2J2", "E2J3"


# --- Core Planning Functions ---

def calculate_j2_plan(veh_id, entry_j1_time):
    """Plans J2 green light based on entry at J1."""
    N2 = traci.lanearea.getLastStepHaltingNumber(DET_J2)

    # Physics: Time to travel + half of acceleration time
    t_travel = (D_12 / V_CONST) + (T_ACCEL / 2)

    # When to open the light (factoring in the queue at J2)
    planned_start = entry_j1_time + t_travel - (N2 * T_DELAY) - Turn_GreenEarlier_for

    pending_green["J2"] = min(pending_green["J2"], planned_start) if pending_green["J2"] else planned_start
    latest_green_release_time["J2"] = max(latest_green_release_time["J2"],
                                          entry_j1_time + t_travel + 2.0 + CLEARANCE_BUFFER)

    logged_vehicles.add(veh_id)
    print(f" [PLAN J2] {veh_id} crossed J1 -> J2 Green @ {planned_start:.1f}s")


def _schedule_j3_relay(veh_id, cross_j2_time):
    """Relay trigger: Plans J3 green light based on actual exit from J2."""
    global last_j1_origin_j3_time, last_j2_native_j3_time
    N3 = traci.lanearea.getLastStepHaltingNumber(DET_J3)

    # Physics: Already moving, so no T_ACCEL needed
    t_travel = (D_23 / V_CONST)

    planned_start = cross_j2_time + t_travel - (N3 * T_DELAY) - Turn_GreenEarlier_for

    # Synchronize overlapping flows
    actual_start = max(planned_start, last_j1_origin_j3_time + SAFETY_GAP, last_j2_native_j3_time + SAFETY_GAP)

    if veh_id in logged_vehicles:
        last_j1_origin_j3_time = actual_start
    else:
        last_j2_native_j3_time = actual_start

    pending_green["J3"] = min(pending_green["J3"], actual_start) if pending_green["J3"] else actual_start
    latest_green_release_time["J3"] = max(latest_green_release_time["J3"],
                                          cross_j2_time + t_travel + 2.0 + CLEARANCE_BUFFER)

    print(f"  [RELAY J3] {veh_id} cleared J2 @ {cross_j2_time:.1f}s -> J3 Green @ {actual_start:.1f}s")


# --- Logging Function ---

def monitor_crossings(veh_id, current_time):
    """Optional: Just prints logs when cars cross intersections for debugging."""
    current_lane = traci.vehicle.getLaneID(veh_id)
    if veh_id in last_lane_tracker:
        prev_lane = last_lane_tracker[veh_id]
        if current_lane != prev_lane:
            if "E0_0" in prev_lane and "E0_0" not in current_lane:
                print(f" >>> [CROSS] {veh_id} passed J1 at {current_time:.1f}s")
            elif "E1_0" in prev_lane and "E1_0" not in current_lane:
                print(f" >>> [CROSS] {veh_id} passed J2 at {current_time:.1f}s")
            elif "E2_0" in prev_lane and "E2_0" not in current_lane:
                print(f" >>> [CROSS] {veh_id} passed J3 at {current_time:.1f}s")
    last_lane_tracker[veh_id] = current_lane


# --- Main Simulation Loop ---

traci.start(Sumo_config)
print("--- LIVE GREEN WAVE MONITORING STARTED (RELAY MODE) ---")

while traci.simulation.getMinExpectedNumber() > 0:
    traci.simulationStep()
    current_time = traci.simulation.getTime()
    active_vehs = traci.vehicle.getIDList()

    for v_id in active_vehs:
        monitor_crossings(v_id, current_time)

        # --- 1. TLC LOCK LOGIC (Triggering the Green Wave) ---
        for junc, g_phase in [("J2", J2_GREEN_PHASE), ("J3", J3_GREEN_PHASE)]:
            t_trig = pending_green[junc]
            if t_trig is not None and current_time >= t_trig:
                if traci.trafficlight.getPhase(junc) != g_phase:
                    traci.trafficlight.setPhase(junc, g_phase)

                traci.trafficlight.setPhaseDuration(junc, 9999)  # Замораживаем фазу

                if not green_locked[junc]:
                    print(f"  [TLC] {junc} -> LOCKED GREEN (Phase {g_phase}) @ {current_time:.1f}s")
                    green_locked[junc] = True
                    # ВНИМАНИЕ: Я убрал отсюда обнуление latest_green_release_time.
                    # Оно стирало рассчитанное время!

                pending_green[junc] = None

        # --- 2. TLC RELEASE LOGIC (Ending the Green Wave) ---
        for junc in ["J2", "J3"]:
            if green_locked[junc] and current_time >= latest_green_release_time[junc]:
                y_phase = YELLOW_PHASE[junc]
                traci.trafficlight.setPhase(junc, y_phase)
                traci.trafficlight.setPhaseDuration(junc, 3)
                print(f"  [TLC] {junc} -> RELEASED to yellow (Phase {y_phase}) @ {current_time:.1f}s")
                green_locked[junc] = False

                # Сброс старого значения должен происходить ТОЛЬКО ЗДЕСЬ,
                # когда группа машин уже проехала и мы вернули светофор в статику.
                latest_green_release_time[junc] = 0.0

    # --- 3. J3 PHASE LOGGING ---
    curr_p_j3 = traci.trafficlight.getPhase("J3")
    if curr_p_j3 != j3_last_phase:
        if j3_last_phase is not None:
            dur = current_time - j3_phase_start_time
            print(f"      [LOG J3] Phase {j3_last_phase} lasted {dur:.1f}s")
        j3_last_phase = curr_p_j3
        j3_phase_start_time = current_time

    # --- 4. DETECTORS & RELAY TRIGGERS ---

    # J1 Detector (Initiates the process)
    vehs_j1 = set(traci.lanearea.getLastStepVehicleIDs(DET_J1))
    for v in (vehs_j1 - seen_on_detector_j1):
        entry_time_tracker_j1[v] = current_time
    for v in (seen_on_detector_j1 - vehs_j1):
        if v not in logged_vehicles:
            calculate_j2_plan(v, current_time)
    seen_on_detector_j1 = vehs_j1

    # J2 Detector (Triggers J3 upon exit - The Relay Model)
    vehs_j2 = set(traci.lanearea.getLastStepVehicleIDs(DET_J2))
    for v in (vehs_j2 - seen_on_detector_j2):
        entry_time_tracker_j2[v] = current_time
    for v in (seen_on_detector_j2 - vehs_j2):
        # A vehicle (either from J1 or side-flow E5) cleared J2 -> Trigger J3
        _schedule_j3_relay(v, current_time)
        entry_time_tracker_j2.pop(v, None)  # Clean up immediately
    seen_on_detector_j2 = vehs_j2

    # --- 5. MEMORY CLEANUP ---
    for d in [entry_time_tracker_j1, entry_time_tracker_j2, last_lane_tracker]:
        for vid in list(d.keys()):
            if vid not in active_vehs:
                del d[vid]

traci.close()