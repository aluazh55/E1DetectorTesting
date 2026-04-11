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
    '-c', 'net3.sumocfg',
    '--step-length', '0.1',
    '--delay', '50'
]

traci.start(Sumo_config)

V_CONST = 13.89
D = 272.16
DET_UPSTREAM = "E2J1"
DET_QUEUE = "E2J2"
T_ACCEL = 10.0
DIST_CAR = 7.0
T_DELAY = 2.0
REFRESH_EVERY = 10  # every 10 steps = 1 simulated second

waiting_tracker = {}
seen_on_detector = set()
logged_vehicles = set()
live_vehicles = {}   # { veh_id: { speed, wait_so_far, state } }
event_log = []       # completed vehicle entries


def calculate_complex_reach_time(wait_time, veh_id, current_time):
    N = traci.lanearea.getLastStepHaltingNumber(DET_QUEUE)
    d_accel = (V_CONST * T_ACCEL) / 2
    d_queue = N * DIST_CAR
    d_remaining = D - d_queue - d_accel
    t_reach = T_ACCEL + (d_remaining / V_CONST) - (N * T_DELAY)
    status = "STOPPED" if wait_time >= 0.5 else "FLOWING"
    log_entry = (
        f"[{current_time:.1f}s] {veh_id} | {status} (waited {wait_time:.1f}s) | "
        f"N={N} | T2 = {T_ACCEL}s + ({D}-{N}*7-{d_accel:.1f}) / {V_CONST} "
        f"- {N}*{T_DELAY} = {t_reach:.2f}s"
    )
    return t_reach, N, status, log_entry


def print_dashboard(current_time):
    os.system('cls' if os.name == 'nt' else 'clear')

    N_queue = traci.lanearea.getLastStepHaltingNumber(DET_QUEUE)

    print("=" * 70)
    print(f"  SUMO LIVE MONITOR   |   Sim Time: {current_time:.1f}s   |   Queue at J2: {N_queue} vehicles")
    print("=" * 70)

    # --- Active vehicles ---
    print(f"\n  ACTIVE ON DETECTOR {DET_UPSTREAM}:")
    print(f"  {'Vehicle':<12} {'Speed (m/s)':>12} {'Waiting (s)':>12} {'State':<10}")
    print("  " + "-" * 50)
    if live_vehicles:
        for veh_id, data in live_vehicles.items():
            print(
                f"  {veh_id:<12} "
                f"{data['speed']:>12.2f} "
                f"{data['wait_so_far']:>12.1f} "
                f"{data['state']:<10}"
            )
    else:
        print("  (no vehicles on detector)")

    # --- Completed event log ---
    print(f"\n  COMPLETED EVENTS (last 10):")
    print("  " + "-" * 66)
    if event_log:
        for entry in event_log[-10:]:
            print(f"  {entry}")
    else:
        print("  (none yet)")

    print("=" * 70)


step_count = 0

while traci.simulation.getMinExpectedNumber() > 0:
    traci.simulationStep()
    current_time = traci.simulation.getTime()
    step_count += 1

    vehs_on_det = set(traci.lanearea.getLastStepVehicleIDs(DET_UPSTREAM))

    # --- Vehicles that just LEFT the detector ---
    just_left = seen_on_detector - vehs_on_det
    for veh_id in just_left:
        if veh_id not in logged_vehicles:
            wait = current_time - waiting_tracker[veh_id] if veh_id in waiting_tracker else 0.0
            _, _, _, log_entry = calculate_complex_reach_time(wait, veh_id, current_time)
            event_log.append(log_entry)
            logged_vehicles.add(veh_id)
        waiting_tracker.pop(veh_id, None)
        logged_vehicles.discard(veh_id)
        live_vehicles.pop(veh_id, None)

    seen_on_detector = vehs_on_det.copy()

    # --- Monitor every vehicle on detector ---
    for veh_id in vehs_on_det:
        speed = traci.vehicle.getSpeed(veh_id)

        if speed < 0.1:
            if veh_id not in waiting_tracker:
                waiting_tracker[veh_id] = current_time

        elif speed > 0.1 and veh_id in waiting_tracker and veh_id not in logged_vehicles:
            total_wait = current_time - waiting_tracker[veh_id]
            _, _, _, log_entry = calculate_complex_reach_time(total_wait, veh_id, current_time)
            event_log.append(log_entry)
            logged_vehicles.add(veh_id)
            del waiting_tracker[veh_id]

        wait_so_far = current_time - waiting_tracker[veh_id] if veh_id in waiting_tracker else 0.0
        live_vehicles[veh_id] = {
            "speed": speed,
            "wait_so_far": wait_so_far,
            "state": "STOPPED" if speed < 0.1 else "MOVING",
        }

    # --- Cleanup vanished vehicles ---
    all_vehs = set(traci.vehicle.getIDList())
    for vid in list(waiting_tracker.keys()):
        if vid not in all_vehs:
            del waiting_tracker[vid]
            live_vehicles.pop(vid, None)

    # --- Refresh every 1 simulated second ---
    if step_count % REFRESH_EVERY == 0:
        print_dashboard(current_time)

traci.close()
print("\nSimulation finished.")