import os
import sys
import traci

# --- SETUP ---
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("Please declare environment variable 'SUMO_HOME'")

Sumo_config = ['sumo-gui', '-c', 'net3.sumocfg', '--step-length', '0.1', '--delay', '50']
traci.start(Sumo_config)

# --- CONSTANTS (From your successful manual test) ---
V_CONST = 13.89
D_12 = 272.16
D_23 = 269.84  # Updated to match your manual calculation distance
T_ACCEL = 10.0
DIST_CAR = 7.0
T_DELAY = 2.0

DET_J1 = "E2J1"
DET_J2 = "E2J2"
DET_J3 = "E2J3"

last_calculation_time = -1


def calculate_platoon_wave(veh_id, current_step):
    global last_calculation_time

    # Get current queue counts
    N2 = traci.lanearea.getLastStepHaltingNumber(DET_J2)
    N3 = traci.lanearea.getLastStepHaltingNumber(DET_J3)

    # --- LEG 1: J1 to J2 ---
    # Formula: current_step + T_accel + (Remaining Dist / V) - (N2 * T_delay)
    d_accel = (V_CONST * T_ACCEL) / 2
    d_remaining_12 = D_12 - (N2 * DIST_CAR) - d_accel
    t_reach_12 = T_ACCEL + (d_remaining_12 / V_CONST) - (N2 * T_DELAY)
    tlc_j2_green = current_step + t_reach_12

    # --- LEG 2: J2 to J3 ---
    # Formula: TLC_J2_Green + (Remaining Dist / V) - (N3 * T_delay)
    # Note: We do not add N2 delay here to match your 'reality' results
    d_remaining_23 = D_23 - (N3 * DIST_CAR)
    t_reach_23 = (d_remaining_23 / V_CONST) - (N3 * T_DELAY)
    tlc_j3_green = tlc_j2_green + t_reach_23

    # Display results exactly like your manual format
    print(f"\n--- PLATOON WAVE INITIATED BY {veh_id} ---")
    print(f"[Step: {current_step:.1f}s] Vehicle: {veh_id} DEPARTED J1")
    print(
        f"  >> J2 Plan: {current_step:.1f} + ({D_12} - {N2 * 7} - {d_accel:.1f})/{V_CONST} - {N2 * 2} = {tlc_j2_green:.2f}s")
    print(f"  >> J3 Plan: {tlc_j2_green:.2f} + ({D_23} - {N3 * 7})/{V_CONST} - {N3 * 2} = {tlc_j3_green:.2f}s")
    print("-" * 50)

    last_calculation_time = current_step


# --- MAIN LOOP ---
while traci.simulation.getMinExpectedNumber() > 0:
    traci.simulationStep()
    current_time = traci.simulation.getTime()

    vehs_j1 = traci.lanearea.getLastStepVehicleIDs(DET_J1)

    if vehs_j1:
        lead_veh = vehs_j1[0]
        speed = traci.vehicle.getSpeed(lead_veh)

        # Trigger on movement, but only once per 20 seconds to catch the platoon leader
        if speed > 0.1 and (current_time - last_calculation_time) > 20:
            calculate_platoon_wave(lead_veh, current_time)

traci.close()