# Step 1: Add modules
import os
import sys

# Step 2: Establish path to SUMO
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("Please declare environment variable 'SUMO_HOME'")

# Step 3: Add Traci module
import traci

# Step 4: Define Sumo configuration
Sumo_config = [
    'sumo-gui',
    '-c', 'net3.sumocfg',
    '--step-length', '0.1',
    '--delay', '50'
]

# Step 5: Open connection
traci.start(Sumo_config)

# Step 6: Define Variables & Constants
V_CONST = 13.89  # Target speed m/s
D = 272.16  # Distance between junctions in meters
DETECTOR_ID = "E2J1"  # Your 150m LaneArea detector ID
T_ACCEL = 10.0  # Assumed acceleration time

waiting_tracker = {}  # Stores {vehicleID: start_idling_time}
processed_vehicles = set()  # Stores vehicles already logged for this detector pass

# Open a log file
log_file = open("simulation_log.txt", "w")


# Step 7: Define Functions
def calculate_reach_time(wait_time, veh_id):
    """Calculates reach time and provides a detailed math breakdown."""
    if wait_time < 5:
        # Case 1: Flowing
        t_reach = D / V_CONST
        status = "FLOWING"
        math_log = f"T_reach = {D}m / {V_CONST}m/s = {t_reach:.2f}s"
    else:
        # Case 2: Stopped (Acceleration Formula)
        d_accel = (V_CONST * T_ACCEL) / 2
        d_remaining = D - d_accel
        t_const = d_remaining / V_CONST
        t_reach = T_ACCEL + t_const

        status = "STOPPED"
        math_log = (f"T_reach = {T_ACCEL}s(accel) + "
                    f"({D}m - ({V_CONST}*10)/2) / {V_CONST} = "
                    f"{T_ACCEL} + ({d_remaining:.2f} / {V_CONST}) = {t_reach:.2f}s")

    full_log = f"Vehicle: {veh_id} | Result: {status} ({wait_time:.1f}s) | {math_log}"
    return t_reach, full_log


# Step 8: Main Simulation Loop
print(f"Simulation started. Logging to 'simulation_log.txt'...")

while traci.simulation.getMinExpectedNumber() > 0:
    traci.simulationStep()
    current_time = traci.simulation.getTime()

    # Get IDs of all vehicles currently on the 150m detector
    vehs_on_det = traci.lanearea.getLastStepVehicleIDs(DETECTOR_ID)

    if vehs_on_det:
        # Monitor the lead vehicle
        lead_veh = vehs_on_det[0]

        # Only process if we haven't logged this specific vehicle in its current pass
        if lead_veh not in processed_vehicles:
            speed = traci.vehicle.getSpeed(lead_veh)

            # A: If car is STOPPED, start the wait timer
            if speed < 0.1:
                if lead_veh not in waiting_tracker:
                    waiting_tracker[lead_veh] = current_time

            # B: If car was WAITING and now STARTS MOVING (net2 scenario)
            elif speed > 0.1 and lead_veh in waiting_tracker:
                total_wait = current_time - waiting_tracker[lead_veh]
                reach_time, log_entry = calculate_reach_time(total_wait, lead_veh)

                print(log_entry)
                log_file.write(log_entry + "\n")

                processed_vehicles.add(lead_veh)  # Mark as logged
                del waiting_tracker[lead_veh]

            # C: If car is FLOWING THROUGH (net3 scenario)
            # We trigger this if the car is moving and didn't stop previously
            elif speed > (V_CONST * 0.8) and lead_veh not in waiting_tracker:
                reach_time, log_entry = calculate_reach_time(0.0, lead_veh)

                print(log_entry)
                log_file.write(log_entry + "\n")

                processed_vehicles.add(lead_veh)  # Mark as logged

    # --- Cleanup Section ---
    # Remove vehicles from processed set once they leave the detector
    # This allows them to be logged again if they pass through later in the simulation
    for vid in list(processed_vehicles):
        if vid not in vehs_on_det:
            processed_vehicles.remove(vid)

    # Cleanup tracker for vehicles that might vanish/teleport
    all_vehs = traci.vehicle.getIDList()
    for vid in list(waiting_tracker.keys()):
        if vid not in all_vehs:
            del waiting_tracker[vid]

# Step 9: Close
log_file.close()
traci.close()
print("Simulation finished.")