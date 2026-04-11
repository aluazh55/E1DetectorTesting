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
    '-c', 'net2.sumocfg',
    '--step-length', '0.1',
    '--delay', '50'
]

# Step 5: Open connection
traci.start(Sumo_config)

# Step 6: Define Variables & Constants
# --- UPDATE THESE VALUES FROM NETEDIT ---
V_CONST = 13.89  # Target speed m/s (approx 50km/h)
D = 272.16  # Distance between junctions in meters
DETECTOR_ID = "E2J1"  # Your 150m LaneArea detector ID
# ----------------------------------------

T_ACCEL = 10.0  # Your assumed acceleration time
waiting_tracker = {}  # Stores {vehicleID: start_time}

# Open a log file to save results
log_file = open("simulation_log.txt", "w")


# Step 7: Define Functions
def calculate_reach_time(wait_time, veh_id):
    """Calculates reach time and returns a detailed breakdown of the math."""

    if wait_time < 5:
        # Case 1: Flowing
        t_reach = D / V_CONST
        status = "FLOWING"
        # Detailed log for Case 1
        math_log = f"T_reach = {D}m / {V_CONST}m/s = {t_reach:.2f}s"
    else:
        # Case 2: Stopped (Acceleration)
        d_accel = (V_CONST * T_ACCEL) / 2
        d_remaining = D - d_accel
        t_const = d_remaining / V_CONST
        t_reach = T_ACCEL + t_const

        status = "STOPPED"
        # Detailed log for Case 2
        math_log = (f"T_reach = {T_ACCEL}s (accel) + "
                    f"({D}m - ({V_CONST}m/s * {T_ACCEL}s)/2) / {V_CONST}m/s = "
                    f"{T_ACCEL}s + ({d_remaining:.2f}m / {V_CONST}m/s) = {t_reach:.2f}s")

    full_log = f"Vehicle: {veh_id} | Result: {status} ({wait_time:.1f}s) | {math_log}"
    return t_reach, full_log

# Step 8: Main Simulation Loop
print("Simulation started. Monitoring Detector:", DETECTOR_ID)

while traci.simulation.getMinExpectedNumber() > 0:
    traci.simulationStep()
    current_time = traci.simulation.getTime()

    # Get vehicles on the E2 detector (index 0 is the one at the stop line)
    vehs_on_det = traci.lanearea.getLastStepVehicleIDs(DETECTOR_ID)

    if vehs_on_det:
        lead_veh = vehs_on_det[0]
        speed = traci.vehicle.getSpeed(lead_veh)

        # A: Check if the lead vehicle is standing still
        if speed < 0.1:
            if lead_veh not in waiting_tracker:
                waiting_tracker[lead_veh] = current_time

        # B: Check if the lead vehicle was waiting and has now started moving
        elif speed > 0.1 and lead_veh in waiting_tracker:
            total_wait = current_time - waiting_tracker[lead_veh]

            # Perform calculation and generate log
            reach_time, log_entry = calculate_reach_time(total_wait, lead_veh)

            # Print to console and write to file
            print(log_entry)
            log_file.write(log_entry + "\n")

            # Remove from tracker so we only log once for this specific "start"
            del waiting_tracker[lead_veh]

    # Cleanup tracker for vehicles that leave the simulation or teleport
    all_vehs = traci.vehicle.getIDList()
    for vid in list(waiting_tracker.keys()):
        if vid not in all_vehs:
            del waiting_tracker[vid]

# Step 9: Close connection and save logs
log_file.close()
traci.close()
print("Simulation finished. Logs saved to simulation_log.txt")