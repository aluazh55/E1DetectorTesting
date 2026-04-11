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
# Note: Ensure 'Traci.sumocfg' exists in the same folder
Sumo_config = [
    'sumo-gui',
    '-c', 'net2.sumocfg',
    '--step-length', '0.1',  # Using 0.1 for better performance
    '--delay', '100'
]

# Step 5: Open connection
traci.start(Sumo_config)

# Step 6: Define Variables & Constants
V_CONST = 13.89  # Constant speed (m/s)
D = 272.16  # Distance between junctions (m)
T_ACCEL = 10.0  # Time assumed for acceleration (s)
DETECTOR_ID = "E2J1"  # Replace with your E2 detector ID from the .det.xml file

# Dictionary to track {vehicleID: start_waiting_time}
waiting_tracker = {}


# Step 7: Define Functions
def calculate_reach_time(wait_time):
    """Applies the green wave formulas based on wait time."""
    if wait_time < 5:
        # Case 1: Car is already flowing
        t_reach = D / V_CONST
        print(f">>> Result: FLOWING. T_reach: {t_reach:.2f}s")
    else:
        # Case 2: Car stopped, needs acceleration time
        # Formula: T_accel + ((D - (V_const * T_accel)/2) / V_const)
        t_reach = T_ACCEL + ((D - (V_CONST * T_ACCEL) / 2) / V_CONST)
        print(f">>> Result: STOPPED ({wait_time:.1f}s). T_reach: {t_reach:.2f}s")
    return t_reach


# Step 8: Simulation Loop
while traci.simulation.getMinExpectedNumber() > 0:
    traci.simulationStep()
    current_time = traci.simulation.getTime()

    # Get all vehicles on the 150m E2 detector
    vehs_on_det = traci.lanearea.getLastStepVehicleIDs(DETECTOR_ID)

    if vehs_on_det:
        # We only care about the first vehicle (index 0)
        lead_veh = vehs_on_det[0]
        speed = traci.vehicle.getSpeed(lead_veh)

        # Logic for standing still
        if speed < 0.1:
            if lead_veh not in waiting_tracker:
                waiting_tracker[lead_veh] = current_time

            # Optional: Print ongoing wait for debugging
            current_wait = current_time - waiting_tracker[lead_veh]
            # print(f"Lead Vehicle {lead_veh} waiting: {current_wait:.1f}s")

        # Logic for when the vehicle starts moving OR passes the stop line
        else:
            if lead_veh in waiting_tracker:
                # Car was waiting but just started moving
                total_wait = current_time - waiting_tracker[lead_veh]

                # Trigger the Green Wave calculation
                arrival_time = calculate_reach_time(total_wait)

                # IMPORTANT: Remove from tracker so we don't calculate every step
                del waiting_tracker[lead_veh]

    # Cleanup: Remove vehicles from tracker if they leave the detector
    # (prevents memory leak if a vehicle teleports or is removed)
    active_ids = traci.vehicle.getIDList()
    for vid in list(waiting_tracker.keys()):
        if vid not in active_ids:
            del waiting_tracker[vid]

# Step 9: Close connection
traci.close()