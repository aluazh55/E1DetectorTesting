import os
import sys
import traci

# --- Standard SUMO path check ---
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("Please declare environment variable 'SUMO_HOME'")

class PrecisionIDTracker:
    def __init__(self):
        # Lists to store IDs of cars currently inside each physical road segment
        self.segments = {
            "Segment_1": [], # Between E1J1_In and E1J1_Out
            "Segment_2": [], # Between E1J2_In and E1J2_Out
            "Segment_3": []  # Between E1J3_In and E1J3_Out
        }

        # Track last step to detect new entries only
        self.detector_ids = ["E1J1_In", "E1J1_Out", "E1J2_In", "E1J2_Out", "E1J3_In", "E1J3_Out"]
        self.last_step_vehs = {det: set() for det in self.detector_ids}

    def _get_new_arrivals(self, det_id):
        """Returns IDs of vehicles that just hit the detector this step"""
        current_vehs = set(traci.inductionloop.getLastStepVehicleIDs(det_id))
        new_vehs = current_vehs - self.last_step_vehs[det_id]
        self.last_step_vehs[det_id] = current_vehs
        return new_vehs

    def update(self):
        current_time = traci.simulation.getTime()
        changed = False

        # --- Logic for Segment 1 ---
        for v_id in self._get_new_arrivals("E1J1_In"):
            self.segments["Segment_1"].append(v_id)
            changed = True
        for v_id in self._get_new_arrivals("E1J1_Out"):
            if v_id in self.segments["Segment_1"]:
                self.segments["Segment_1"].remove(v_id)
                changed = True

        # --- Logic for Segment 2 ---
        for v_id in self._get_new_arrivals("E1J2_In"):
            self.segments["Segment_2"].append(v_id)
            changed = True
        for v_id in self._get_new_arrivals("E1J2_Out"):
            if v_id in self.segments["Segment_2"]:
                self.segments["Segment_2"].remove(v_id)
                changed = True

        # --- Logic for Segment 3 ---
        for v_id in self._get_new_arrivals("E1J3_In"):
            self.segments["Segment_3"].append(v_id)
            changed = True
        for v_id in self._get_new_arrivals("E1J3_Out"):
            if v_id in self.segments["Segment_3"]:
                self.segments["Segment_3"].remove(v_id)
                changed = True

        # --- Print IDs and Totals only when a change occurs ---
        if changed:
            print(f"\n--- STEP: {current_time:.1f}s ---")
            for seg, veh_list in self.segments.items():
                count = len(veh_list)
                print(f"{seg}: {count} veh(s) -> {veh_list}")

def run():
    # Ensure your .sumocfg points to the additional-file with your new detectors
    traci.start(["sumo-gui", "-c", "net.sumocfg"])
    tracker = PrecisionIDTracker()

    print("Tracking IDs across In/Out segments...")

    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        tracker.update()

    traci.close()

if __name__ == "__main__":
    run()