import os
import sys
import traci

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("Please declare environment variable 'SUMO_HOME'")


class PrecisionTracker:
    def __init__(self):
        # Храним ID машин, находящихся внутри каждого сегмента
        self.in_segment_1 = []  # Между J1_In и J1_Out
        self.in_segment_2 = []  # Между J2_In и J2_Out
        self.in_segment_3 = []  # Между J3_In и J3_Out

        # Список всех детекторов для отслеживания новых машин
        self.detector_ids = ["E1J1_In", "E1J1_Out", "E1J2_In", "E1J2_Out", "E1J3_In", "E1J3_Out"]
        self.last_step_vehs = {det: set() for det in self.detector_ids}

    def _get_new_arrivals(self, det_id):
        current_vehs = set(traci.inductionloop.getLastStepVehicleIDs(det_id))
        new_vehs = current_vehs - self.last_step_vehs[det_id]
        self.last_step_vehs[det_id] = current_vehs
        return new_vehs

    def update(self):
        current_time = traci.simulation.getTime()

        # --- СЕГМЕНТ 1 (J1) ---
        for v_id in self._get_new_arrivals("E1J1_In"):
            self.in_segment_1.append(v_id)
        for v_id in self._get_new_arrivals("E1J1_Out"):
            if v_id in self.in_segment_1:
                self.in_segment_1.remove(v_id)

        # --- СЕГМЕНТ 2 (J2) ---
        for v_id in self._get_new_arrivals("E1J2_In"):
            self.in_segment_2.append(v_id)
        for v_id in self._get_new_arrivals("E1J2_Out"):
            if v_id in self.in_segment_2:
                self.in_segment_2.remove(v_id)

        # --- СЕГМЕНТ 3 (J3) ---
        for v_id in self._get_new_arrivals("E1J3_In"):
            self.in_segment_3.append(v_id)
        for v_id in self._get_new_arrivals("E1J3_Out"):
            if v_id in self.in_segment_3:
                self.in_segment_3.remove(v_id)

        # Вывод данных (суммы)
        s1, s2, s3 = len(self.in_segment_1), len(self.in_segment_2), len(self.in_segment_3)

        if s1 > 0 or s2 > 0 or s3 > 0:
            print(f"Time: {current_time:.1f}s | "
                  f"J1: {s1} шт. | "
                  f"J2: {s2} шт. | "
                  f"J3: {s3} шт.")


def run():
    traci.start(["sumo-gui", "-c", "net.sumocfg"])
    tracker = PrecisionTracker()

    print("Мониторинг зон запущен...")

    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        tracker.update()

    traci.close()


if __name__ == "__main__":
    run()