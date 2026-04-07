import os
import sys
import traci

# --- Стандартная проверка путей ---
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("Please declare environment variable 'SUMO_HOME'")


class HandoverTracker:
    def __init__(self):
        # Списки (очереди) машин, находящихся между перекрестками
        self.between_J1_J2 = []
        self.between_J2_J3 = []

        # Храним ID машин на датчиках с прошлого шага, чтобы не дублировать
        self.last_step_vehs = {"E1J1": set(), "E1J2": set(), "E1J3": set()}

    def _get_new_arrivals(self, det_id):
        """Возвращает список ID машин, которые только что наехали на детектор"""
        current_vehs = set(traci.inductionloop.getLastStepVehicleIDs(det_id))
        new_vehs = current_vehs - self.last_step_vehs[det_id]
        self.last_step_vehs[det_id] = current_vehs
        return new_vehs

    def update(self):
        # 1. Машина проехала ПЕРВЫЙ детектор (J1)
        for v_id in self._get_new_arrivals("E1J1"):
            self.between_J1_J2.append(v_id)
            print(f">>> [J1] Авто {v_id} выехало. Теперь на пути к J2: {self.between_J1_J2}")

        # 2. Машина проехала ВТОРОЙ детектор (J2)
        for v_id in self._get_new_arrivals("E1J2"):
            if v_id in self.between_J1_J2:
                self.between_J1_J2.remove(v_id)  # Удаляем из первой очереди

            self.between_J2_J3.append(v_id)  # Добавляем во вторую
            print(f">>> [J2] Авто {v_id} проехало. Передано в очередь к J3: {self.between_J2_J3}")

        # 3. Машина проехала ТРЕТИЙ детектор (J3)
        for v_id in self._get_new_arrivals("E1J3"):
            if v_id in self.between_J2_J3:
                self.between_J2_J3.remove(v_id)  # Удаляем из списков совсем
            print(f">>> [J3] Авто {v_id} покинуло систему.")


def run():
    traci.start(["sumo-gui", "-c", "net.sumocfg"])
    tracker = HandoverTracker()

    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        tracker.update()

    traci.close()


if __name__ == "__main__":
    run()