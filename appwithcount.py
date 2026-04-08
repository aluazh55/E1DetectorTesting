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
        # Массивы для ID (они нужны внутри для логики удаления),
        # но выводить мы будем их длину (сумму)
        self.between_J1_J2 = []
        self.between_J2_J3 = []

        # Храним ID машин на датчиках с прошлого шага
        self.last_step_vehs = {"E1J1_In": set(), "E1J2_In": set(), "E1J3_In": set()}

    def _get_new_arrivals(self, det_id):
        """Возвращает список ID машин, которые только что наехали на детектор"""
        current_vehs = set(traci.inductionloop.getLastStepVehicleIDs(det_id))
        new_vehs = current_vehs - self.last_step_vehs[det_id]
        self.last_step_vehs[det_id] = current_vehs
        return new_vehs

    def update(self):
        # 1. Машина проехала J1
        for v_id in self._get_new_arrivals("E1J1_In"):
            self.between_J1_J2.append(v_id)

        # 2. Машина проехала J2 (Handover)
        for v_id in self._get_new_arrivals("E1J2_In"):
            if v_id in self.between_J1_J2:
                self.between_J1_J2.remove(v_id)
            self.between_J2_J3.append(v_id)

        # 3. Машина проехала J3
        for v_id in self._get_new_arrivals("E1J3_In"):
            if v_id in self.between_J2_J3:
                self.between_J2_J3.remove(v_id)

        # --- ВЫВОД СУММЫ (КОЛИЧЕСТВА) ---
        count_1 = len(self.between_J1_J2)
        count_2 = len(self.between_J2_J3)

        # Печатаем только если на дороге есть хоть кто-то, чтобы не спамить нулями
        if count_1 > 0 or count_2 > 0:
            print(
                f"t={traci.simulation.getTime():.1f}s | Участок J1->J2: {count_1} шт. | Участок J2->J3: {count_2} шт.")


def run():
    # Замени "net.sumocfg" на свой файл, если нужно
    traci.start(["sumo-gui", "-c", "net.sumocfg"])
    tracker = HandoverTracker()

    print("Система мониторинга запущена...")

    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        tracker.update()

    traci.close()
    print("Симуляция завершена.")


if __name__ == "__main__":
    run()