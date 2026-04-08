import os
import sys
import math
import traci

# ==============================================================================
# КОНФИГУРАЦИОННЫЙ БЛОК
# ==============================================================================
SUMO_CONFIG = "net.sumocfg"
SUMO_CMD = "sumo-gui"

ACCELERATION_TIME = 10.0  # Будет добавлено, если была остановка
MAIN_GREEN_PHASE = 0
DET_PREFIX = "E1"
DET_OUT_SUFFIX = "_Out"

JUNCTIONS = [
    {"id": "J1", "x": 50.61, "y": -0.29, "speed": 13.89},
    {"id": "J2", "x": 322.77, "y": 1.12, "speed": 13.89},
    {"id": "J3", "x": 592.45, "y": 10.28, "speed": 13.89}
]

# ==============================================================================
# ЛОГИКА
# ==============================================================================

if 'SUMO_HOME' in os.environ:
    sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))
else:
    sys.exit("Declare SUMO_HOME")


class AdvancedGreenWave:
    def __init__(self, nodes):
        self.nodes = nodes
        self.history = {}
        self.queue = []

    def run_step(self):
        curr_time = traci.simulation.getTime()

        for i in range(len(self.nodes) - 1):
            j_start = self.nodes[i]
            j_end = self.nodes[i + 1]

            det_name = f"{DET_PREFIX}{j_start['id']}{DET_OUT_SUFFIX}"

            try:
                current_vehs = set(traci.inductionloop.getLastStepVehicleIDs(det_name))
            except traci.exceptions.TraCIException:
                continue

            new_arrivals = current_vehs - self.history.get(det_name, set())
            self.history[det_name] = current_vehs

            for v_id in new_arrivals:
                try:
                    # 1. Считаем дистанцию
                    dist = math.dist((j_start['x'], j_start['y']), (j_end['x'], j_end['y']))
                    travel_time = dist / j_end['speed']

                    # 2. Проверка накопленного времени ожидания
                    # Используем getAccumulatedWaitingTime, оно более стабильно для "истории" остановки
                    wait_time = traci.vehicle.getAccumulatedWaitingTime(v_id)

                    startup_delay = ACCELERATION_TIME if wait_time > 0.1 else 0

                    target_time = curr_time + travel_time + startup_delay

                    self.queue.append({"time": target_time, "tls": j_end['id']})

                    print(f"[*] Vehicle {v_id} passed {j_start['id']}:")
                    print(
                        f"    History Wait: {wait_time:.1f}s | Travel: {travel_time:.1f}s | Accel Bonus: {startup_delay}s")
                    print(f"    Scheduled Green at {j_end['id']}: {target_time:.1f}s")

                except traci.exceptions.TraCIException:
                    # Если машина исчезла прямо во время расчетов
                    print(f"[!] Vehicle {v_id} vanished, skipping calculation.")
                    continue

        # Исполнение очереди
        for task in self.queue[:]:
            if curr_time >= task["time"]:
                try:
                    traci.trafficlight.setPhase(task["tls"], MAIN_GREEN_PHASE)
                    print(f"[!!!] {task['tls']} SWITCHED TO GREEN (Time: {curr_time}s)")
                    self.queue.remove(task)
                except traci.exceptions.TraCIException:
                    self.queue.remove(task)


def main():
    traci.start([SUMO_CMD, "-c", SUMO_CONFIG])
    logic = AdvancedGreenWave(JUNCTIONS)
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        logic.run_step()
    traci.close()


if __name__ == "__main__":
    main()