import os
import sys
import math
import traci

# ==============================================================================
# КОНФИГУРАЦИОННЫЙ БЛОК (ВСЕ ПЕРЕМЕННЫЕ ТУТ)
# ==============================================================================
SUMO_CONFIG = "net.sumocfg"
SUMO_CMD = "sumo-gui"

# --- Параметры алгоритма ---
ACCELERATION_TIME = 6.0  # Среднее время разгона (в секундах)
MAIN_GREEN_PHASE = 0  # Индекс зеленой фазы (Main Road)
CAR_SPACE_IN_QUEUE = 7.5  # Метров на одну машину в очереди

# --- Параметры имен детекторов ---
DET_PREFIX = "E1"
DET_OUT_SUFFIX = "_Out"

# --- Данные узлов (ID, Координаты, Скорость потока) ---
JUNCTIONS = [
    {"id": "J1", "x": 0.0, "y": 0.0, "speed": 13.89},
    {"id": "J2", "x": 250.0, "y": 0.0, "speed": 11.11},
    {"id": "J3", "x": 700.0, "y": 50.0, "speed": 13.89}
]

# ==============================================================================
# ЛОГИКА УПРАВЛЕНИЯ
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

    def get_remaining_red(self, tls_id):
        """Возвращает время ожидания на красном. 0 если горит зеленый."""
        state = traci.trafficlight.getRedYellowGreenState(tls_id)
        if 'G' in state or 'g' in state:
            return 0

        next_switch = traci.trafficlight.getNextSwitch(tls_id)
        return next_switch - traci.simulation.getTime()

    def run_step(self):
        curr_time = traci.simulation.getTime()

        for i in range(len(self.nodes) - 1):
            j_start = self.nodes[i]
            j_end = self.nodes[i + 1]

            det_name = f"{DET_PREFIX}{j_start['id']}{DET_OUT_SUFFIX}"

            # Логика детекции новых машин на выезде из J1
            current_vehs = set(traci.inductionloop.getLastStepVehicleIDs(det_name))
            new_arrivals = current_vehs - self.history.get(det_name, set())
            self.history[det_name] = current_vehs

            if new_arrivals:
                # 1. Считаем чистую дистанцию и время в пути
                dist = math.dist((j_start['x'], j_start['y']), (j_end['x'], j_end['y']))
                travel_time = dist / j_end['speed']

                # 2. Получаем время ожидания на текущем светофоре
                red_wait = self.get_remaining_red(j_start['id'])

                # 3. ИТОГОВАЯ ФОРМУЛА С УЧЕТОМ РАЗГОНА
                # Если машина стояла на красном, добавляем ACCELERATION_TIME.
                # Если ехала на зеленый (red_wait=0), разгон не добавляем.
                startup_delay = ACCELERATION_TIME if red_wait > 0 else 0

                target_time = curr_time + red_wait + travel_time + startup_delay

                self.queue.append({"time": target_time, "tls": j_end['id']})

                print(f"[*] Platoon from {j_start['id']} to {j_end['id']}:")
                print(f"    Wait: {red_wait:.1f}s | Travel: {travel_time:.1f}s | Accel: {startup_delay}s")
                print(f"    Scheduled Green at: {target_time:.1f}s")

        # Исполнение задач
        for task in self.queue[:]:
            if curr_time >= task["time"]:
                traci.trafficlight.setPhase(task["tls"], MAIN_GREEN_PHASE)
                print(f"[!!!] {task['tls']} SWITCHED TO GREEN (Time: {curr_time}s)")
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