import os
import sys
import math
import traci

# ==============================================================================
# КОНФИГУРАЦИОННЫЙ БЛОК
# ==============================================================================
SUMO_CONFIG = "net.sumocfg"
SUMO_CMD = "sumo-gui"

# Параметры имен (Префиксы и Суффиксы)
DET_PREFIX = "E1"
DET_OUT_SUFFIX = "_Out"

# Фазы (измените под ваш net.xml)
# Обычно в SUMO четные фазы - это сигналы, нечетные - желтый/переходный
MAIN_GREEN_PHASE = 0

# Данные узлов
# target_speed: средняя скорость движения К этому узлу
JUNCTIONS = [
    {"id": "J1", "x": 50.61, "y": -0.29, "speed": 13.89},
    {"id": "J2", "x": 322.77, "y": 1.12, "speed": 13.89},
    {"id": "J3", "x": 592.45, "y": 10.28, "speed": 13.89}
]

# ==============================================================================
# ЛОГИКА УПРАВЛЕНИЯ
# ==============================================================================

if 'SUMO_HOME' in os.environ:
    sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))
else:
    sys.exit("Declare SUMO_HOME")


class SmartGreenWave:
    def __init__(self, nodes):
        self.nodes = nodes
        self.history = {}
        self.queue = []

    def get_remaining_red(self, tls_id):
        """
        Возвращает время до конца текущей фазы, если горит красный.
        Если уже горит зеленый, возвращает 0.
        """
        state = traci.trafficlight.getRedYellowGreenState(tls_id)
        # Проверяем, есть ли 'G' или 'g' (зеленый) в строке состояния
        if 'G' in state or 'g' in state:
            return 0

        # traci.trafficlight.getNextSwitch() возвращает абсолютное время симуляции
        next_switch = traci.trafficlight.getNextSwitch(tls_id)
        return next_switch - traci.simulation.getTime()

    def run_step(self):
        curr_time = traci.simulation.getTime()

        for i in range(len(self.nodes) - 1):
            j_start = self.nodes[i]
            j_end = self.nodes[i + 1]

            det_name = f"{DET_PREFIX}{j_start['id']}{DET_OUT_SUFFIX}"

            # Детекция проезда выездного датчика на J1
            current_vehs = set(traci.inductionloop.getLastStepVehicleIDs(det_name))
            new_arrivals = current_vehs - self.history.get(det_name, set())
            self.history[det_name] = current_vehs

            if new_arrivals:
                # 1. Расстояние и время в пути
                dist = math.dist((j_start['x'], j_start['y']), (j_end['x'], j_end['y']))
                travel_time = dist / j_end['speed']

                # 2. Время ожидания на текущем светофоре
                red_time_left = self.get_remaining_red(j_start['id'])

                # 3. Итоговое время включения для следующего светофора
                # Т_вкл = Сейчас + Остаток_Красного + Время_в_пути
                target_time = curr_time + red_time_left + travel_time

                self.queue.append({"time": target_time, "tls": j_end['id']})

                print(f"[*] Platoon at {j_start['id']} detected.")
                print(f"    Wait at J1: {red_time_left:.1f}s | Travel to J2: {travel_time:.1f}s")
                print(f"    J2 will turn GREEN at: {target_time:.1f}s")

        # Исполнение очереди
        for task in self.queue[:]:
            if curr_time >= task["time"]:
                traci.trafficlight.setPhase(task["tls"], MAIN_GREEN_PHASE)
                print(f"[!!!] GREEN ON at {task['tls']} (Time: {curr_time}s)")
                self.queue.remove(task)


def main():
    traci.start([SUMO_CMD, "-c", SUMO_CONFIG])
    logic = SmartGreenWave(JUNCTIONS)

    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        logic.run_step()

    traci.close()


if __name__ == "__main__":
    main()