import os
import sys
import math
import traci

# ==============================================================================
# КОНФИГУРАЦИОННЫЙ БЛОК
# ==============================================================================
SUMO_CONFIG = "net2.sumocfg"
SUMO_CMD = "sumo-gui"

ACCELERATION_TIME = 10.0

DET_PREFIX = "E1"
DET_OUT_SUFFIX = "_Out"

JUNCTIONS = [
    {"id": "J1", "x": 50.61,  "y": -0.29, "speed": 13.89, "main_green": 2},
    {"id": "J2", "x": 322.77, "y":  1.12, "speed": 13.89, "main_green": 2},
    {"id": "J3", "x": 592.45, "y": 10.28, "speed": 13.89, "main_green": 0},
]

DETECTOR_TO_STOPLINE = {
    "J1": 8.7,
    "J2": 9.5,
    "J3": 9.0,
}

SLOW_SPEED_THRESHOLD = 0.85  # Если скорость < 85% от максимума — машина разгоняется

# ==============================================================================
# ЛОГИКА
# ==============================================================================

if 'SUMO_HOME' in os.environ:
    sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))
else:
    sys.exit("Declare environment variable SUMO_HOME")


def time_until_green(tls_id, current_phase, time_remaining_in_phase, main_green_phase):
    """Суммирует время от текущей фазы до main_green_phase (через все промежуточные)."""
    if current_phase == main_green_phase:
        return 0

    logics = traci.trafficlight.getAllProgramLogics(tls_id)
    phases = logics[0].phases
    n      = len(phases)

    wait  = max(time_remaining_in_phase, 0)
    phase = (current_phase + 1) % n
    while phase != main_green_phase:
        wait += phases[phase].duration
        phase = (phase + 1) % n

    return wait


class AdvancedGreenWave:
    def __init__(self, nodes):
        self.nodes   = nodes
        self.history = {}
        self.queue   = {}  # {tls_id: target_time} — одна задача на светофор

        self.veh_waiting_start = {}
        self.tls_last_phase    = {j['id']: -1 for j in nodes}

    def run_step(self):
        curr_time = traci.simulation.getTime()

        # --- Лог переключений светофоров ---
        for j in self.nodes:
            tls_id     = j['id']
            curr_phase = traci.trafficlight.getPhase(tls_id)
            if curr_phase != self.tls_last_phase[tls_id]:
                print(f"[TLS LOG] {tls_id} → фаза {curr_phase} на {curr_time:.1f}с")
                self.tls_last_phase[tls_id] = curr_phase

        # --- Мониторинг ожидания ---
        for v_id in traci.vehicle.getIDList():
            if traci.vehicle.getSpeed(v_id) < 0.1:
                if v_id not in self.veh_waiting_start:
                    self.veh_waiting_start[v_id] = curr_time
            else:
                if v_id in self.veh_waiting_start:
                    wait_duration = curr_time - self.veh_waiting_start[v_id]
                    if wait_duration > 0.5:
                        print(f"[WAIT LOG] {v_id} простояло {wait_duration:.1f}с")
                    del self.veh_waiting_start[v_id]

        # --- Основная логика ---
        for i in range(len(self.nodes) - 1):
            j_start    = self.nodes[i]
            j_end      = self.nodes[i + 1]
            det_name   = f"{DET_PREFIX}{j_start['id']}{DET_OUT_SUFFIX}"
            main_green = j_start['main_green']

            try:
                current_vehs = set(traci.inductionloop.getLastStepVehicleIDs(det_name))
            except traci.exceptions.TraCIException:
                continue

            new_arrivals = current_vehs - self.history.get(det_name, set())
            self.history[det_name] = current_vehs

            for v_id in new_arrivals:
                print(f"[PASS LOG] {v_id} проехал {j_start['id']} на {curr_time:.1f}с")

                try:
                    # --- Средняя скорость на детекторе (реальная физическая) ---
                    avg_speed = traci.inductionloop.getLastStepMeanSpeed(det_name)

                    # --- Параметры полосы для расчёта travel_time ---
                    lane_id   = traci.vehicle.getLaneID(v_id)
                    max_speed = traci.lane.getMaxSpeed(lane_id)

                    # --- Расчёт stop_delay через скорость на детекторе ---
                    if 0 < avg_speed < (max_speed * SLOW_SPEED_THRESHOLD):
                        # Машина едет медленно — разгоняется после красного
                        speed_status = f"STARTING (avg={avg_speed:.2f} м/с)"

                        # Дополнительно уточняем через фазу: сколько ещё ждать,
                        # если машина ещё не проехала стоп-линию
                        speed            = traci.vehicle.getSpeed(v_id)
                        remaining_dist   = DETECTOR_TO_STOPLINE.get(j_start['id'], 0)
                        time_to_stopline = remaining_dist / speed if speed > 0.1 else 0

                        current_phase  = traci.trafficlight.getPhase(j_start['id'])
                        phase_duration = traci.trafficlight.getPhaseDuration(j_start['id'])
                        time_in_phase  = traci.trafficlight.getSpentDuration(j_start['id'])
                        time_remaining = phase_duration - time_in_phase - time_to_stopline

                        if current_phase != main_green:
                            # Всё ещё красный — ждём смены + разгон
                            wait_for_green = time_until_green(
                                j_start['id'], current_phase, time_remaining, main_green
                            )
                            stop_delay = wait_for_green + ACCELERATION_TIME
                        else:
                            # Уже зелёный, но скорость низкая — только разгон
                            stop_delay = ACCELERATION_TIME

                    elif avg_speed <= 0:
                        # Детектор не зафиксировал скорость (нет данных) — fallback на фазу
                        speed_status = f"NO SPEED DATA (avg={avg_speed:.2f})"

                        speed            = traci.vehicle.getSpeed(v_id)
                        remaining_dist   = DETECTOR_TO_STOPLINE.get(j_start['id'], 0)
                        time_to_stopline = remaining_dist / speed if speed > 0.1 else 0

                        current_phase  = traci.trafficlight.getPhase(j_start['id'])
                        phase_duration = traci.trafficlight.getPhaseDuration(j_start['id'])
                        time_in_phase  = traci.trafficlight.getSpentDuration(j_start['id'])
                        time_remaining = phase_duration - time_in_phase - time_to_stopline

                        if current_phase != main_green:
                            wait_for_green = time_until_green(
                                j_start['id'], current_phase, time_remaining, main_green
                            )
                            stop_delay = wait_for_green + ACCELERATION_TIME
                        else:
                            accumulated_wait = traci.vehicle.getAccumulatedWaitingTime(v_id)
                            stop_delay = ACCELERATION_TIME if accumulated_wait > 0.1 else 0

                    else:
                        # Машина едет на крейсерской скорости — без задержки
                        speed_status = f"CRUISING (avg={avg_speed:.2f} м/с)"
                        stop_delay   = 0

                    dist        = math.dist((j_start['x'], j_start['y']),
                                            (j_end['x'],   j_end['y']))
                    travel_time = dist / j_end['speed']
                    target_time = curr_time + travel_time + stop_delay

                    tls_target = j_end['id']

                    # Дедупликация: оставляем самое раннее время для каждого светофора
                    if tls_target not in self.queue or target_time < self.queue[tls_target]:
                        self.queue[tls_target] = target_time
                        print(f"    {speed_status} | stop_delay={stop_delay:.1f}с "
                              f"travel={travel_time:.1f}с → {tls_target} green @ {target_time:.1f}с")

                except traci.exceptions.TraCIException:
                    print(f"[!] {v_id} исчез, пропускаем.")
                    continue

        # --- Исполнение очереди ---
        for tls_id in list(self.queue):
            if curr_time >= self.queue[tls_id]:
                try:
                    tls_main_green = next(
                        j['main_green'] for j in self.nodes if j['id'] == tls_id
                    )
                    traci.trafficlight.setPhase(tls_id, tls_main_green)
                    print(f"[TLS EVENT] Принудительный ЗЕЛЕНЫЙ на {tls_id} в {curr_time:.1f}с")
                except (traci.exceptions.TraCIException, StopIteration):
                    pass
                del self.queue[tls_id]


def main():
    traci.start([SUMO_CMD, "-c", SUMO_CONFIG])
    logic = AdvancedGreenWave(JUNCTIONS)

    while traci.simulation.getMinExpectedNumber() > 0 or logic.queue:
        traci.simulationStep()
        logic.run_step()

    traci.close()


if __name__ == "__main__":
    main()