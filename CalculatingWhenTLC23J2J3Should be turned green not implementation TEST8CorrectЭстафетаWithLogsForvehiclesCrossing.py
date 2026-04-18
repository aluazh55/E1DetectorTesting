import os
import sys
import traci

# --- Конфигурация SUMO
# именно этот код для J2 работает нормально,
# но дело в том что переключние на J3 происходит так:
# red, green - 1 second, yellow -1 second, red .---
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("Please declare environment variable 'SUMO_HOME'")

Sumo_config = [
    'sumo-gui', '-c', 'net3.sumocfg',
    '--step-length', '0.1', '--delay', '50'
]

# --- Константы ---
V_CONST = 13.89
D_12 = 272.16
D_23 = 269.84
T_ACCEL = 5.0
DIST_CAR = 5.0
T_DELAY = 2.0
SAFETY_GAP = 1.5
Turn_GreenEarlier_for = 1

J2_GREEN_PHASE = 0
J3_GREEN_PHASE = 0

# --- НОВЫЕ ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ ДЛЯ АДАПТИВНОГО УДЕРЖАНИЯ ---
CLEARANCE_BUFFER = 4.0  # Дополнительные секунды для безопасного проезда перекрестка последней машиной
pending_green = {"J2": None, "J3": None}
latest_green_release_time = {"J2": 0.0, "J3": 0.0}
green_locked = {"J2": False, "J3": False}

DET_J1, DET_J2, DET_J3 = "E2J1", "E2J2", "E2J3"

# --- Трекеры состояния ---
entry_time_tracker_j1, seen_on_detector_j1 = {}, set()
entry_time_tracker_j2, seen_on_detector_j2 = {}, set()
last_lane_tracker = {}
logged_vehicles = set()

last_j2_native_j3_time = 0.0
last_j1_origin_j3_time = 0.0


def calculate_green_wave_plan(veh_id, departure_time, start_at_j1=True):
    global last_j2_native_j3_time, last_j1_origin_j3_time
    global pending_green, latest_green_release_time

    N2 = traci.lanearea.getLastStepHaltingNumber(DET_J2)
    N3 = traci.lanearea.getLastStepHaltingNumber(DET_J3)
    d_accel = (V_CONST * T_ACCEL) / 2

    if start_at_j1:
        # Расчет для J2
        t_reach_12 = T_ACCEL + (D_12 - (N2 * DIST_CAR) - d_accel) / V_CONST - (N2 * T_DELAY) - Turn_GreenEarlier_for
        arrival_j2 = departure_time + t_reach_12

        # Обновляем старт зеленого (ориентируемся на ПЕРВУЮ машину)
        if pending_green["J2"] is None:
            pending_green["J2"] = arrival_j2
        else:
            pending_green["J2"] = min(pending_green["J2"], arrival_j2)

        # Обновляем конец зеленого (ориентируемся на ПОСЛЕДНЮЮ машину)
        latest_green_release_time["J2"] = max(latest_green_release_time["J2"], arrival_j2 + CLEARANCE_BUFFER)

        # Расчет для J3
        t_reach_23 = (D_23 - (N3 * DIST_CAR)) / V_CONST - (N3 * T_DELAY) - Turn_GreenEarlier_for
        ideal_j3_arrival = arrival_j2 + t_reach_23

        actual_j3_green = max(
            ideal_j3_arrival,
            last_j1_origin_j3_time + SAFETY_GAP,
            last_j2_native_j3_time + SAFETY_GAP
        )
        last_j1_origin_j3_time = actual_j3_green

        # Планируем J3
        if pending_green["J3"] is None:
            pending_green["J3"] = actual_j3_green
        else:
            pending_green["J3"] = min(pending_green["J3"], actual_j3_green)

        latest_green_release_time["J3"] = max(latest_green_release_time["J3"], actual_j3_green + CLEARANCE_BUFFER)

        path_info = f"Full Path | J2 Sch: {arrival_j2:.2f}s"
    else:
        # Расчет для бокового потока (только J3)
        t_reach_23 = T_ACCEL + (D_23 - (N3 * DIST_CAR) - d_accel) / V_CONST - (N3 * T_DELAY) - Turn_GreenEarlier_for
        ideal_j3_arrival = departure_time + t_reach_23

        actual_j3_green = max(ideal_j3_arrival, last_j2_native_j3_time + SAFETY_GAP)
        last_j2_native_j3_time = actual_j3_green

        if pending_green["J3"] is None:
            pending_green["J3"] = actual_j3_green
        else:
            pending_green["J3"] = min(pending_green["J3"], actual_j3_green)

        latest_green_release_time["J3"] = max(latest_green_release_time["J3"], actual_j3_green + CLEARANCE_BUFFER)

        path_info = "Starts at E5/J2"

    print(f"[Plan] {veh_id} ({path_info}) -> J3 Green Release pushed to: {latest_green_release_time['J3']:.2f}s")
    logged_vehicles.add(veh_id)


def monitor_crossings(veh_id, current_time):
    current_lane = traci.vehicle.getLaneID(veh_id)
    if veh_id in last_lane_tracker:
        prev_lane = last_lane_tracker[veh_id]
        if current_lane != prev_lane:
            if "E0_0" in prev_lane and "E0_0" not in current_lane:
                print(f" >>> [CROSS] {veh_id} passed J1 at {current_time:.2f}s")
            elif "E1_0" in prev_lane and "E1_0" not in current_lane:
                print(f" >>> [CROSS] {veh_id} passed J2 at {current_time:.2f}s")
            elif "E2_0" in prev_lane and "E2_0" not in current_lane:
                print(f" >>> [CROSS] {veh_id} passed J3 at {current_time:.2f}s")
    last_lane_tracker[veh_id] = current_lane


# --- Запуск ---
traci.start(Sumo_config)

while traci.simulation.getMinExpectedNumber() > 0:
    traci.simulationStep()
    current_time = traci.simulation.getTime()
    active_vehs = traci.vehicle.getIDList()

    for v_id in active_vehs:
        monitor_crossings(v_id, current_time)

    # --- 1. Адаптивное Включение Зеленого (Блокировка) ---
    for junc, green_phase in [("J2", J2_GREEN_PHASE), ("J3", J3_GREEN_PHASE)]:
        t_start = pending_green[junc]

        if t_start is not None and current_time >= t_start:
            cur_phase = traci.trafficlight.getPhase(junc)
            if cur_phase != green_phase:
                traci.trafficlight.setPhase(junc, green_phase)

            # Замораживаем фазу
            traci.trafficlight.setPhaseDuration(junc, 9999)
            if not green_locked[junc]:
                print(f"  [TLC] {junc} -> GREEN LOCKED @ {current_time:.2f}s")
                green_locked[junc] = True

            # Сбрасываем pending_green, чтобы не вызывать постоянно,
            # но мы все еще отслеживаем latest_green_release_time
            pending_green[junc] = None

    # --- 2. Адаптивное Выключение Зеленого (Освобождение) ---
    for junc, green_phase in [("J2", J2_GREEN_PHASE), ("J3", J3_GREEN_PHASE)]:
        if green_locked[junc]:
            # Проверяем, проехало ли время релиза последней запланированной машины
            if current_time >= latest_green_release_time[junc]:
                # Переключаем на желтый (считаем, что это зеленая фаза + 1)
                yellow_phase = green_phase + 1
                traci.trafficlight.setPhase(junc, yellow_phase)
                traci.trafficlight.setPhaseDuration(junc, 3)

                print(f"  [TLC] {junc} -> Platoon cleared. Releasing to yellow @ {current_time:.2f}s")

                # Сбрасываем состояния
                green_locked[junc] = False
                latest_green_release_time[junc] = 0.0

    # --- Детектор J2 (Боковой поток) ---
    vehs_on_det_j2 = set(traci.lanearea.getLastStepVehicleIDs(DET_J2))
    for v_id in (vehs_on_det_j2 - seen_on_detector_j2):
        if v_id not in logged_vehicles and v_id not in entry_time_tracker_j1:
            entry_time_tracker_j2[v_id] = current_time
    for v_id in (seen_on_detector_j2 - vehs_on_det_j2):
        if v_id in entry_time_tracker_j2 and v_id not in logged_vehicles:
            calculate_green_wave_plan(v_id, current_time, start_at_j1=False)
            entry_time_tracker_j2.pop(v_id, None)
    seen_on_detector_j2 = vehs_on_det_j2.copy()

    # --- Детектор J1 (Основной поток) ---
    vehs_on_det_j1 = set(traci.lanearea.getLastStepVehicleIDs(DET_J1))
    for v_id in (vehs_on_det_j1 - seen_on_detector_j1):
        entry_time_tracker_j1[v_id] = current_time
    for v_id in (seen_on_detector_j1 - vehs_on_det_j1):
        if v_id not in logged_vehicles:
            calculate_green_wave_plan(v_id, current_time, start_at_j1=True)
            entry_time_tracker_j1.pop(v_id, None)
    seen_on_detector_j1 = vehs_on_det_j1.copy()

    # --- Очистка памяти ---
    for d in [entry_time_tracker_j1, entry_time_tracker_j2, last_lane_tracker]:
        for vid in list(d.keys()):
            if vid not in active_vehs: del d[vid]

traci.close()