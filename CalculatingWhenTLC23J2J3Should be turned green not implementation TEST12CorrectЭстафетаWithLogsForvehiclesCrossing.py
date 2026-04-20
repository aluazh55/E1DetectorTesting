import os
import sys
import traci
# --- SUMO Configuration
# Оптимизация команд TLC, разделение боковых потоков, защита от сбоев фаз и отсутствие утечек памяти.---
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("Please declare environment variable 'SUMO_HOME'")

Sumo_config = [
    'sumo-gui', '-c', 'net3.sumocfg',
    '--step-length', '0.1', '--delay', '50'
]

# --- Константы и физика ---
V_CONST = 13.89  # Скорость (м/с)
D_12 = 272.16  # Дистанция J1-J2
D_23 = 269.84  # Дистанция J2-J3
T_ACCEL = 5.0  # Время разгона
T_DELAY = 2.0  # Время реакции (старт следующей машины)
SAFETY_GAP = 1.5  # Интервал безопасности между группами
Turn_GreenEarlier_for = 1.5  # За сколько секунд "до" открывать свет
CLEARANCE_BUFFER = 4.0  # Запас времени, чтобы хвост успел проехать

# --- Индексы фаз (из вашего XML) ---
J2_GREEN_PHASE = 2  # rG
J3_GREEN_PHASE = 1  # G
YELLOW_PHASE = {"J2": 3, "J3": 2}

# --- Глобальные трекеры состояния ---
pending_green = {"J2": None, "J3": None}
latest_green_release_time = {"J2": 0.0, "J3": 0.0}
green_locked = {"J2": False, "J3": False}

# Таймеры синхронизации (уравнение задержки из ATLCS)
last_j2_native_j3_time = 0.0
last_j1_origin_j3_time = 0.0

# Словари для отслеживания машин
entry_time_tracker_j1 = {}
entry_time_tracker_j2 = {}
side_flow_vehicles = set()  # Машины, въехавшие с E5 (J2)
logged_vehicles = set()  # Машины, уже учтенные в плане
seen_on_detector_j1 = set()
seen_on_detector_j2 = set()
last_lane_tracker = {}

DET_J1, DET_J2, DET_J3 = "E2J1", "E2J2", "E2J3"


# --- Функции планирования ---

def calculate_j2_plan(veh_id, entry_j1_time):
    """Планирует открытие J2 для машин, идущих от J1."""
    t_travel = (D_12 / V_CONST) + (T_ACCEL / 2)
    planned_start = entry_j1_time + t_travel - (
                traci.lanearea.getLastStepHaltingNumber(DET_J2) * T_DELAY) - Turn_GreenEarlier_for

    # ФИКС №3: Защита от повторного срабатывания, если светофор уже занят этой группой
    if not green_locked["J2"] and latest_green_release_time["J2"] < planned_start:
        pending_green["J2"] = min(pending_green["J2"], planned_start) if pending_green["J2"] else planned_start
        # Время, когда последняя машина гарантированно покинет перекресток
        latest_green_release_time["J2"] = max(latest_green_release_time["J2"],
                                              entry_j1_time + t_travel + 2.0 + CLEARANCE_BUFFER)
        logged_vehicles.add(veh_id)
        print(f" [PLAN J2] {veh_id} detected @ J1 -> J2 Open @ {planned_start:.1f}s")


def _schedule_j3_relay(veh_id, cross_j2_time):
    """Релейная передача: Планирует J3 на основе ФАКТИЧЕСКОГО времени проезда J2."""
    global last_j1_origin_j3_time, last_j2_native_j3_time
    t_travel = (D_23 / V_CONST)
    planned_start = cross_j2_time + t_travel - (
                traci.lanearea.getLastStepHaltingNumber(DET_J3) * T_DELAY) - Turn_GreenEarlier_for

    # Синхронизация потоков (основного и бокового)
    actual_start = max(planned_start, last_j1_origin_j3_time + SAFETY_GAP, last_j2_native_j3_time + SAFETY_GAP)

    # ФИКС №2: Разделение логики для бокового и основного потока
    if veh_id in side_flow_vehicles:
        last_j2_native_j3_time = actual_start
        side_flow_vehicles.discard(veh_id)  # Очищаем сразу после использования
    else:
        last_j1_origin_j3_time = actual_start

    pending_green["J3"] = min(pending_green["J3"], actual_start) if pending_green["J3"] else actual_start
    latest_green_release_time["J3"] = max(latest_green_release_time["J3"],
                                          cross_j2_time + t_travel + 2.0 + CLEARANCE_BUFFER)
    print(f"  [RELAY J3] {veh_id} cleared J2 @ {cross_j2_time:.1f}s -> J3 Open @ {actual_start:.1f}s")


def monitor_crossings(veh_id, current_time):
    """ФИКС №4: Логирование пересечений с игнорированием внутренних полос."""
    lane = traci.vehicle.getLaneID(veh_id)
    if lane.startswith(":"): return  # Пропускаем внутреннюю часть перекрестка

    if veh_id in last_lane_tracker:
        prev = last_lane_tracker[veh_id]
        if lane != prev:
            if "E0_0" in prev:
                print(f" >>> {veh_id} entered J1-J2 link @ {current_time:.1f}s")
            elif "E1_0" in prev:
                print(f" >>> {veh_id} entered J2-J3 link @ {current_time:.1f}s")
    last_lane_tracker[veh_id] = lane


# --- Запуск TRACI ---
traci.start(Sumo_config)
print("--- СИСТЕМА ATLCS ЗАПУЩЕНА (РЕЖИМ ЭСТАФЕТЫ) ---")

while traci.simulation.getMinExpectedNumber() > 0:
    traci.simulationStep()
    current_time = traci.simulation.getTime()
    active_vehs = traci.vehicle.getIDList()

    # ФИКС №1: Мониторинг вынесен из блока управления светофорами
    for v_id in active_vehs:
        monitor_crossings(v_id, current_time)

    # ФИКС №1: Блок управления TLC (выполняется 1 раз за шаг, а не N раз для каждой машины)
    for junc, g_phase in [("J2", J2_GREEN_PHASE), ("J3", J3_GREEN_PHASE)]:
        # Включение и блокировка (Lock)
        if pending_green[junc] is not None and current_time >= pending_green[junc]:
            traci.trafficlight.setPhase(junc, g_phase)
            traci.trafficlight.setPhaseDuration(junc, 9999)  # Заморозка
            green_locked[junc] = True
            pending_green[junc] = None
            print(f"  [TLC] {junc} LOCKED GREEN @ {current_time:.1f}s")

        # Выключение и разблокировка (Release)
        if green_locked[junc] and current_time >= latest_green_release_time[junc]:
            traci.trafficlight.setPhase(junc, YELLOW_PHASE[junc])  # ФИКС: Явный желтый
            traci.trafficlight.setPhaseDuration(junc, 3)
            green_locked[junc] = False
            latest_green_release_time[junc] = 0.0  # ФИКС: Сброс только при релизе
            print(f"  [TLC] {junc} RELEASED @ {current_time:.1f}s")

    # --- Работа с детекторами ---

    # J1: Вход основного потока
    vehs_j1 = set(traci.lanearea.getLastStepVehicleIDs(DET_J1))
    for v in (vehs_j1 - seen_on_detector_j1):
        entry_time_tracker_j1[v] = current_time
    for v in (seen_on_detector_j1 - vehs_j1):
        if v not in logged_vehicles: calculate_j2_plan(v, current_time)
    seen_on_detector_j1 = vehs_j1

    # J2: Выход на финишную прямую к J3
    vehs_j2 = set(traci.lanearea.getLastStepVehicleIDs(DET_J2))
    for v in (vehs_j2 - seen_on_detector_j2):
        # Если машина на J2 не пришла с J1 — значит это боковой поток (E5)
        if v not in entry_time_tracker_j1 and v not in logged_vehicles:
            side_flow_vehicles.add(v)
        entry_time_tracker_j2[v] = current_time
    for v in (seen_on_detector_j2 - vehs_j2):
        # Машина (любая) покинула детектор J2 -> запускаем эстафету для J3
        _schedule_j3_relay(v, current_time)
        entry_time_tracker_j2.pop(v, None)  # Чистим память сразу
    seen_on_detector_j2 = vehs_j2

    # ФИКС №3: Глобальная очистка памяти (защита от утечек)
    for d in [entry_time_tracker_j1, entry_time_tracker_j2, last_lane_tracker]:
        for vid in list(d.keys()):
            if vid not in active_vehs: del d[vid]

traci.close()