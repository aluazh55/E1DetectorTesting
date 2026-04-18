import os
import sys
import traci

# --- Настройка SUMO
# Сorrect time of arrival time for all vehicles
# [Dep. Time: 53.1s] Vehicle: t_0 (Full Path | J2 Green: 67.68s)
#   >> Ideal J3 Arrival: 87.11s
#   >> ACTUAL J3 GREEN:  87.11s
#  [Dep. Time: 62.1s] Vehicle: t_3 (Starts at E1/J2)
#   >> Ideal J3 Arrival: 84.03s
#   >> ACTUAL J3 GREEN:  91.61s (Adjusted for Traffic)
#   ---


if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("Please declare environment variable 'SUMO_HOME'")

# Запуск GUI
Sumo_config = [
    'sumo-gui', '-c', 'net3.sumocfg',
    '--step-length', '0.1', '--delay', '50'
]
traci.start(Sumo_config)

# --- Константы ---
V_CONST = 13.89  # Целевая скорость (50 км/ч)
D_12 = 272.16  # Дистанция J1 -> J2
D_23 = 269.84  # Дистанция J2 -> J3
T_ACCEL = 5.0  # Время разгона (как в вашем последнем примере)
DIST_CAR = 7.0  # Место, занимаемое машиной в очереди
T_DELAY = 2.0  # Задержка старта каждой следующей машины в очереди
SAFETY_GAP = 1.5  # Минимальный интервал между проездами разных машин

DET_J1 = "E2J1"
DET_J2 = "E2J2"
DET_J3 = "E2J3"

# --- Состояние ---
waiting_tracker_j1 = {}
entry_time_tracker_j1 = {}
seen_on_detector_j1 = set()

waiting_tracker_j2 = {}
entry_time_tracker_j2 = {}
seen_on_detector_j2 = set()

logged_vehicles = set()
log_file = open("simulation_log.txt", "w")

# ГЛОБАЛЬНЫЙ ГРАФИК ОСВОБОЖДЕНИЯ J3
last_scheduled_j3_time = 0.0


def calculate_green_wave_plan(veh_id, departure_time, start_at_j1=True):
    global last_scheduled_j3_time

    N2 = traci.lanearea.getLastStepHaltingNumber(DET_J2)
    N3 = traci.lanearea.getLastStepHaltingNumber(DET_J3)
    d_accel = (V_CONST * T_ACCEL) / 2  # Расстояние за время разгона

    if start_at_j1:
        # Расчет для тех, кто едет с самого начала (t_0, t_1, t_2)
        # 1. Время прибытия на J2 (с учетом очереди N2)
        t_reach_12 = T_ACCEL + (D_12 - (N2 * DIST_CAR) - d_accel) / V_CONST - (N2 * T_DELAY)
        tlc_j2_green = departure_time + t_reach_12

        # 2. Идеальное время прибытия на J3
        t_reach_23 = (D_23 - (N3 * DIST_CAR)) / V_CONST - (N3 * T_DELAY)
        ideal_j3_arrival = tlc_j2_green + t_reach_23

        # КОРРЕКЦИЯ: t_0 не может проехать J3 раньше, чем стоящая перед ней t_3
        actual_j3_green = max(ideal_j3_arrival, last_scheduled_j3_time + SAFETY_GAP)

        path_info = f"Full Path | J2 Green: {tlc_j2_green:.2f}s"
    else:
        # Расчет для тех, кто уже стоит на E1/J2 (t_3, t_4, t_5)
        # Им нужно время на разгон до J3
        t_reach_23 = T_ACCEL + (D_23 - (N3 * DIST_CAR) - d_accel) / V_CONST - (N3 * T_DELAY)
        ideal_j3_arrival = departure_time + t_reach_23

        # Корректируем по предыдущей машине в этой же очереди
        actual_j3_green = max(ideal_j3_arrival, last_scheduled_j3_time + SAFETY_GAP)
        path_info = "Starts at E1/J2"

    # Обновляем глобальное время «занятости» перекрестка J3
    last_scheduled_j3_time = actual_j3_green

    adjustment_note = "(Adjusted for Traffic)" if actual_j3_green > ideal_j3_arrival else ""

    log_entry = (
        f"\n[Dep. Time: {departure_time:.1f}s] Vehicle: {veh_id} ({path_info})\n"
        f"  >> Ideal J3 Arrival: {ideal_j3_arrival:.2f}s\n"
        f"  >> ACTUAL J3 GREEN:  {actual_j3_green:.2f}s {adjustment_note}\n"
    )
    return log_entry


def log_event(veh_id, departure_time, start_at_j1=True):
    entry = calculate_green_wave_plan(veh_id, departure_time, start_at_j1)
    print(entry, flush=True)
    log_file.write(entry + "\n")
    logged_vehicles.add(veh_id)


# --- Основной цикл ---
print("--- LIVE GREEN WAVE MONITORING STARTED (RELAY MODE) ---")

while traci.simulation.getMinExpectedNumber() > 0:
    traci.simulationStep()
    current_time = traci.simulation.getTime()

    # Сначала проверяем J2 (тех, кто ВПЕРЕДИ), чтобы они первыми забронировали время
    vehs_on_det_j2 = set(traci.lanearea.getLastStepVehicleIDs(DET_J2))
    for veh_id in (vehs_on_det_j2 - seen_on_detector_j2):
        # Если машина не приехала с J1, значит она новая на E1
        if veh_id not in logged_vehicles and veh_id not in entry_time_tracker_j1:
            entry_time_tracker_j2[veh_id] = current_time

    # Логика J2: Уход с детектора или старт после остановки
    for veh_id in (seen_on_detector_j2 - vehs_on_det_j2):
        if veh_id in entry_time_tracker_j2 and veh_id not in logged_vehicles:
            log_event(veh_id, entry_time_tracker_j2.pop(veh_id), start_at_j1=False)

    for veh_id in vehs_on_det_j2:
        if veh_id in logged_vehicles or veh_id in entry_time_tracker_j1: continue
        speed = traci.vehicle.getSpeed(veh_id)
        if speed < 0.1:
            waiting_tracker_j2.setdefault(veh_id, current_time)
        elif speed > 0.1 and veh_id in waiting_tracker_j2:
            log_event(veh_id, current_time, start_at_j1=False)
            waiting_tracker_j2.pop(veh_id)
    seen_on_detector_j2 = vehs_on_det_j2.copy()

    # Теперь проверяем J1 (тех, кто СЗАДИ)
    vehs_on_det_j1 = set(traci.lanearea.getLastStepVehicleIDs(DET_J1))
    for veh_id in (vehs_on_det_j1 - seen_on_detector_j1):
        entry_time_tracker_j1[veh_id] = current_time

    for veh_id in (seen_on_detector_j1 - vehs_on_det_j1):
        if veh_id not in logged_vehicles:
            log_event(veh_id, entry_time_tracker_j1.get(veh_id, current_time), start_at_j1=True)
        entry_time_tracker_j1.pop(veh_id, None)

    for veh_id in vehs_on_det_j1:
        if veh_id in logged_vehicles: continue
        speed = traci.vehicle.getSpeed(veh_id)
        if speed < 0.1:
            waiting_tracker_j1.setdefault(veh_id, current_time)
        elif speed > 0.1 and veh_id in waiting_tracker_j1:
            log_event(veh_id, current_time, start_at_j1=True)
            waiting_tracker_j1.pop(veh_id)
    seen_on_detector_j1 = vehs_on_det_j1.copy()

    # Очистка исчезнувших машин
    all_vehs = set(traci.vehicle.getIDList())
    for d in [waiting_tracker_j1, entry_time_tracker_j1, waiting_tracker_j2, entry_time_tracker_j2]:
        for vid in list(d.keys()):
            if vid not in all_vehs: del d[vid]

log_file.close()
traci.close()
print("\n--- SIMULATION FINISHED ---")