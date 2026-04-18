import os
import sys
import traci

# --- Настройка среды SUMO ---
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("Please declare environment variable 'SUMO_HOME'")

# Конфигурация запуска (убедитесь, что net3.sumocfg находится в той же папке)
Sumo_config = [
    'sumo-gui', '-c', 'net3.sumocfg',
    '--step-length', '0.1', '--delay', '50'
]
traci.start(Sumo_config)

# --- Константы ---
V_CONST = 13.89  # Целевая скорость (50 км/ч)
D_12 = 272.16  # Дистанция J1 -> J2
D_23 = 269.84  # Дистанция J2 -> J3
T_ACCEL = 5.0  # Время разгона
DIST_CAR = 7.0  # Место, занимаемое одной машиной в очереди (метры)
T_DELAY = 1.0  # Задержка реакции (сек) — уменьшил до 1.0 для реализма

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


def calculate_green_wave_plan(veh_id, departure_time, start_at_j1=True):
    """
    Рассчитывает план 'Зеленой волны' согласно ATLCS.
    """
    # Получаем количество стоящих машин перед перекрестками прямо сейчас
    N2 = traci.lanearea.getLastStepHaltingNumber(DET_J2)
    N3 = traci.lanearea.getLastStepHaltingNumber(DET_J3)

    # Дистанция, проходимая при разгоне: d = (v*t)/2
    d_accel = (V_CONST * T_ACCEL) / 2

    if start_at_j1:
        # --- ПУТЬ J1 -> J2 -> J3 ---

        # 1. Расчет для J2 (машина едет от J1 до хвоста очереди N2)
        dist_to_j2_queue = max(0, D_12 - (N2 * DIST_CAR) - d_accel)
        t_reach_12 = T_ACCEL + (dist_to_j2_queue / V_CONST) + (N2 * T_DELAY)
        tlc_j2_green = departure_time + t_reach_12

        # 2. Расчет для J3 (машина стартует от стоп-линии J2 и едет до стоп-линии J3)
        # Учитываем, что перед J3 стоит N3 машин, которые должны успеть начать движение.
        dist_to_j3_queue = max(0, D_23 - (N3 * DIST_CAR))
        # Время доезда до J3 с учетом того, что N3 машин перед нами должны тронуться
        t_reach_23 = (dist_to_j3_queue / V_CONST) + (N3 * T_DELAY)
        tlc_j3_green = tlc_j2_green + t_reach_23

        log_entry = (
            f"\n[Time: {departure_time:.1f}s] Vehicle: {veh_id} (Full Path)\n"
            f"  J2: N2={N2}. Включить зеленый в {tlc_j2_green:.2f}s\n"
            f"  J3: N3={N3}. Включить зеленый в {tlc_j3_green:.2f}s"
        )
    else:
        # --- ПУТЬ J2 -> J3 (для машин, выехавших с боковой дороги E1) ---
        dist_to_j3_queue = max(0, D_23 - (N3 * DIST_CAR) - d_accel)
        t_reach_23 = T_ACCEL + (dist_to_j3_queue / V_CONST) + (N3 * T_DELAY)
        tlc_j3_green = departure_time + t_reach_23

        log_entry = (
            f"\n[Time: {departure_time:.1f}s] Vehicle: {veh_id} (Starts at J2)\n"
            f"  J3: N3={N3}. Включить зеленый в {tlc_j3_green:.2f}s"
        )

    return log_entry


def log_event(veh_id, departure_time, start_at_j1=True):
    entry = calculate_green_wave_plan(veh_id, departure_time, start_at_j1)
    if entry:
        print(entry, flush=True)
        log_file.write(entry + "\n")
        logged_vehicles.add(veh_id)


# --- Основной цикл симуляции ---
print("--- LIVE GREEN WAVE MONITORING STARTED ---")

try:
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        current_time = traci.simulation.getTime()

        # --- Логика детектора J1 (Сквозной поток) ---
        vehs_on_det_j1 = set(traci.lanearea.getLastStepVehicleIDs(DET_J1))

        # Новые машины на детекторе
        for veh_id in (vehs_on_det_j1 - seen_on_detector_j1):
            if veh_id not in entry_time_tracker_j1:
                entry_time_tracker_j1[veh_id] = current_time

        # Уход с детектора (проезд без остановки)
        for veh_id in (seen_on_detector_j1 - vehs_on_det_j1):
            if veh_id not in logged_vehicles:
                dep_time = entry_time_tracker_j1.get(veh_id, current_time)
                log_event(veh_id, dep_time, start_at_j1=True)
            entry_time_tracker_j1.pop(veh_id, None)

        # Остановка и начало движения на J1 (Stop-and-Go)
        for veh_id in vehs_on_det_j1:
            if veh_id in logged_vehicles: continue
            speed = traci.vehicle.getSpeed(veh_id)
            if speed < 0.1:
                waiting_tracker_j1.setdefault(veh_id, current_time)
            elif speed > 1.0 and veh_id in waiting_tracker_j1:
                log_event(veh_id, current_time, start_at_j1=True)
                waiting_tracker_j1.pop(veh_id)

        seen_on_detector_j1 = vehs_on_det_j1.copy()

        # --- Логика детектора J2 (Только для вливания с E1) ---
        vehs_on_det_j2 = set(traci.lanearea.getLastStepVehicleIDs(DET_J2))

        for veh_id in (vehs_on_det_j2 - seen_on_detector_j2):
            # Если машина не пришла с J1 и еще не залогирована
            if veh_id not in logged_vehicles and veh_id not in entry_time_tracker_j1:
                entry_time_tracker_j2[veh_id] = current_time

        for veh_id in (seen_on_detector_j2 - vehs_on_det_j2):
            if veh_id in entry_time_tracker_j2 and veh_id not in logged_vehicles:
                log_event(veh_id, entry_time_tracker_j2[veh_id], start_at_j1=False)
            entry_time_tracker_j2.pop(veh_id, None)

        seen_on_detector_j2 = vehs_on_det_j2.copy()

        # Очистка данных для исчезнувших машин
        if int(current_time) % 10 == 0:
            all_vehs = set(traci.vehicle.getIDList())
            for d in [waiting_tracker_j1, entry_time_tracker_j1, entry_time_tracker_j2]:
                for vid in list(d.keys()):
                    if vid not in all_vehs:
                        del d[vid]

finally:
    log_file.close()
    traci.close()
    print("\n--- SIMULATION FINISHED ---")