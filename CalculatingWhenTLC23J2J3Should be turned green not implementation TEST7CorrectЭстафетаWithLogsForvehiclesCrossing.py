import os
import sys
import traci

# --- Конфигурация SUMO
# Логика немного поломана в условиях крупных заторов
# Проблема $N$ (Halting Number):
# Код берет количество стоящих машин ($N$) в момент,
# когда первая машина только проехала J1.
# Но пока эта машина доедет до J2,
# с бокового направления (E5) могут приехать новые авто,
# и реальное $N$ увеличится.Парадокс опережения (J2 раньше J1):
# Если между J1 и J2 уже стоит огромная пробка,
# зеленый на J2 действительно нужно включать сильно заранее,
# чтобы «рассосать» затор и дать место пребывающим машинам.
# Старая формула этого не видит,
# так как она считает время «доезда» конкретной машины,
# а не время очистки пути для неё.Статичные константы:
# Использование фиксированного V_CONST ($13.89$ м/с) не учитывает,
# что в плотном потоке скорость падает, и время доезда увеличивается.---
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("Please declare environment variable 'SUMO_HOME'")

Sumo_config = [
    'sumo-gui', '-c', 'net3.sumocfg',
    '--step-length', '0.1', '--delay', '50'
]

# --- Константы (из ваших файлов) ---


# --- Constants ---
V_CONST = 13.89
D_12 = 272.16
D_23 = 269.84
T_ACCEL = 5.0
DIST_CAR = 5.0
T_DELAY = 2.0
SAFETY_GAP = 1.5
Turn_GreenEarlier_for = 1

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

    # N — количество стоящих машин перед светофором
    N2 = traci.lanearea.getLastStepHaltingNumber(DET_J2)
    N3 = traci.lanearea.getLastStepHaltingNumber(DET_J3)
    d_accel = (V_CONST * T_ACCEL) / 2

    if start_at_j1:
        # Путь от J1 до J2
        t_reach_12 = T_ACCEL + (D_12 - (N2 * DIST_CAR) - d_accel) / V_CONST - (N2 * T_DELAY) - Turn_GreenEarlier_for
        # это не расчет за сколько аремени доедет машины, это расчет времени через сколько секунд нужно включить зеленый
        tlc_j2_green = departure_time + t_reach_12

        # Путь от J2 до J3
        t_reach_23 = (D_23 - (N3 * DIST_CAR)) / V_CONST - (N3 * T_DELAY) - Turn_GreenEarlier_for
        ideal_j3_arrival = tlc_j2_green + t_reach_23

        # Учитываем эстафету (не допускаем наслоения потоков)
        actual_j3_green = max(
            ideal_j3_arrival,
            last_j1_origin_j3_time + SAFETY_GAP,
            last_j2_native_j3_time + SAFETY_GAP
        )
        last_j1_origin_j3_time = actual_j3_green
        path_info = f"Full Path | J2 Green: {tlc_j2_green:.2f}s"
    else:
        # Путь от J2 до J3 (для тех, кто заехал с E5)
        t_reach_23 = T_ACCEL + (D_23 - (N3 * DIST_CAR) - d_accel) / V_CONST - (N3 * T_DELAY) - Turn_GreenEarlier_for
        ideal_j3_arrival = departure_time + t_reach_23

        actual_j3_green = max(ideal_j3_arrival, last_j2_native_j3_time + SAFETY_GAP)
        last_j2_native_j3_time = actual_j3_green
        path_info = "Starts at E5/J2"

    adj = " (Adjusted for Traffic)" if actual_j3_green > ideal_j3_arrival else ""

    print(f"\n[Dep. Time: {departure_time:.1f}s] Vehicle: {veh_id} ({path_info})")
    print(f"  >> Ideal J3 Arrival: {ideal_j3_arrival:.2f}s")
    print(f"  >> ACTUAL J3 GREEN:  {actual_j3_green:.2f}s {adj}")

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
print("--- LIVE GREEN WAVE MONITORING STARTED (DUAL-STREAM MODE) ---")

while traci.simulation.getMinExpectedNumber() > 0:
    traci.simulationStep()
    current_time = traci.simulation.getTime()
    active_vehs = traci.vehicle.getIDList()

    for v_id in active_vehs:
        monitor_crossings(v_id, current_time)

    # Детектор J2 (Боковой поток)
    vehs_on_det_j2 = set(traci.lanearea.getLastStepVehicleIDs(DET_J2))
    for v_id in (vehs_on_det_j2 - seen_on_detector_j2):
        if v_id not in logged_vehicles and v_id not in entry_time_tracker_j1:
            entry_time_tracker_j2[v_id] = current_time

    for v_id in (seen_on_detector_j2 - vehs_on_det_j2):
        if v_id in entry_time_tracker_j2 and v_id not in logged_vehicles:
            calculate_green_wave_plan(v_id, current_time, start_at_j1=False)
            entry_time_tracker_j2.pop(v_id, None)
    seen_on_detector_j2 = vehs_on_det_j2.copy()

    # Детектор J1 (Основной поток)
    vehs_on_det_j1 = set(traci.lanearea.getLastStepVehicleIDs(DET_J1))
    for v_id in (vehs_on_det_j1 - seen_on_detector_j1):
        entry_time_tracker_j1[v_id] = current_time

    for v_id in (seen_on_detector_j1 - vehs_on_det_j1):
        if v_id not in logged_vehicles:
            calculate_green_wave_plan(v_id, current_time, start_at_j1=True)
            entry_time_tracker_j1.pop(v_id, None)
    seen_on_detector_j1 = vehs_on_det_j1.copy()

    # Очистка словарей
    for d in [entry_time_tracker_j1, entry_time_tracker_j2, last_lane_tracker]:
        for vid in list(d.keys()):
            if vid not in active_vehs: del d[vid]

traci.close()