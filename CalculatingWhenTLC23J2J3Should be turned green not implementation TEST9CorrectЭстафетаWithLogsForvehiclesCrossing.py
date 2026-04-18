import os
import sys
import traci

# --- Конфигурация SUMO
# Работает практически идеально, но включает зеленый на J3 сликшом рано---
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("Please declare environment variable 'SUMO_HOME'")

Sumo_config = [
    'sumo-gui', '-c', 'net3.sumocfg',
    '--step-length', '0.1', '--delay', '50'
]

# --- Константы (параметры сети) ---
V_CONST = 13.89  # м/с
D_12 = 272.16  # J1-J2
D_23 = 269.84  # J2-J3
T_ACCEL = 5.0  # Время разгона
DIST_CAR = 5.0  # Машина + зазор
T_DELAY = 2.0  # Реакция водителей
SAFETY_GAP = 1.5
Turn_GreenEarlier_for = 2.0  # Включаем чуть заранее для рассасывания очереди

# --- ИНДЕКСЫ ФАЗ ИЗ ВАШЕГО XML ---
# J2: 0:gr, 1:yr, 2:rG, 3:ry -> Нам нужен rG (индекс 2) для потока от J1
J2_GREEN_PHASE = 2
# J3: 0:r, 1:G, 2:y -> Нам нужен G (индекс 1)
J3_GREEN_PHASE = 1

# --- Глобальные переменные управления ---
CLEARANCE_BUFFER = 5.0  # Увеличили буфер, чтобы точно успели проехать
pending_green = {"J2": None, "J3": None}
latest_green_release_time = {"J2": 0.0, "J3": 0.0}
green_locked = {"J2": False, "J3": False}

# Трекеры логирования фаз J3
j3_last_phase = None
j3_phase_start_time = 0.0

DET_J1, DET_J2, DET_J3 = "E2J1", "E2J2", "E2J3"

# --- Трекеры состояния машин ---
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
        # --- J2 ---
        # Время включения (с учетом очереди)
        t_start_j2 = T_ACCEL + (D_12 - (N2 * DIST_CAR) - d_accel) / V_CONST - (N2 * T_DELAY) - Turn_GreenEarlier_for
        planned_start_j2 = departure_time + t_start_j2
        # Время физического доезда (Clearance)
        real_arrival_j2 = departure_time + (D_12 / V_CONST)

        pending_green["J2"] = min(pending_green["J2"], planned_start_j2) if pending_green["J2"] else planned_start_j2
        latest_green_release_time["J2"] = max(latest_green_release_time["J2"], real_arrival_j2 + CLEARANCE_BUFFER)

        # --- J3 ---
        t_start_j3 = (D_23 - (N3 * DIST_CAR)) / V_CONST - (N3 * T_DELAY) - Turn_GreenEarlier_for
        planned_start_j3 = planned_start_j2 + t_start_j3
        actual_start_j3 = max(planned_start_j3, last_j1_origin_j3_time + SAFETY_GAP,
                              last_j2_native_j3_time + SAFETY_GAP)
        last_j1_origin_j3_time = actual_start_j3

        real_arrival_j3 = real_arrival_j2 + (D_23 / V_CONST)
        pending_green["J3"] = min(pending_green["J3"], actual_start_j3) if pending_green["J3"] else actual_start_j3
        latest_green_release_time["J3"] = max(latest_green_release_time["J3"], real_arrival_j3 + CLEARANCE_BUFFER)

    else:
        # --- E5 Entry to J3 ---
        t_start_j3 = T_ACCEL + (D_23 - (N3 * DIST_CAR) - d_accel) / V_CONST - (N3 * T_DELAY) - Turn_GreenEarlier_for
        planned_start_j3 = departure_time + t_start_j3
        actual_start_j3 = max(planned_start_j3, last_j2_native_j3_time + SAFETY_GAP)
        last_j2_native_j3_time = actual_start_j3

        real_arrival_j3 = departure_time + (D_23 / V_CONST)
        pending_green["J3"] = min(pending_green["J3"], actual_start_j3) if pending_green["J3"] else actual_start_j3
        latest_green_release_time["J3"] = max(latest_green_release_time["J3"], real_arrival_j3 + CLEARANCE_BUFFER)

    print(f"[PLAN] {veh_id}: J3 Start: {pending_green['J3']:.1f}s, Release: {latest_green_release_time['J3']:.1f}s")
    logged_vehicles.add(veh_id)


# --- Запуск ---
traci.start(Sumo_config)

while traci.simulation.getMinExpectedNumber() > 0:
    traci.simulationStep()
    current_time = traci.simulation.getTime()
    active_vehs = traci.vehicle.getIDList()

    # --- 1. ВКЛЮЧЕНИЕ ---
    for junc, g_phase in [("J2", J2_GREEN_PHASE), ("J3", J3_GREEN_PHASE)]:
        t_trig = pending_green[junc]
        if t_trig is not None and current_time >= t_trig:
            traci.trafficlight.setPhase(junc, g_phase)
            traci.trafficlight.setPhaseDuration(junc, 9999)
            if not green_locked[junc]:
                print(f"  [TLC] {junc} -> LOCKED GREEN (Phase {g_phase}) @ {current_time:.1f}s")
                green_locked[junc] = True
            pending_green[junc] = None

    # --- 2. РЕЛИЗ ---
    for junc, g_phase in [("J2", J2_GREEN_PHASE), ("J3", J3_GREEN_PHASE)]:
        if green_locked[junc]:
            if current_time >= latest_green_release_time[junc]:
                # Желтый всегда идет после зеленого в ваших XML (J2: 2->3(ry), J3: 1->2(y))
                y_phase = g_phase + 1
                traci.trafficlight.setPhase(junc, y_phase)
                traci.trafficlight.setPhaseDuration(junc, 3)
                print(f"  [TLC] {junc} -> RELEASED to yellow (Phase {y_phase}) @ {current_time:.1f}s")
                green_locked[junc] = False
                latest_green_release_time[junc] = 0.0

    # --- 3. ЛОГ ФАЗ J3 ---
    curr_p_j3 = traci.trafficlight.getPhase("J3")
    if curr_p_j3 != j3_last_phase:
        if j3_last_phase is not None:
            dur = current_time - j3_phase_start_time
            print(f"      [LOG J3] Phase {j3_last_phase} lasted {dur:.1f}s")
        j3_last_phase = curr_p_j3
        j3_phase_start_time = current_time

    # --- ДЕТЕКТОРЫ ---
    vehs_on_det_j1 = set(traci.lanearea.getLastStepVehicleIDs(DET_J1))
    for v_id in (vehs_on_det_j1 - seen_on_detector_j1):
        entry_time_tracker_j1[v_id] = current_time
    for v_id in (seen_on_detector_j1 - vehs_on_det_j1):
        if v_id not in logged_vehicles:
            calculate_green_wave_plan(v_id, current_time, start_at_j1=True)
    seen_on_detector_j1 = vehs_on_det_j1.copy()

    vehs_on_det_j2 = set(traci.lanearea.getLastStepVehicleIDs(DET_J2))
    for v_id in (vehs_on_det_j2 - seen_on_detector_j2):
        if v_id not in logged_vehicles and v_id not in entry_time_tracker_j1:
            entry_time_tracker_j2[v_id] = current_time
    for v_id in (seen_on_detector_j2 - vehs_on_det_j2):
        if v_id in entry_time_tracker_j2 and v_id not in logged_vehicles:
            calculate_green_wave_plan(v_id, current_time, start_at_j1=False)
    seen_on_detector_j2 = vehs_on_det_j2.copy()

traci.close()