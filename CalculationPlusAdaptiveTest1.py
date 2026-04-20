import os
import sys
import traci

# --- SUMO Configuration ---
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("Please declare environment variable 'SUMO_HOME'")

Sumo_config = [
    'sumo-gui', '-c', 'net3.sumocfg',
    '--step-length', '0.1', '--delay', '50'
]

# --- Параметры физики и эстафеты ---
V_CONST, D_12, D_23 = 13.89, 272.16, 269.84
T_ACCEL, T_DELAY = 5.0, 2.0
Turn_GreenEarlier_for, CLEARANCE_BUFFER = 1.5, 4.0
SAFETY_GAP = 1.5

# --- Ограничения безопасности (Restrictions) ---
MIN_GREEN = 10.0
YELLOW_DURATION = 3.0
ALL_RED_DURATION = 2.0
SIM_STEP = 0.1

# --- Фазы (XML) ---
PHASES = {
    "J1": {"GREEN": 0, "YELLOW": 1, "RED": 2},  # Пример: 0-G, 1-y, 2-r
    "J2": {"GREEN": 2, "YELLOW": 3, "RED": 0},
    "J3": {"GREEN": 1, "YELLOW": 2, "RED": 0}
}


class State:
    IDLE = "IDLE"  # Машин нет, ждем (для J1)
    STATIC = "STATIC"  # Обычный режим (для J2, J3)
    LOCKING = "LOCKING"  # Переход: Желтый -> Все Красные
    LOCKED = "LOCKED"  # Удержание зеленого (Зеленая волна)
    RELEASING = "RELEASING"  # Переход: Желтый -> Возврат в базу


# --- Универсальный контроллер с ограничениями ---
class AdaptiveRelayController:
    def __init__(self, junc_id, phases):
        self.id = junc_id
        self.phases = phases
        self.state = State.IDLE if junc_id == "J1" else State.STATIC
        self.timer = 0.0
        self.pending_wave_start = None
        self.release_time = 0.0
        self.lock_start_time = 0.0

    def request_wave(self, start_at, end_at):
        """Для J2 и J3: Запрос на включение в зеленую волну."""
        if self.pending_wave_start is None or start_at < self.pending_wave_start:
            self.pending_wave_start = start_at
        self.release_time = max(self.release_time, end_at)

    def update(self, current_time, demand_detected=False):
        # --- ЛОГИКА ДЛЯ J1 (ADAPTIVE SOURCE) ---
        if self.id == "J1":
            if self.state == State.IDLE and demand_detected:
                self._start_transition(State.LOCKING)  # Начинаем переход к зеленому
            elif self.state == State.LOCKED and not demand_detected and (
                    current_time - self.lock_start_time) >= MIN_GREEN:
                self._start_transition(State.RELEASING)

        # --- ЛОГИКА ДЛЯ J2/J3 (RELAY FOLLOWERS) ---
        else:
            if self.state == State.STATIC and self.pending_wave_start and current_time >= self.pending_wave_start:
                self._start_transition(State.LOCKING)

        # --- МАШИНА СОСТОЯНИЙ (RESTRICTIONS) ---
        self._run_fsm(current_time)

    def _run_fsm(self, t):
        if self.state == State.LOCKING:
            self.timer = round(self.timer + SIM_STEP, 2)
            # Сначала включаем красный для всех (безопасность), потом зеленый
            traci.trafficlight.setPhase(self.id, self.phases["RED"])
            if self.timer >= ALL_RED_DURATION:
                self._lock(t)

        elif self.state == State.LOCKED:
            # Для J2/J3 держим до release_time. Для J1 держит логика demand.
            if self.id != "J1":
                if t >= self.release_time and (t - self.lock_start_time) >= MIN_GREEN:
                    self._start_transition(State.RELEASING)

        elif self.state == State.RELEASING:
            self.timer = round(self.timer + SIM_STEP, 2)
            traci.trafficlight.setPhase(self.id, self.phases["YELLOW"])
            if self.timer >= YELLOW_DURATION:
                self.state = State.IDLE if self.id == "J1" else State.STATIC
                self.release_time = 0.0
                traci.trafficlight.setPhaseDuration(self.id, -1)  # Вернуть управление SUMO
                print(f"  [TLC {self.id}] RESTRICTION COMPLETED @ {t:.1f}s")

    def _start_transition(self, next_state):
        self.state = next_state
        self.timer = 0.0

    def _lock(self, t):
        self.state = State.LOCKED
        self.lock_start_time = t
        self.pending_wave_start = None
        traci.trafficlight.setPhase(self.id, self.phases["GREEN"])
        traci.trafficlight.setPhaseDuration(self.id, 9999)
        print(f"  [TLC {self.id}] GREEN ACTIVATED (Adaptive/Wave) @ {t:.1f}s")


# --- Инициализация ---
ctrl_j1 = AdaptiveRelayController("J1", PHASES["J1"])
ctrl_j2 = AdaptiveRelayController("J2", PHASES["J2"])
ctrl_j3 = AdaptiveRelayController("J3", PHASES["J3"])

controllers = [ctrl_j1, ctrl_j2, ctrl_j3]
side_flow_vehs = set()
logged_vehs = set()
entry_time_j1 = {}
seen_j1, seen_j2 = set(), set()


# --- Планирование ---
def relay_to_j2(v_id, t_entry):
    n2 = traci.lanearea.getLastStepHaltingNumber("E2J2")
    t_travel = (D_12 / V_CONST) + (T_ACCEL / 2)
    start = t_entry + t_travel - (n2 * T_DELAY) - Turn_GreenEarlier_for
    end = t_entry + t_travel + 2.0 + CLEARANCE_BUFFER
    ctrl_j2.request_wave(start, end)
    logged_vehs.add(v_id)


def relay_to_j3(v_id, t_cross):
    global last_j1_origin_j3_time, last_j2_native_j3_time
    n3 = traci.lanearea.getLastStepHaltingNumber("E2J3")
    t_travel = (D_23 / V_CONST)
    start = t_cross + t_travel - (n3 * T_DELAY) - Turn_GreenEarlier_for

    actual_start = max(start, last_j1_origin_j3_time + SAFETY_GAP, last_j2_native_j3_time + SAFETY_GAP)
    if v_id in side_flow_vehs:
        last_j2_native_j3_time = actual_start
        side_flow_vehs.discard(v_id)
    else:
        last_j1_origin_j3_time = actual_start

    end = t_cross + t_travel + 2.0 + CLEARANCE_BUFFER
    ctrl_j3.request_wave(actual_start, end)


# --- Simulation Loop ---
traci.start(['sumo-gui', '-c', 'net3.sumocfg', '--step-length', str(SIM_STEP)])

last_j1_origin_j3_time = 0.0
last_j2_native_j3_time = 0.0

while traci.simulation.getMinExpectedNumber() > 0:
    traci.simulationStep()
    t = traci.simulation.getTime()
    active = traci.vehicle.getIDList()

    # 1. Адаптивное обнаружение для J1
    demand_j1 = traci.lanearea.getLastStepVehicleNumber("E2J1") > 0

    # 2. Обновление всех контроллеров с учетом их ролей
    ctrl_j1.update(t, demand_detected=demand_j1)
    ctrl_j2.update(t)
    ctrl_j3.update(t)

    # 3. Релейные триггеры (Детекторы)
    v_j1 = set(traci.lanearea.getLastStepVehicleIDs("E2J1"))
    for v in (v_j1 - seen_j1): entry_time_j1[v] = t

    # Когда J1 открыл зеленый и машина ПОКИНУЛА детектор J1 -> планируем J2
    for v in (seen_j1 - v_j1):
        if v not in logged_vehs: relay_to_j2(v, t)
    seen_j1 = v_j1

    v_j2 = set(traci.lanearea.getLastStepVehicleIDs("E2J2"))
    for v in (v_j2 - seen_j2):
        if v not in entry_time_j1 and v not in logged_vehs: side_flow_vehs.add(v)
    for v in (seen_j2 - v_j2):
        relay_to_j3(v, t)
    seen_j2 = v_j2

    # Cleanup
    for vid in list(entry_time_j1.keys()):
        if vid not in active: del entry_time_j1[vid]

traci.close()