import os, sys, traci

if 'SUMO_HOME' in os.environ:
    sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))

SUMO_CFG = [
    'sumo-gui', '-c', 'net3.sumocfg',
    '--step-length', '0.1', '--delay', '50'
]

# ── Состояния сигналов ────────────────────────────────────────
SIDE_GO = "Gr"; SIDE_YW = "yr"
MAIN_GO = "rG"; MAIN_YW = "ry"
MAIN_GO_J3 = "G"; MAIN_YW_J3 = "y"; RED_J3 = "r"

GREEN_STATES = {MAIN_GO, MAIN_GO_J3}   # используется для поиска фазы

# ── Конфигурация светофоров ───────────────────────────────────
TL_CONFIGS = {
    "J1": {
        "cycle": 106,
        "phases": [(50, SIDE_GO), (3, SIDE_YW), (50, MAIN_GO), (3, MAIN_YW)],
        # offset для J1 — статичный старт (секунда включения MAIN_GO от t=0)
        "green_at": 53,   # 50+3 → MAIN_GO начнётся на 53-й секунде цикла
    },
    "J2": {
        "cycle": 90,
        "phases": [(42, SIDE_GO), (3, SIDE_YW), (42, MAIN_GO), (3, MAIN_YW)],
        "green_at": None,  # рассчитывается динамически из зелёной волны
    },
    "J3": {
        "cycle": 94,
        "phases": [(71, RED_J3), (20, MAIN_GO_J3), (3, MAIN_YW_J3)],
        "green_at": None,  # рассчитывается динамически
    },
}

# ── Параметры зелёной волны ───────────────────────────────────
V_CONST = 13.89
D_12    = 272.16   # расстояние J1 → J2
D_23    = 269.84   # расстояние J2 → J3
T_ACCEL = 10.0
DIST_CAR = 7.0
T_DELAY  = 2.0
DET_J1 = "E2J1"; DET_J2 = "E2J2"; DET_J3 = "E2J3"

# ── Helpers ───────────────────────────────────────────────────
def apply_program(tl_id, cfg):
    logic = traci.trafficlight.Logic(
        programID="custom", type=0, currentPhaseIndex=0,
        phases=[traci.trafficlight.Phase(d, s) for d, s in cfg["phases"]]
    )
    traci.trafficlight.setCompleteRedYellowGreenDefinition(tl_id, logic)

def time_to_green(cfg):
    """Сколько секунд от начала цикла до первой зелёной фазы главной дороги."""
    t = 0
    for dur, state in cfg["phases"]:
        if state in GREEN_STATES:
            return t
        t += dur
    return 0

def sync_green_at(tl_id, cfg, target_abs_time):
    """
    Подкручивает светофор так, чтобы MAIN_GO включился ровно в target_abs_time.
    target_abs_time — абсолютная секунда симуляции.
    """
    cycle      = cfg["cycle"]
    t_green    = time_to_green(cfg)          # позиция MAIN_GO внутри цикла
    current_t  = traci.simulation.getTime()

    # Сколько секунд осталось до target_abs_time
    delta = target_abs_time - current_t      # может быть отрицательным — берём по модулю цикла

    # Нам нужен offset такой, что:
    #   (offset + t_green) % cycle == delta % cycle
    # => offset = (delta - t_green) % cycle
    offset = (delta - t_green) % cycle

    elapsed = 0
    for idx, (dur, _) in enumerate(cfg["phases"]):
        if elapsed + dur > offset:
            traci.trafficlight.setPhase(tl_id, idx)
            traci.trafficlight.setPhaseDuration(tl_id, (elapsed + dur) - offset)
            return
        elapsed += dur

# ── Инициализация J1 (статичный offset) ──────────────────────
def init_static(tl_id, cfg):
    """Для J1: просто стартуем с нужной позиции внутри цикла."""
    offset = cfg["green_at"] - time_to_green(cfg)
    elapsed = 0
    for idx, (dur, _) in enumerate(cfg["phases"]):
        if elapsed + dur > offset % cfg["cycle"]:
            traci.trafficlight.setPhase(tl_id, idx)
            traci.trafficlight.setPhaseDuration(tl_id, (elapsed + dur) - offset % cfg["cycle"])
            return
        elapsed += dur

# ── Расчёт зелёной волны ──────────────────────────────────────
last_sync_time = -999  # антидребезг

def calculate_and_sync(veh_id, current_step):
    global last_sync_time
    if current_step - last_sync_time < 20:   # не чаще раза в 20 сек
        return

    N2 = traci.lanearea.getLastStepHaltingNumber(DET_J2)
    N3 = traci.lanearea.getLastStepHaltingNumber(DET_J3)

    d_accel     = (V_CONST * T_ACCEL) / 2
    t_reach_12  = T_ACCEL + (D_12 - N2 * DIST_CAR - d_accel) / V_CONST - N2 * T_DELAY
    tlc_j2_green = current_step + t_reach_12

    t_reach_23  = (D_23 - N3 * DIST_CAR) / V_CONST - N3 * T_DELAY
    tlc_j3_green = tlc_j2_green + t_reach_23

    sync_green_at("J2", TL_CONFIGS["J2"], tlc_j2_green)
    sync_green_at("J3", TL_CONFIGS["J3"], tlc_j3_green)
    last_sync_time = current_step

    print(f"[{current_step:.1f}s] {veh_id}: J2 зелёный → {tlc_j2_green:.1f}s, J3 зелёный → {tlc_j3_green:.1f}s")

# ── Main ──────────────────────────────────────────────────────
def main():
    traci.start(SUMO_CFG)

    for tl_id, cfg in TL_CONFIGS.items():
        apply_program(tl_id, cfg)

    init_static("J1", TL_CONFIGS["J1"])
    # J2 и J3 сначала стартуют с offset=0, потом синхронизируются по волне

    waiting_tracker = {}
    seen_on_det     = set()
    logged_vehicles = set()

    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        t = traci.simulation.getTime()

        vehs = set(traci.lanearea.getLastStepVehicleIDs(DET_J1))

        # Машина покинула детектор без остановки
        for vid in seen_on_det - vehs:
            if vid not in logged_vehicles:
                calculate_and_sync(vid, t)
                logged_vehicles.add(vid)
            waiting_tracker.pop(vid, None)

        seen_on_det = vehs.copy()

        # Машина стояла и тронулась
        for vid in vehs:
            if vid in logged_vehicles:
                continue
            speed = traci.vehicle.getSpeed(vid)
            if speed < 0.1:
                waiting_tracker.setdefault(vid, t)
            elif speed > 0.1 and vid in waiting_tracker:
                calculate_and_sync(vid, t)
                logged_vehicles.add(vid)
                del waiting_tracker[vid]

        # Чистим исчезнувшие машины
        all_vehs = set(traci.vehicle.getIDList())
        for vid in list(waiting_tracker):
            if vid not in all_vehs:
                del waiting_tracker[vid]

    traci.close()

if __name__ == "__main__":
    main()