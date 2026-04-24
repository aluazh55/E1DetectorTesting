import os, sys, collections
import traci

if 'SUMO_HOME' in os.environ:
    sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))
else:
    sys.exit("Please declare environment variable 'SUMO_HOME'")

SUMO_CMD = ['sumo-gui', '-c', 'net3.sumocfg', '--step-length', '0.1', '--delay', '50']

# ══════════════════════════════════════════════
#  SCATS PARAMETERS
# ══════════════════════════════════════════════
STEP            = 0.1
CYCLE           = 140.0         # единый цикл для всех перекрёстков (с)
YELLOW_DUR      = 3.0
MIN_GREEN       = 10.0

V_FREE          = 13.89         # м/с
D_12            = 272.16        # J1 → J2 (м)
D_23            = 269.84        # J2 → J3 (м)
T_12            = D_12 / V_FREE # ≈ 19.6 с
T_23            = D_23 / V_FREE # ≈ 19.4 с

EMA_ALPHA       = 0.3           # 0 < α ≤ 1, меньше = медленнее реакция

SPLIT_STEP_S    = 5.0           # макс. перекладка за раз (с)
SPLIT_PERIOD    = 2             # каждые N циклов — Fast Loop

OFFSET_STEP_S   = 2.0           # шаг сдвига смещения (с)
OFFSET_DEADBAND = 3.0           # зона нечувствительности (с)
OFFSET_PERIOD   = 30            # каждые N циклов — Slow Loop

PLATOON_WINDOW  = 20.0          # окно для расчёта центра масс (с)

AVAIL_2PH = CYCLE - 2 * YELLOW_DUR   # 134.0 с — доступное время для 2-фазных
AVAIL_1PH = CYCLE - YELLOW_DUR       # 137.0 с — для J3

# ══ Состояния фаз (из net3.net.xml) ══════════
# J1: linkIndex 0=E4(бок.), 1=E0(осн.)  → "Gr"=бок.зелёный, "rG"=осн.зелёный
# J2: linkIndex 0=E5(бок.), 1=E1(осн.)  → аналогично
# J3: единственный поток E2→E3
PHASE_STATES = {
    "J1": ["Gr", "yr", "rG", "ry"],
    "J2": ["gr", "yr", "rG", "ry"],
    "J3": ["r",  "G",  "y"],
}

# ══ Начальные сплиты (масштаб из XML до 140 с) ══
# J1 XML: side=50, main=70 → 134 с: side≈60, main≈74
# J2 XML: side=64, main=42 → 134 с: side≈78, main≈56
J1_G_SIDE_0 = 60.0;  J1_G_MAIN_0 = 74.0   # 60+74+6 = 140 ✓
J2_G_SIDE_0 = 78.0;  J2_G_MAIN_0 = 56.0   # 78+56+6 = 140 ✓
J3_G_MAIN_0 = 20.0;  J3_G_RED_0  = AVAIL_1PH - J3_G_MAIN_0  # 117.0

# ══ Расчёт смещений для зелёной волны ══════
# Основной зелёный J1 стартует в t = offset_J1 + g_side + yellow = 0 + 60 + 3 = 63 с
# Платон прибывает в J2: 63 + T_12 = 82.6 с → offset_J2 = 82.6 - (78+3) = 1.6 с
# Платон прибывает в J3: 82.6 + T_23 = 102 с → offset_J3 = (102 - 117) % 140 = 125 с
J1_MAIN_T   = 0.0 + J1_G_SIDE_0 + YELLOW_DUR              # = 63.0 с
J2_OFFSET_0 = round((J1_MAIN_T + T_12) - (J2_G_SIDE_0 + YELLOW_DUR), 1)  # ≈  1.6
J3_OFFSET_0 = round(((J1_MAIN_T + T_12 + T_23) - J3_G_RED_0) % CYCLE, 1) # ≈ 125.0

# ══ Состояние перекрёстков ══════════════════
jstate = {
    "J1": dict(
        offset    = 0.0,
        g_side    = J1_G_SIDE_0,   # phase 0 "Gr"  — бок. зелёный (E4→E1)
        g_main    = J1_G_MAIN_0,   # phase 2 "rG"  — осн. зелёный (E0→E1)
        ema_main  = 0.0,
        ema_side  = 0.0,
        cycle_cnt = 0,
        arrivals  = collections.deque(),
        det_area  = "E2J1",        # laneAreaDetector на осн. подъезде
        det_in    = "E1J1_In",     # inductionLoop
        lane_side = "E4_0",        # боковая полоса для подсчёта очереди
        has_side  = True,
    ),
    "J2": dict(
        offset    = J2_OFFSET_0,
        g_side    = J2_G_SIDE_0,
        g_main    = J2_G_MAIN_0,
        ema_main  = 0.0,
        ema_side  = 0.0,
        cycle_cnt = 0,
        arrivals  = collections.deque(),
        det_area  = "E2J2",
        det_in    = "E1J2_In",
        lane_side = "E5_0",
        has_side  = True,
    ),
    "J3": dict(
        offset    = J3_OFFSET_0,
        g_main    = J3_G_MAIN_0,   # phase 1 "G"
        ema_main  = 0.0,
        cycle_cnt = 0,
        arrivals  = collections.deque(),
        det_area  = "E2J3",
        det_in    = "E1J3_In",
        has_side  = False,
    ),
}

# ══════════════════════════════════════════════
#  ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ
# ══════════════════════════════════════════════

def make_durations(junc: str) -> list:
    """Возвращает длительности фаз, сумма = CYCLE."""
    s = jstate[junc]
    if junc == "J3":
        r = AVAIL_1PH - s["g_main"]
        return [max(r, MIN_GREEN), s["g_main"], YELLOW_DUR]  # [red, green, yellow]
    return [s["g_side"], YELLOW_DUR, s["g_main"], YELLOW_DUR]  # [side_g, y, main_g, y]


def push_plan(junc: str):
    """Загружает план (сплиты) в SUMO. SUMO перезапускается с фазы 0."""
    durations = make_durations(junc)
    phases = [traci.trafficlight.Phase(d, st)
              for d, st in zip(durations, PHASE_STATES[junc])]
    logic = traci.trafficlight.Logic("scats", 0, 0, phases)
    traci.trafficlight.setCompleteRedYellowGreenDefinition(junc, logic)


def init_phases():
    """
    Устанавливает каждый перекрёсток в правильную фазу при t=0
    на основе смещения — реализует начальную синхронизацию зелёной волны.
    """
    for junc, s in jstate.items():
        push_plan(junc)
        durations  = make_durations(junc)
        t_local_0  = (-s["offset"]) % CYCLE      # положение в цикле при t=0
        cumulative = 0.0
        for i, d in enumerate(durations):
            if t_local_0 < cumulative + d:
                remaining = (cumulative + d) - t_local_0
                traci.trafficlight.setPhase(junc, i)
                traci.trafficlight.setPhaseDuration(junc, remaining)
                print(f"  [INIT] {junc}: phase {i} ({PHASE_STATES[junc][i]}), "
                      f"осталось {remaining:.1f}с | offset={s['offset']:.1f}с")
                break
            cumulative += d


def update_ema(junc: str):
    """EMA-сглаживание очередей на основном и боковом подъездах."""
    s = jstate[junc]
    q_main = traci.lanearea.getLastStepHaltingNumber(s["det_area"])
    s["ema_main"] = EMA_ALPHA * q_main + (1 - EMA_ALPHA) * s["ema_main"]
    if s["has_side"]:
        q_side = traci.lane.getLastStepHaltingNumber(s["lane_side"])
        s["ema_side"] = EMA_ALPHA * q_side + (1 - EMA_ALPHA) * s["ema_side"]


def collect_arrivals(junc: str, t: float):
    """Фиксирует время прохода машин через индукционную петлю."""
    s = jstate[junc]
    for _ in traci.inductionloop.getLastStepVehicleIDs(s["det_in"]):
        s["arrivals"].append(t)
    # Удаляем устаревшие записи
    cutoff = t - PLATOON_WINDOW * 3
    while s["arrivals"] and s["arrivals"][0] < cutoff:
        s["arrivals"].popleft()


def get_com(junc: str, t: float):
    """
    Центр масс (среднее) времён прибытия машин за последние PLATOON_WINDOW секунд.
    Ключевое отличие от реакции на первую машину — учитывает весь платон.
    """
    s      = jstate[junc]
    recent = [a for a in s["arrivals"] if a >= t - PLATOON_WINDOW]
    return sum(recent) / len(recent) if recent else None


def adapt_split(junc: str):
    """
    FAST LOOP: перераспределяет время внутри цикла между фазами.
    Сумма зелёных фаз остаётся неизменной (консервация).
    """
    s = jstate[junc]
    if not s["has_side"]:
        return
    diff = s["ema_side"] - s["ema_main"]
    if abs(diff) < 1.0:      # незначительный дисбаланс
        return
    transfer = min(SPLIT_STEP_S, abs(diff))
    if diff > 0:             # боковая очередь больше
        actual = min(transfer, s["g_main"] - MIN_GREEN)
        if actual <= 0: return
        s["g_side"] += actual
        s["g_main"] -= actual
        print(f"  [SPLIT↑side] {junc}: g_side={s['g_side']:.0f}с  g_main={s['g_main']:.0f}с")
    else:                    # основная очередь больше
        actual = min(transfer, s["g_side"] - MIN_GREEN)
        if actual <= 0: return
        s["g_main"] += actual
        s["g_side"] -= actual
        print(f"  [SPLIT↑main] {junc}: g_main={s['g_main']:.0f}с  g_side={s['g_side']:.0f}с")


def adapt_offset(junc: str, t: float):
    """
    SLOW LOOP: сдвигает смещение так, чтобы начало зелёного окна совпало
    с центром масс платона. Зона нечувствительности предотвращает «дрожание».
    """
    s   = jstate[junc]
    com = get_com(junc, t)
    if com is None:
        return

    # Момент начала основного зелёного в текущем цикле
    t_local       = (t - s["offset"]) % CYCLE
    t_cycle_start = t - t_local

    if junc == "J3":
        t_green_start = t_cycle_start + (AVAIL_1PH - s["g_main"])  # после красной
    else:
        t_green_start = t_cycle_start + s["g_side"] + YELLOW_DUR   # после бок.зел.+желт.

    # Ошибка: (+) → платон прибыл ПОСЛЕ старта зелёного (offset нужно увеличить)
    #          (-) → платон прибыл ДО старта зелёного (offset нужно уменьшить)
    error = com - t_green_start
    while error >  CYCLE / 2: error -= CYCLE
    while error < -CYCLE / 2: error += CYCLE

    if abs(error) < OFFSET_DEADBAND:
        return   # внутри зоны нечувствительности — не трогать

    delta          = OFFSET_STEP_S * (1 if error > 0 else -1)
    s["offset"]    = (s["offset"] + delta) % CYCLE
    print(f"  [OFFSET] {junc}: offset={s['offset']:.1f}с  "
          f"| ошибка CoM={error:+.1f}с  Δ={delta:+.1f}с")


# ══════════════════════════════════════════════
#  ГЛАВНЫЙ ЦИКЛ
# ══════════════════════════════════════════════
traci.start(SUMO_CMD)
init_phases()

print("\n=== SCATS КОНТРОЛЛЕР ЗЕЛЁНОЙ ВОЛНЫ ===")
print(f"  ЦИКЛ={CYCLE:.0f}с  |  T12={T_12:.1f}с  |  T23={T_23:.1f}с")
print(f"  Смещения: J1=0.0с  J2={J2_OFFSET_0}с  J3={J3_OFFSET_0}с")
print(f"  Осн.зелёный: J1@{J1_MAIN_T:.1f}с → J2@{J2_OFFSET_0+J2_G_SIDE_0+YELLOW_DUR:.1f}с"
      f" → J3@{(J3_OFFSET_0+J3_G_RED_0)%CYCLE:.1f}с\n")

while traci.simulation.getMinExpectedNumber() > 0:
    traci.simulationStep()
    t = traci.simulation.getTime()

    # ── Каждый шаг: EMA + сбор прибытий ────────────────────
    for junc in jstate:
        update_ema(junc)
        collect_arrivals(junc, t)

    # ── Проверка границы цикла (безопасное окно синхронизации) ──
    for junc, s in jstate.items():
        t_local = (t - s["offset"]) % CYCLE

        if t_local < STEP:          # ← t_local < STEP вместо == 0
            s["cycle_cnt"] += 1
            cnt   = s["cycle_cnt"]
            dirty = False

            # Fast Loop: Split
            if cnt % SPLIT_PERIOD == 0:
                adapt_split(junc)
                dirty = True

            # Slow Loop: Offset (изменение вступит в силу на следующей границе)
            if cnt % OFFSET_PERIOD == 0:
                adapt_offset(junc, t)

            # Загружаем обновлённый план в SUMO
            if dirty or cnt == 1:
                push_plan(junc)
                d = make_durations(junc)
                if junc == "J3":
                    print(f"  [C#{cnt:3d}] {junc} | green={d[1]:.0f}с  red={d[0]:.0f}с"
                          f" | offset={s['offset']:.1f}с")
                else:
                    print(f"  [C#{cnt:3d}] {junc} | g_side={d[0]:.0f}с  g_main={d[2]:.0f}с"
                          f" | offset={s['offset']:.1f}с")

traci.close()
print("=== СИМУЛЯЦИЯ ЗАВЕРШЕНА ===")