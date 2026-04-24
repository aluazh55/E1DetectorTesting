import traci

SUMO_BINARY = "sumo-gui"
SUMO_CFG    = "net3.sumocfg"

# ── Состояния сигналов (linkIndex 0 = юг E4/E5, linkIndex 1 = запад E0/E1) ──
SIDE_GO = "Gr"   # южное направление зелёный, главная красный
SIDE_YW = "yr"
MAIN_GO = "rG"   # главная зелёный, южное красный
MAIN_YW = "ry"

# J3 — только главная (один link)
MAIN_GO_J3 = "G"
MAIN_YW_J3 = "y"
RED_J3     = "r"

# ── Конфигурация светофоров (зеркало net.xml) ────────────────
# Меняйте только duration — порядок и state совпадают с net.xml
TL_CONFIGS = {
    "J1": {
        "cycle": 106,   # 50+3+50+3
        "phases": [
            (50,  SIDE_GO),
            (3,   SIDE_YW),
            (50,  MAIN_GO),
            (3,   MAIN_YW),
        ],
        "offset": 0,
    },
    "J2": {
        "cycle": 90,   # 450+3+42+3
        "phases": [
            (42, SIDE_GO),
            (3,   SIDE_YW),
            (42,  MAIN_GO),
            (3,   MAIN_YW),
        ],
        "offset": 62,
    },
    "J3": {
        "cycle": 94,   # 710+20+3
        "phases": [
            (71, RED_J3),
            (20,  MAIN_GO_J3),
            (3,   MAIN_YW_J3),
        ],
        "offset": 0,
    },
}

# ── Helpers ───────────────────────────────────────────────────
def apply_program(tl_id, cfg):
    logic_phases = [
        traci.trafficlight.Phase(dur, state)
        for dur, state in cfg["phases"]
    ]
    logic = traci.trafficlight.Logic(
        programID="custom",
        type=0,
        currentPhaseIndex=0,
        phases=logic_phases,
    )
    traci.trafficlight.setCompleteRedYellowGreenDefinition(tl_id, logic)

def apply_offset(tl_id, cfg):
    offset = cfg["offset"] % cfg["cycle"]
    elapsed = 0
    for idx, (dur, _) in enumerate(cfg["phases"]):
        if elapsed + dur > offset:
            remaining = (elapsed + dur) - offset
            traci.trafficlight.setPhase(tl_id, idx)
            traci.trafficlight.setPhaseDuration(tl_id, remaining)
            return
        elapsed += dur

# ── Main ──────────────────────────────────────────────────────
def main():
    traci.start([SUMO_BINARY, "-c", SUMO_CFG, "--no-step-log"])

    for tl_id, cfg in TL_CONFIGS.items():
        apply_program(tl_id, cfg)
        apply_offset(tl_id, cfg)

    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()

    traci.close()

if __name__ == "__main__":
    main()