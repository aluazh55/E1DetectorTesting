"""
actuator.py — единственное место, где вызываются traci.trafficlight.setPhase()
и setPhaseDuration(). Ни один другой модуль не должен трогать светофоры напрямую.
"""
import traci


class Actuator:
    def set_green(self, junction_id: str, phase_index: int) -> None:
        """Заморозить зелёную фазу (до явного вызова set_yellow)."""
        traci.trafficlight.setPhase(junction_id, phase_index)
        traci.trafficlight.setPhaseDuration(junction_id, 9999)

    def set_yellow(self, junction_id: str, phase_index: int) -> None:
        """Переключить в жёлтую фазу на 3 секунды, затем вернуть нативный цикл."""
        traci.trafficlight.setPhase(junction_id, phase_index)
        traci.trafficlight.setPhaseDuration(junction_id, 3)
