"""
main.py — точка входа.
Собирает компоненты, запускает TraCI, выполняет основной цикл.

Порядок в каждом тике:
  1. detector_mgr.step()  → определяет входы/выходы, вызывает plan_from_upstream()
  2. node.update()        → активирует или снимает зелёный через Actuator
  3. detector_mgr.cleanup() → удаляет данные уехавших машин
"""
import traci

from config import ACTIVE_NODES, DISTANCES, DOWNSTREAM, PHASES, SUMO_CONFIG
from actuator import Actuator
from detector_manager import DetectorManager
from traffic_node import TrafficNode


def build_network() -> dict:
    """
    Создаёт TrafficNode для каждого активного узла и прописывает
    downstream-ссылки согласно конфигу DOWNSTREAM.

    Чтобы добавить 3 новых перекрёстка:
      1. В config.py добавьте "J4","J5","J6" в ACTIVE_NODES
      2. Раскомментируйте детекторы в QUEUE_DETECTORS
      — больше ничего менять не нужно.
    """
    # Предвычисляем дистанцию для каждого downstream-узла
    dist_from_upstream = {
        down: dist
        for (up, down), dist in DISTANCES.items()
    }

    nodes: dict = {}
    for jid in ACTIVE_NODES:
        nodes[jid] = TrafficNode(
            node_id=jid,
            phases=PHASES[jid],
            dist_from_upstream=dist_from_upstream.get(jid, 0.0),
        )

    # Прописываем downstream-ссылки (только среди активных узлов)
    for up_id, down_id in DOWNSTREAM.items():
        if up_id in nodes and down_id in nodes:
            nodes[up_id].downstream = nodes[down_id]
            print(f"  [NET] {up_id} -> {down_id}  ({dist_from_upstream.get(down_id, 0):.1f} m)")

    return nodes


def main() -> None:
    traci.start(SUMO_CONFIG)
    print("=" * 50)
    print("  ATLCS STARTED  (relay mode, fixed topology)")
    print("=" * 50)

    actuator    = Actuator()
    nodes       = build_network()
    detector_mgr = DetectorManager(nodes)

    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        t           = traci.simulation.getTime()
        active_vehs = set(traci.vehicle.getIDList())

        # 1. Детекторы: обнаружить входы/выходы, запустить эстафету
        detector_mgr.step(t)

        # 2. State machine: активировать/снять зелёный
        for node in nodes.values():
            node.update(t, actuator)

        # 3. Очистка памяти от уехавших машин
        detector_mgr.cleanup(active_vehs)

    traci.close()
    print("--- SIMULATION ENDED ---")


if __name__ == "__main__":
    main()
