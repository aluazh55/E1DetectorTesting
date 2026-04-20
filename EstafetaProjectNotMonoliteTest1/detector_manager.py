"""
detector_manager.py — мониторинг lane-area детекторов.

Ответственности:
  - Читать traci.lanearea (только чтение, без управления светофорами)
  - Определять тип потока: "main" (с магистрали) или "cross" (с поперечной)
  - При выходе машины с детектора — напрямую вызывать plan_from_upstream()
    у downstream-узла (без диспетчера, без очереди событий)

Метод определения бокового потока (_passed_through):
  Когда машина покидает детектор узла X, она добавляется в _passed_through[X].
  Когда та же машина входит в детектор узла Y (downstream от X):
    - если она есть в _passed_through[X] → main flow (пришла с магистрали)
    - если нет → cross flow (въехала с поперечной улицы между X и Y)
"""
from __future__ import annotations

from typing import Dict, Optional, Set

import traci

from config import DOWNSTREAM, QUEUE_DETECTORS
from traffic_node import TrafficNode


class DetectorManager:
    def __init__(self, nodes: Dict[str, TrafficNode]) -> None:
        self.nodes = nodes

        # Кэшируем обратный маппинг для быстрого поиска upstream
        self._upstream_of: Dict[str, Optional[str]] = {jid: None for jid in nodes}
        for up, down in DOWNSTREAM.items():
            if down in self._upstream_of:
                self._upstream_of[down] = up

        # Машины, находящиеся на детекторе в данный момент
        self._seen: Dict[str, Set[str]] = {jid: set() for jid in nodes}

        # Машины, вышедшие с детектора данного узла (кандидаты "main flow")
        # Очищаются, как только машина подтверждает прибытие на downstream
        self._passed_through: Dict[str, Set[str]] = {jid: set() for jid in nodes}

    # ──────────────────────────────────────────────────────────────────

    def step(self, current_time: float) -> None:
        """Вызывать каждый шаг симуляции."""
        for jid, node in self.nodes.items():
            det_id = QUEUE_DETECTORS.get(jid)
            if det_id is None:
                continue

            try:
                current_vehs: Set[str] = set(traci.lanearea.getLastStepVehicleIDs(det_id))
            except traci.exceptions.TraCIException:
                continue

            prev_vehs    = self._seen[jid]
            upstream_jid = self._upstream_of.get(jid)

            # ── Машины, вошедшие в зону детектора ─────────────────────
            for v in current_vehs - prev_vehs:
                if upstream_jid is not None:
                    if v in self._passed_through.get(upstream_jid, set()):
                        # Пришла с магистрали — подтверждаем, убираем из кэша
                        self._passed_through[upstream_jid].discard(v)
                    else:
                        # Не проходила через upstream → боковой поток
                        node.side_flow_vehicles.add(v)

            # ── Машины, покинувшие зону детектора → эстафета ──────────
            for v in prev_vehs - current_vehs:
                # Фиксируем прохождение через этот узел
                self._passed_through[jid].add(v)

                downstream_node = node.downstream
                if downstream_node is not None:
                    queue_len = self._read_queue(downstream_node.node_id)
                    flow_type = "cross" if v in node.side_flow_vehicles else "main"
                    node.side_flow_vehicles.discard(v)
                    # Прямой вызов без диспетчера
                    downstream_node.plan_from_upstream(current_time, queue_len, flow_type)

            self._seen[jid] = current_vehs

    # ──────────────────────────────────────────────────────────────────

    def cleanup(self, active_vehs: Set[str]) -> None:
        """Убирает данные о машинах, покинувших симуляцию (защита от утечек)."""
        for jid, node in self.nodes.items():
            self._seen[jid]           &= active_vehs
            self._passed_through[jid] &= active_vehs
            node.side_flow_vehicles   &= active_vehs

    # ──────────────────────────────────────────────────────────────────

    def _read_queue(self, jid: str) -> int:
        """Читает число стоящих машин на детекторе данного узла."""
        det_id = QUEUE_DETECTORS.get(jid)
        if det_id is None:
            return 0
        try:
            return traci.lanearea.getLastStepHaltingNumber(det_id)
        except traci.exceptions.TraCIException:
            return 0
