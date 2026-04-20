"""
traffic_node.py — класс TrafficNode.
Каждый перекрёсток — отдельный объект со своей state machine и планировщиком.
Нет глобальных переменных: всё состояние хранится внутри экземпляра.
"""
from __future__ import annotations

from enum import Enum, auto
from typing import Dict, Optional, Set, TYPE_CHECKING

from config import SAFETY_GAP
from physics import travel_time, green_start_time, release_time

if TYPE_CHECKING:
    from actuator import Actuator


class NodeState(Enum):
    IDLE        = auto()   # нет активного плана
    GREEN_MAIN  = auto()   # главная дорога — зелёный (заморожен)
    GREEN_CROSS = auto()   # поперечная улица — зелёный (заморожен)
    TRANSITION  = auto()   # жёлтый, возврат к нативному циклу


class TrafficNode:
    """
    Параметры конструктора:
        node_id            : ID перекрёстка в SUMO (например "J2")
        phases             : {"MAIN_GREEN": int, "CROSS_GREEN": int, "YELLOW": int}
        dist_from_upstream : расстояние (м) от выхода upstream-детектора до этого узла
    """

    def __init__(
        self,
        node_id: str,
        phases: Dict[str, int],
        dist_from_upstream: float = 0.0,
    ) -> None:
        self.node_id = node_id
        self.phases = phases
        self.dist_from_upstream = dist_from_upstream

        # Downstream-узел задаётся из main.py после создания всех узлов
        self.downstream: Optional[TrafficNode] = None

        # ── State machine ──────────────────────────────────────────────
        self.state: NodeState = NodeState.IDLE

        # ── Планирование зелёного ──────────────────────────────────────
        self.pending_green: Optional[float] = None  # время активации
        self.green_locked: bool = False
        self._release_time: float = 0.0             # время снятия блокировки

        # Последнее запланированное время по типу потока (для SAFETY_GAP)
        self._last_scheduled: Dict[str, float] = {"main": 0.0, "cross": 0.0}

        # Машины, идентифицированные как боковой поток (с поперечной улицы)
        self.side_flow_vehicles: Set[str] = set()

    # ──────────────────────────────────────────────────────────────────
    # Интерфейс планирования
    # ──────────────────────────────────────────────────────────────────

    def plan_from_upstream(
        self,
        current_time: float,
        queue_length: int,
        flow_type: str = "main",
    ) -> None:
        """
        Вызывается DetectorManager напрямую, когда машина покидает зону
        upstream-детектора. Планирует открытие зелёного на ЭТОМ узле.

        current_time : момент выхода машины с upstream-детектора
        queue_length : текущее число стоящих машин на детекторе ЭТОГО узла
        flow_type    : "main" — по магистрали, "cross" — с поперечной улицы
        """
        if self.green_locked:
            # Зелёный уже активен — не перебиваем текущий цикл
            return

        t_travel = travel_time(self.dist_from_upstream)
        arrival  = current_time + t_travel
        start    = green_start_time(arrival, queue_length)

        # Не планируем в прошлом
        start = max(start, current_time)

        # Соблюдаем SAFETY_GAP относительно обоих типов потока
        other_flow = "cross" if flow_type == "main" else "main"
        start = max(
            start,
            self._last_scheduled[flow_type] + SAFETY_GAP,
            self._last_scheduled[other_flow] + SAFETY_GAP,
        )

        # Берём наиболее раннее значение (не затираем более ранний план)
        if self.pending_green is None or start < self.pending_green:
            self.pending_green   = start
            self._release_time   = max(self._release_time, release_time(arrival))
            self._last_scheduled[flow_type] = start
            print(
                f"  [PLAN {self.node_id}] {flow_type:5s} flow "
                f"-> green @ {start:.1f}s  (arrival ~ {arrival:.1f}s)"
            )

    # ──────────────────────────────────────────────────────────────────
    # Шаговое обновление (вызывается каждый тик симуляции)
    # ──────────────────────────────────────────────────────────────────

    def update(self, current_time: float, actuator: Actuator) -> None:
        """Продвигает state machine: активирует или снимает зелёный."""

        # Активация
        if self.pending_green is not None and current_time >= self.pending_green:
            actuator.set_green(self.node_id, self.phases["MAIN_GREEN"])
            self.green_locked  = True
            self.state         = NodeState.GREEN_MAIN
            self.pending_green = None
            print(f"  [TLC] {self.node_id}  GREEN_MAIN locked  @ {current_time:.1f}s")

        # Снятие блокировки
        if (
            self.green_locked
            and self._release_time > 0
            and current_time >= self._release_time
        ):
            actuator.set_yellow(self.node_id, self.phases["YELLOW"])
            self.green_locked  = False
            self._release_time = 0.0
            self.state         = NodeState.TRANSITION
            print(f"  [TLC] {self.node_id}  YELLOW             @ {current_time:.1f}s")
