"""
physics.py — чистые функции расчёта времени.
Никаких вызовов TraCI. Легко тестировать изолированно.
"""
from config import V_CONST, T_ACCEL, T_DELAY, TURN_GREEN_EARLIER, CLEARANCE_BUFFER


def travel_time(distance: float) -> float:
    """
    Оценочное время в пути с учётом разгона.
    distance: расстояние от точки выхода с upstream-детектора до данного узла (м).
    """
    return (distance / V_CONST) + (T_ACCEL / 2)


def green_start_time(arrival_time: float, queue_length: int) -> float:
    """
    Момент открытия зелёного, чтобы машины приехали «на зелёный».
    Смещается назад: на задержку очереди и на TURN_GREEN_EARLIER.
    """
    return arrival_time - (queue_length * T_DELAY) - TURN_GREEN_EARLIER


def release_time(arrival_time: float) -> float:
    """Раннее время, к которому хвост колонны гарантированно освободит перекрёсток."""
    return arrival_time + 2.0 + CLEARANCE_BUFFER
