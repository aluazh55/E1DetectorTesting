import os
import sys
import math
import traci

# --- 1. ПРОВЕРКА ПУТЕЙ (Чтобы скрипт нашел SUMO) ---
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("Please declare environment variable 'SUMO_HOME'")

# ==========================================
# 2. ОПРЕДЕЛЕНИЕ КООРДИНАТ (Ручной ввод)
# ==========================================
# Впишите сюда актуальные X и Y из NetEdit (Inspect Tool)
junctions_data = [
    {"id": "J1", "x": 50.61,   "y": -0.29},
    {"id": "J2", "x": 322.77, "y": 1.12},
    {"id": "J3", "x": 592.45, "y": 10.28}
]

def run_sumo_and_calculate(data):
    """
    Запускает SUMO и рассчитывает расстояния между узлами.
    """
    # Запуск SUMO-GUI (замените 'net.sumocfg' на ваше имя файла)
    # Если GUI не нужен, замените 'sumo-gui' на 'sumo'
    traci.start(["sumo-gui", "-c", "net.sumocfg"])

    print("\n" + "="*65)
    print(f"{'Connection':<15} | {'Coordinates (x, y)':<25} | {'Distance'}")
    print("-" * 65)

    total_path_distance = 0

    for i in range(len(data) - 1):
        j1 = data[i]
        j2 = data[i+1]

        p1 = (j1["x"], j1["y"])
        p2 = (j2["x"], j2["y"])

        # Расчет Евклидова расстояния
        distance = math.dist(p1, p2)
        total_path_distance += distance

        # Вывод в терминал
        print(f"{j1['id']} -> {j2['id']:<8} | {str(p1):<12} to {str(p2):<11} | {distance:.2f} meters")

    print("-" * 65)
    print(f"TOTAL NETWORK LENGTH: {total_path_distance:.2f} meters")
    print("="*65 + "\n")

    # Симуляция должна сделать хотя бы один шаг, чтобы TraCI заработал полноценно
    step = 0
    while step < 10:
        traci.simulationStep()
        step += 1

    traci.close()

if __name__ == "__main__":
    # Убедитесь, что ваш файл .sumocfg лежит в той же папке, что и этот скрипт
    try:
        run_sumo_and_calculate(junctions_data)
    except Exception as e:
        print(f"Ошибка: {e}")