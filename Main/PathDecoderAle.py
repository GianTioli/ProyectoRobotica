import serial
import time
import csv
import os

# ----------------------------------------------------
# CONFIGURACIÓN
# ----------------------------------------------------
# El CSV debe tener 3 columnas:
# columna 1 -> ángulo stepper 1
# columna 2 -> ángulo stepper 2
# columna 3 -> posición cilindro prismático
CSV_PATH = "trayectoria_robot.csv"

SERIAL_PORT = "COM14"
BAUDRATE = 115200
TIMEOUT = 1


def cargar_trayectoria(csv_path):
    trayectoria = []

    if not os.path.exists(csv_path):
        raise FileNotFoundError(f"No se encontró el archivo CSV: {csv_path}")

    with open(csv_path, newline="") as file:
        reader = csv.reader(file, delimiter=",")

        for row in reader:
            if len(row) < 3:
                continue

            try:
                q1 = float(row[0])
                q2 = float(row[1])
                q3 = float(row[2])
                trayectoria.append((q1, q2, q3))
            except ValueError:
                # Ignora encabezados o filas con texto
                continue

    if not trayectoria:
        raise ValueError("El CSV no contiene puntos válidos con 3 valores numéricos.")

    return trayectoria


def enviar_trayectoria(trayectoria):
    ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=TIMEOUT)
    time.sleep(2)

    try:
        for i, (q1, q2, q3) in enumerate(trayectoria, start=1):
            msg = f"{q1:.3f},{q2:.3f},{q3:.3f}\n"
            ser.write(msg.encode())

            print(f"Punto {i}/{len(trayectoria)} enviado: {msg.strip()}")

            while True:
                line = ser.readline().decode(errors="ignore").strip()

                if line:
                    print("[ARDUINO]:", line)

                if line == "OK":
                    break

    finally:
        ser.close()


if __name__ == "__main__":
    trayectoria = cargar_trayectoria(CSV_PATH)
    print(f"Se cargaron {len(trayectoria)} puntos desde {CSV_PATH}\n")

    enviar_trayectoria(trayectoria)

    print("\nTrayectoria finalizada.")
