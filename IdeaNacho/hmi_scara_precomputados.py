import csv
import os
import threading
import time
import tkinter as tk
from tkinter import messagebox

import serial

CSV_DIR = "trayectorias_precalculadas"
SERIAL_PORT = "COM14"
BAUDRATE = 115200
TIMEOUT = 1
SAFE_ORDER = [5, 3, 1, 2, 4, 6]

HOLE_LABELS = {
    1: "1 inferior izquierda",
    2: "2 inferior derecha",
    3: "3 medio izquierda",
    4: "4 medio derecha",
    5: "5 superior izquierda",
    6: "6 superior derecha",
}


def script_dir():
    return os.path.dirname(os.path.abspath(__file__))


def csv_path(nombre):
    return os.path.join(script_dir(), CSV_DIR, nombre)


def cargar_trayectoria(path):
    trayectoria = []

    if not os.path.exists(path):
        raise FileNotFoundError(f"No se encontro el archivo: {path}")

    with open(path, newline="") as file:
        reader = csv.reader(file)

        for row in reader:
            if len(row) < 3:
                continue

            try:
                trayectoria.append((float(row[0]), float(row[1]), float(row[2])))
            except ValueError:
                continue

    if not trayectoria:
        raise ValueError(f"El CSV no tiene puntos validos: {path}")

    return trayectoria


class ScaraHMI:
    def __init__(self, root):
        self.root = root
        self.root.title("HMI SCARA - CSV precomputados")
        self.mode = tk.StringVar(value="todos")
        self.port = tk.StringVar(value=SERIAL_PORT)
        self.holes = {i: tk.BooleanVar(value=False) for i in range(1, 7)}

        self.build_ui()

    def build_ui(self):
        tk.Label(self.root, text="Modo de operacion").grid(row=0, column=0, sticky="w", padx=10, pady=(10, 0))
        tk.Radiobutton(self.root, text="Todos los hoyos", variable=self.mode, value="todos").grid(row=1, column=0, sticky="w", padx=20)
        tk.Radiobutton(self.root, text="Un solo hoyo", variable=self.mode, value="uno").grid(row=2, column=0, sticky="w", padx=20)
        tk.Radiobutton(self.root, text="Subgrupo de hoyos", variable=self.mode, value="subgrupo").grid(row=3, column=0, sticky="w", padx=20)

        tk.Label(self.root, text="Seleccion de hoyos").grid(row=4, column=0, sticky="w", padx=10, pady=(10, 0))
        positions = {5: (5, 0), 6: (5, 1), 3: (6, 0), 4: (6, 1), 1: (7, 0), 2: (7, 1)}
        for hole, (row, col) in positions.items():
            tk.Checkbutton(self.root, text=HOLE_LABELS[hole], variable=self.holes[hole]).grid(row=row, column=col, sticky="w", padx=20)

        tk.Label(self.root, text="Puerto serial").grid(row=8, column=0, sticky="w", padx=10, pady=(10, 0))
        tk.Entry(self.root, textvariable=self.port, width=12).grid(row=8, column=1, sticky="w", padx=10, pady=(10, 0))

        self.start_button = tk.Button(self.root, text="Iniciar proceso", command=self.start)
        self.start_button.grid(row=9, column=0, sticky="w", padx=10, pady=10)

        tk.Button(self.root, text="Salir", command=self.root.destroy).grid(row=9, column=1, sticky="w", padx=10, pady=10)

        self.log_box = tk.Text(self.root, width=78, height=18)
        self.log_box.grid(row=10, column=0, columnspan=2, padx=10, pady=(0, 10))

    def log(self, text):
        self.root.after(0, self._append_log, text)

    def _append_log(self, text):
        self.log_box.insert("end", text + "\n")
        self.log_box.see("end")

    def selected_files(self):
        mode = self.mode.get()

        if mode == "todos":
            return [csv_path("todos.csv")]

        selected = [i for i in range(1, 7) if self.holes[i].get()]

        if mode == "uno" and len(selected) != 1:
            raise ValueError("Seleccione exactamente un hoyo.")

        if mode == "subgrupo" and not selected:
            raise ValueError("Seleccione al menos un hoyo.")

        selected = [i for i in SAFE_ORDER if i in selected]
        return [csv_path(f"pozo_{i}.csv") for i in selected]

    def start(self):
        try:
            files = self.selected_files()
        except Exception as error:
            messagebox.showerror("Seleccion invalida", str(error))
            return

        msg = "Antes de iniciar, verifique que el robot este en Home.\n\nArchivos a enviar:\n"
        msg += "\n".join(os.path.basename(path) for path in files)

        if not messagebox.askokcancel("Confirmar ejecucion", msg):
            return

        self.start_button.config(state="disabled")
        threading.Thread(target=self.send_files, args=(files,), daemon=True).start()

    def send_files(self, files):
        try:
            with serial.Serial(self.port.get(), BAUDRATE, timeout=TIMEOUT) as ser:
                time.sleep(2)

                for path in files:
                    trayectoria = cargar_trayectoria(path)
                    self.log(f"\nEnviando {os.path.basename(path)} ({len(trayectoria)} puntos)")
                    self.send_trajectory(ser, trayectoria)

            self.log("\nProceso finalizado.")

        except Exception as error:
            self.log(f"\nERROR: {error}")
            self.root.after(0, messagebox.showerror, "Error", str(error))

        finally:
            self.root.after(0, self.start_button.config, {"state": "normal"})

    def send_trajectory(self, ser, trayectoria):
        total = len(trayectoria)

        for i, (q1, q2, q3) in enumerate(trayectoria, start=1):
            msg = f"{q1:.3f},{q2:.3f},{q3:.3f}\n"
            ser.write(msg.encode())
            self.log(f"Punto {i}/{total}: {msg.strip()}")

            while True:
                line = ser.readline().decode(errors="ignore").strip()

                if line:
                    self.log(f"[ARDUINO]: {line}")

                if line == "OK":
                    break


if __name__ == "__main__":
    root = tk.Tk()
    app = ScaraHMI(root)
    root.mainloop()
