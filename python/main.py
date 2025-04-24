import sys
import glob
import serial
import pyvjoy
import pyautogui
pyautogui.PAUSE = 0.0
import tkinter as tk
from tkinter import ttk, messagebox

# Inicializa o vJoy (Device ID 1)
j = pyvjoy.VJoyDevice(1)

def map_range(valor, in_min, in_max, out_min, out_max):
    return int((valor - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

def move_stick_x(axis, value):
    """Move o eixo X do vJoy com base nos dados recebidos."""
    if axis == 0:
        vjoy_val = map_range(value, -127, 127, 0, 32768)
        j.set_axis(pyvjoy.HID_USAGE_X, vjoy_val)
        print(f"Eixo X: {value} -> vJoy: {vjoy_val}")
    elif axis == 1:
        pyautogui.press('u')
        print(f"upshift detected")
    elif axis == 2:
        pyautogui.press('d')
        print(f"downshift detected")
    elif axis == 3:
        pyautogui.press('m')
        print(f"ovtk detected")
    elif axis == 4:
        pyautogui.press('f')
        print(f"drs detected")
    

def controle(ser):
    """Loop principal que lê dados da porta serial."""
    while True:
        sync_byte = ser.read(size=1)
        if not sync_byte:
            continue
        if sync_byte[0] == 0xFF:
            data = ser.read(size=3)
            if len(data) < 3:
                continue
            print(data)
            axis, value = parse_data(data)
            move_stick_x(axis, value)

def serial_ports():
    """Retorna uma lista das portas seriais disponíveis na máquina."""
    ports = []
    if sys.platform.startswith('win'):
        for i in range(1, 256):
            port = f'COM{i}'
            try:
                s = serial.Serial(port)
                s.close()
                ports.append(port)
            except (OSError, serial.SerialException):
                pass
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Plataforma não suportada.')
    return ports

def parse_data(data):
    axis = data[0]
    value = int.from_bytes(data[1:3], byteorder='big', signed=True)
    return axis, value

def conectar_porta(port_name, root, botao_conectar, status_label, mudar_cor_circulo):
    if not port_name:
        messagebox.showwarning("Aviso", "Selecione uma porta serial.")
        return

    try:
        ser = serial.Serial(port_name, 115200, timeout=1)
        status_label.config(text=f"Conectado em {port_name}", foreground="green")
        mudar_cor_circulo("green")
        botao_conectar.config(text="Conectado")
        root.update()

        controle(ser)

    except KeyboardInterrupt:
        print("Encerrado via KeyboardInterrupt.")
    except Exception as e:
        messagebox.showerror("Erro", f"Erro na conexão: {e}")
        mudar_cor_circulo("red")
    finally:
        ser.close()
        status_label.config(text="Conexão encerrada.", foreground="red")
        mudar_cor_circulo("red")

def criar_janela():
    root = tk.Tk()
    root.title("Controle de Volante F1")
    root.geometry("400x250")
    root.resizable(False, False)

    dark_bg = "#2e2e2e"
    dark_fg = "#ffffff"
    accent_color = "#007acc"
    root.configure(bg=dark_bg)

    style = ttk.Style(root)
    style.theme_use("clam")
    style.configure("TFrame", background=dark_bg)
    style.configure("TLabel", background=dark_bg, foreground=dark_fg, font=("Segoe UI", 11))
    style.configure("TButton", font=("Segoe UI", 10, "bold"), foreground=dark_fg, background="#444444", borderwidth=0)
    style.map("TButton", background=[("active", "#555555")])
    style.configure("Accent.TButton", font=("Segoe UI", 12, "bold"), foreground=dark_fg, background=accent_color, padding=6)
    style.map("Accent.TButton", background=[("active", "#005f9e")])
    style.configure("TCombobox", fieldbackground=dark_bg, background=dark_bg, foreground=dark_fg, padding=4)
    style.map("TCombobox", fieldbackground=[("readonly", dark_bg)])

    frame_principal = ttk.Frame(root, padding="20")
    frame_principal.pack(expand=True, fill="both")

    titulo_label = ttk.Label(frame_principal, text="Controle de Volante F1", font=("Segoe UI", 14, "bold"))
    titulo_label.pack(pady=(0, 10))

    porta_var = tk.StringVar(value="")

    botao_conectar = ttk.Button(
        frame_principal,
        text="Conectar e Iniciar Leitura",
        style="Accent.TButton",
        command=lambda: conectar_porta(porta_var.get(), root, botao_conectar, status_label, mudar_cor_circulo)
    )
    botao_conectar.pack(pady=10)

    footer_frame = tk.Frame(root, bg=dark_bg)
    footer_frame.pack(side="bottom", fill="x", padx=10, pady=(10, 0))

    status_label = tk.Label(footer_frame, text="Aguardando seleção de porta...", font=("Segoe UI", 11),
                            bg=dark_bg, fg=dark_fg)
    status_label.grid(row=0, column=0, sticky="w")

    portas_disponiveis = serial_ports()
    if portas_disponiveis:
        porta_var.set(portas_disponiveis[0])
    port_dropdown = ttk.Combobox(footer_frame, textvariable=porta_var,
                                 values=portas_disponiveis, state="readonly", width=10)
    port_dropdown.grid(row=0, column=1, padx=10)

    circle_canvas = tk.Canvas(footer_frame, width=20, height=20, highlightthickness=0, bg=dark_bg)
    circle_item = circle_canvas.create_oval(2, 2, 18, 18, fill="red", outline="")
    circle_canvas.grid(row=0, column=2, sticky="e")

    footer_frame.columnconfigure(1, weight=1)

    def mudar_cor_circulo(cor):
        circle_canvas.itemconfig(circle_item, fill=cor)

    root.mainloop()

if __name__ == "__main__":
    criar_janela()