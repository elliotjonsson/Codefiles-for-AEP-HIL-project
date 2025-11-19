import socket
import time
import pygame

# ==============================
# KONFIGURATION
# ==============================
ROBOT_IP = "192.168.149.1"  # Ändra om din robot har annan IP
ROBOT_PORT = 5005           # Måste matcha servern på roboten
SEND_HZ = 50                # Hur ofta vi skickar (50 ggr/sek)

# Dessa index kan behöva justeras beroende på hur G29 mappas
STEERING_AXIS = 0   # oftast ratt
THROTTLE_AXIS = 2   # gaspedal
BRAKE_AXIS = 3      # bromspedal


def init_joystick():
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        raise RuntimeError("Ingen joystick hittad! Är G29 inkopplad?")

    js = pygame.joystick.Joystick(0)
    js.init()
    print(f"Använder joystick: {js.get_name()}")
    return js


def map_inputs(joystick):
    """
    Läser ratt + pedaler och mappar till (v, omega):
    v     = framåt-hastighet (0 till 1)
    omega = svänghastighet (-1 vänster till 1 höger)
    """
    pygame.event.pump()  # uppdatera joystick-event

    steering_raw = joystick.get_axis(STEERING_AXIS)  # -1 (vänster) till 1 (höger)
    throttle_raw = joystick.get_axis(THROTTLE_AXIS)  # oftast 1 (släppt) till -1 (full gas)
    brake_raw = joystick.get_axis(BRAKE_AXIS)        # liknande för broms

    # Mappa ratt direkt till omega
    omega = float(steering_raw)

    # Mappa throttle: 1 (släppt) -> 0,  -1 (full gas) -> 1
    v = (1.0 - throttle_raw) / 2.0   # ger 0 till 1

    # Om du vill ha broms som "back" kan du lägga till logik här
    # t.ex. om brake_raw < -0.5: v = - (1.0 - brake_raw)/2

    return v, omega


def connect_to_robot():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    print(f"Försöker ansluta till roboten på {ROBOT_IP}:{ROBOT_PORT} ...")
    sock.connect((ROBOT_IP, ROBOT_PORT))
    print("Ansluten till roboten! 🎉")
    return sock


def main():
    joystick = init_joystick()
    sock = None

    delay = 1.0 / SEND_HZ

    while True:
        # Försök se till att vi har en uppkopplad socket
        if sock is None:
            try:
                sock = connect_to_robot()
            except Exception as e:
                print(f"Kunde inte ansluta: {e}. Försöker igen om 2 sek...")
                time.sleep(2)
                continue

        # Läs in ratt/pedaler
        v, omega = map_inputs(joystick)

        # Skapa meddelande som text: "v,omega\n"
        msg = f"{v:.2f},{omega:.2f}\n"
        try:
            sock.sendall(msg.encode("utf-8"))
        except Exception as e:
            print(f"Förlorade anslutningen: {e}")
            sock.close()
            sock = None
            continue

        # Debug-utskrift (kan kommenteras bort om det blir jobbigt)
        print(f"Skickar: v={v:.2f}, omega={omega:.2f}")

        time.sleep(delay)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Avslutar klient.")
