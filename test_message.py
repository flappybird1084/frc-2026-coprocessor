import socket

# --- CONFIGURATION ---
UDP_IP = "10.102.52.2"  # RoboRIO IP
UDP_PORT = 5800  # port at which roborio is listening


def main():
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    except PermissionError as exc:
        print(
            "Error: UDP socket creation failed due to permissions. "
            "Cannot send messages."
        )
        print(f"Details: {exc}")
        return

    print(f"Sending UDP messages to {UDP_IP}:{UDP_PORT}")
    print("Type a message and press Enter. Ctrl+C to exit.")

    while True:
        try:
            message = input("> ")
        except (EOFError, KeyboardInterrupt):
            print("\nExiting.")
            break

        if not message:
            continue

        try:
            sock.sendto(message.encode(), (UDP_IP, UDP_PORT))
        except Exception as exc:
            print(f"Network Error: {exc}")


if __name__ == "__main__":
    main()
