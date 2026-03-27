from inputs import devices
print("Mandos detectados por el script:")
for device in devices.gamepads:
    print(f"- {device.name}")