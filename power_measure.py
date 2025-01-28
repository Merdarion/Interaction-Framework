from datetime import datetime
import subprocess

print("Script Started ... Measuring Power Consumption since Battery was unplugged.\n")

# Capture start time and initial charge and voltage
start = datetime.now()
initial_charge = int(subprocess.getoutput('cat /sys/class/power_supply/BAT0/charge_now'))
initial_voltage = int(subprocess.getoutput('cat /sys/class/power_supply/BAT0/voltage_now'))

abort = 0
while not abort:
    key = input("Type p followed by ENTER to Finish Measurement after experiment is over...")
    if key.lower() == "p":
        abort = 1

# Capture end time and final charge and voltage
end = datetime.now()
final_charge = int(subprocess.getoutput('cat /sys/class/power_supply/BAT0/charge_now'))
final_voltage = int(subprocess.getoutput('cat /sys/class/power_supply/BAT0/voltage_now'))

# Calculate elapsed time in hours
delta_t = end - start
elapsed_time_hours = delta_t.total_seconds() / 3600

# Calculate initial and final power in Watts
initial_power = (initial_charge * initial_voltage) / 1_000_000_000
final_power = (final_charge * final_voltage) / 1_000_000_000

# Estimate average power consumption
average_power = (initial_power + final_power) / 2

# Calculate total energy consumed in Watt-hours
energy_consumed_wh = average_power * elapsed_time_hours

print(f"\nScript took: {elapsed_time_hours * 3600:.2f} seconds")
print(f"Average Power Consumption was: {average_power:.6f} W")
print(f"Total Energy Consumed: {energy_consumed_wh:.6f} Wh")
