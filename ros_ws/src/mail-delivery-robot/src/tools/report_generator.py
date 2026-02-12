import os
from jinja2 import Environment, FileSystemLoader
from datetime import datetime
import re

class ReportGenerator:
    def __init__(self):
        self.src_dir = os.path.dirname(os.path.realpath(__file__))
        self.root_dir = os.path.dirname(os.path.join(self.src_dir,"..","..","..", ".."))
        self.log_dir = os.path.join(self.src_dir, "logs")

        self.battery_log_path = os.path.join(self.log_dir, "robot_log_battery.txt")
        self.wall_log_path = os.path.join(self.log_dir, "robot_log_wallFollowing.txt")
        self.time_log_path = os.path.join(self.log_dir, "robot_log_time.txt")

        # Jinja2 setup
        self.env = Environment(loader=FileSystemLoader(self.src_dir))
        self.template = self.env.get_template("template.html")

    def generate_report(self):
        #Parse battery log
        battery_logs = []
        if os.path.exists(self.battery_log_path):
            with open(self.battery_log_path, 'r') as f:
                for line in f:
                    if "[BATTERY]" in line:
                        battery_logs.append(line.strip())
        percentages = []
        voltages = []
        temperatures = []
        for log in battery_logs:
            pct_match = re.search(r"Percentage:\s*([\d.]+)%", log)
            volt_match = re.search(r"Voltage:\s*([\d.]+)V", log)
            temp_match = re.search(r"Temp:\s*([\d.]+)C", log)
            if pct_match:
                percentages.append(float(pct_match.group(1)))
            if volt_match:
                voltages.append(float(volt_match.group(1)))
            if temp_match:
                temperatures.append(float(temp_match.group(1)))

        battery_lost = percentages[0] - percentages[-1]
        #print("Battery lost:", battery_lost)
        final_battery = percentages[-1]
        #print("Final battery:", final_battery)
        average_voltage = sum(voltages)/len(voltages)
        #print("Average voltage:", average_voltage)
        average_temp = sum(temperatures)/len(temperatures)
        #print("Average temperature:", average_temp)

        #parse wall follow log
        wall_follow_logs = []
        if os.path.exists(self.wall_log_path):
            with open(self.wall_log_path, 'r') as f:
                for line in f:
                    if ":WALL_FOLLOW," in line:
                        wall_follow_logs.append(line.strip())

        wall_distances = []
        for log in wall_follow_logs:
            dist_match = re.search(r"WALL_FOLLOW,([-\d.eE]+)", log) 
            if dist_match:
                wall_distances.append(float(dist_match.group(1)))

        avg_wall_distance = sum(wall_distances)/len(wall_distances)


        #parse time log
        time_logs = []
        if os.path.exists(self.time_log_path):
            with open(self.time_log_path, "r") as f:
                for line in f:
                    
                    # Extract elapsed time
                    if line.startswith("Total Elapsed Time"):
                        match = re.search(r"([-+]?\d*\.\d+|\d+)", line)
                        if match:
                            elapsed_time = float(match.group(1))

                    # Extract wall follow time
                    elif line.startswith("Total Wall Following Time"):
                        match = re.search(r"([-+]?\d*\.\d+|\d+)", line)
                        if match:
                            time_spent_wall_following = float(match.group(1))

                    # Extract start timestamp
                    elif line.startswith("Trip Start Timestamp"):
                        match = re.search(r"(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2})", line)
                        if match:
                            trip_start_timestamp = datetime.strptime(match.group(1), "%Y-%m-%d %H:%M:%S")

                    # Extract end timestamp
                    elif line.startswith("Trip End Timestamp"):
                        match = re.search(r"(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2})", line)
                        if match:
                            trip_end_timestamp = datetime.strptime(match.group(1), "%Y-%m-%d %H:%M:%S")
        
        html_content = self.template.render(
            final_battery=f"{final_battery: .2f}",
            average_voltage=f"{average_voltage: .2f}",
            average_temp=f"{average_temp: .2f}",
            battery_lost=f"{battery_lost: .2f}",
            avg_wall_distance=f"{avg_wall_distance:.2f}",
            elapsed_time=f"{elapsed_time: .2f}",
            time_spent_wall_following=f"{time_spent_wall_following: .2f}",
            trip_start_timestamp=trip_start_timestamp.strftime("%Y-%m-%d %H:%M:%S"),
            trip_end_timestamp=trip_end_timestamp.strftime("%Y-%m-%d %H:%M:%S")
        )

        output_path = os.path.join(self.root_dir, "robot_report.html")
        with open(output_path, 'w') as f:
            f.write(html_content)
        print(f"Report generated at {output_path}!")

def main():
    rg = ReportGenerator()
    rg.generate_report()

if __name__ == "__main__":
    main()
