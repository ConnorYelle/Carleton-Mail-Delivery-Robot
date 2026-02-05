import os
import pandas as pd
import json
import re
from github import Github

GITHUB_REPOSITORY = os.environ["GITHUB_REPOSITORY"]
GITHUB_EVENT_PATH = os.environ.get("GITHUB_EVENT_PATH")
GITHUB_TOKEN = os.environ["GITHUB_TOKEN"]
LOG_DIR = "tools/logs/runs"
METADATA_KEYS = ["run", "date", "trip_start_time", "trip_end_time", "docked"]
METRIC_RULES = {"delivery_time": "lower", "battery_used": "lower", "wall_follow_time": "higher"}
EXCLUDE_METRICS = ["battery_start", "battery_end", "voltage_level", "temperature_level"]
IN_DE_METRICS = ["lidar_front_avg", "wall_distance_avg"]

if not GITHUB_EVENT_PATH or not os.path.exists(GITHUB_EVENT_PATH):
    exit(0)

runs = []
for file in sorted(os.listdir(LOG_DIR)):
    if file.endswith(".txt"):
        with open(os.path.join(LOG_DIR, file)) as f:
            data = {}
            for line in f:
                if "=" in line:
                    k, v = line.strip().split("=", 1)
                    try:
                        data[k.strip()] = float(v.strip())
                    except ValueError:
                        data[k.strip()] = v.strip() if v.strip().lower() not in ["none", "n/a"] else None
            data["run"] = file
            match = re.search(r"\d{4}-\d{2}-\d{2}", file)
            data["date"] = match.group(0).replace("-", "") if match else "unknown"
            runs.append(data)

if not runs:
    exit(0)

df = pd.DataFrame(runs)
numeric_cols = df.select_dtypes(include=["number"]).columns
metrics = [c for c in numeric_cols if c not in METADATA_KEYS and c not in EXCLUDE_METRICS]

with open(GITHUB_EVENT_PATH) as f:
    event = json.load(f)

commit_hash = "Unknown"
target_date = None

if "pull_request" in event:
    target_date = event["pull_request"]["created_at"][:10].replace("-", "")
    commit_hash = event["pull_request"]["head"]["sha"][:7]
elif "commits" in event:
    target_date = event["commits"][-1]["timestamp"][:10].replace("-", "")
    commit_hash = event.get("after", event["commits"][-1]["id"])[:7]

if target_date and target_date in df["date"].values:
    day_runs = df[df["date"] == target_date].copy()
    report_date = target_date
    is_fallback = False
else:
    day_runs = pd.DataFrame([df.iloc[-1]])
    report_date = day_runs.iloc[0]["date"]
    is_fallback = True

most_recent_run = day_runs.sort_values("run", ascending=False).iloc[0]

avg_source = df[df["run"] != most_recent_run["run"]]
avg = avg_source[metrics].mean() if not avg_source.empty else df[metrics].mean()

last_run_filename = None
repo = Github(GITHUB_TOKEN).get_repo(GITHUB_REPOSITORY)
pr_number = event.get("pull_request", {}).get("number")

if pr_number:
    pr = repo.get_pull(pr_number)
    comments = pr.get_issue_comments()
    for c in reversed(list(comments)):
        m = re.search(r"### Run: (\S+\.txt)", c.body)
        if m:
            last_run_filename = m.group(1)
            break

md_header = f"**Commit:** `{commit_hash}`\n\n"
if is_fallback:
    md_header += "**NO TEST RUNS TODAY. SHOWING LATEST DATA.**\n\n"

summary_counts = {"Improved": 0, "Worse": 0, "Same": 0, "Increased": 0, "Decreased": 0}
temp_body = f"## Robot Metrics Report: {report_date}\n\n"
temp_body += f"### Run: {most_recent_run['run']}\n"

if last_run_filename:
    temp_body += "| Metric | Value | Average | Overall Status | Comparison To Previous Commit Run |\n"
    temp_body += "|--------|-------|--------|--------|----------------------------|\n"
else:
    temp_body += "| Metric | Value | Average | Status |\n"
    temp_body += "|--------|-------|--------|--------|\n"

compare_run = None
if last_run_filename and last_run_filename in df["run"].values and most_recent_run["run"] != last_run_filename:
    compare_run = df[df["run"] == last_run_filename].iloc[0]

for m in metrics:
    if m not in most_recent_run:
        continue

    val = most_recent_run[m]
    if pd.isna(val):
        continue

    avg_val = avg[m]

    if m in IN_DE_METRICS:
        if abs(val - avg_val) < 0.001:
            status = "Same"
        elif val > avg_val:
            status = "Increased"
        else:
            status = "Decreased"
    else:
        rule = METRIC_RULES.get(m, "higher")
        if abs(val - avg_val) < 0.001:
            status = "Same"
        elif (rule == "lower" and val < avg_val) or (rule == "higher" and val > avg_val):
            status = "Improved"
        else:
            status = "Worse"

    summary_counts[status] += 1

    if last_run_filename:
        if compare_run is None or m not in compare_run or pd.isna(compare_run[m]):
            comparison = "No run"
        else:
            prev_val = compare_run[m]
            if m in IN_DE_METRICS:
                if abs(val - prev_val) < 0.001:
                    comparison = "Same"
                elif val > prev_val:
                    comparison = "Increased"
                else:
                    comparison = "Decreased"
            else:
                rule = METRIC_RULES.get(m, "higher")
                if abs(val - prev_val) < 0.001:
                    comparison = "Same"
                elif (rule == "lower" and val < prev_val) or (rule == "higher" and val > prev_val):
                    comparison = "Improved"
                else:
                    comparison = "Worse"

        temp_body += f"| {m} | {val:.2f} | {avg_val:.2f} | {status} | {comparison} |\n"
    else:
        temp_body += f"| {m} | {val:.2f} | {avg_val:.2f} | {status} |\n"

summary_line = f"**Summary:** {summary_counts['Improved']} Improved, {summary_counts['Worse']} Worse, {summary_counts['Same']} Same, {summary_counts['Increased']} Increased, {summary_counts['Decreased']} Decreased\n\n"
full_md = md_header + summary_line + temp_body

if pr_number:
    pr.create_issue_comment(full_md)
