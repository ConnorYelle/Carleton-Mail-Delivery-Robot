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
METRIC_RULES = {"delivery_time": "lower", "battery_used": "lower", "wall_follow_time": "lower"}
EXCLUDE_METRICS = ["battery_start", "battery_end", "voltage_level", "temperature_level"]

runs = []
for file in sorted(os.listdir(LOG_DIR)):
    if file.endswith(".txt"):
        with open(os.path.join(LOG_DIR, file)) as f:
            data = {}
            for line in f:
                if "=" in line:
                    k, v = line.strip().split("=", 1)
                    k_s, v_s = k.strip(), v.strip()
                    try:
                        if v_s.lower() == "none" or v_s == "N/A":
                            data[k_s] = None
                        else:
                            data[k_s] = float(v_s)
                    except ValueError:
                        data[k_s] = v_s
            data["run"] = file
            match = re.search(r"\d{4}-\d{2}-\d{2}", file)
            data["date"] = match.group(0).replace("-", "") if match else "unknown"
            runs.append(data)

if not runs:
    exit(0)

df = pd.DataFrame(runs)
numeric_cols = df.select_dtypes(include=["number"]).columns
metrics = [c for c in numeric_cols if c not in METADATA_KEYS and c not in EXCLUDE_METRICS]
avg = df[metrics].mean()

with open(GITHUB_EVENT_PATH) as f:
    event = json.load(f)

commit_hash = "Unknown"
if "pull_request" in event:
    target_date = event["pull_request"]["created_at"][:10].replace("-", "")
    commit_hash = event["pull_request"]["head"]["sha"][:7]
elif "commits" in event:
    target_date = event["commits"][-1]["timestamp"][:10].replace("-", "")
    commit_hash = event.get("after", event["commits"][-1]["id"])[:7]

if target_date in df["date"].values:
    day_runs = df[df["date"] == target_date].copy()
    report_date = target_date
    is_fallback = False
else:
    day_runs = pd.DataFrame([df.iloc[-1]])
    report_date = day_runs.iloc[0]["date"]
    is_fallback = True

md_header = f"**Commit:** `{commit_hash}`\n\n"
if is_fallback:
    md_header += "**NO TEST RUNS TODAY. SHOWING LATEST DATA.**\n\n"

summary_counts = {"Improved": 0, "Worse": 0, "Same": 0}
temp_body = f"## Robot Metrics Report: {report_date}\n\n"

for _, run in day_runs.iterrows():
    temp_body += f"### Run: {run['run']}\n"
    temp_body += "| Metric | Value | Average | Status |\n"
    temp_body += "|--------|-------|--------|--------|\n"
    for m in metrics:
        val = run[m]
        if pd.isna(val):
            continue
        avg_val = avg[m]
        rule = METRIC_RULES.get(m, "higher")
        if abs(val - avg_val) < 0.001:
            status = "Same"
        elif (rule == "lower" and val < avg_val) or (rule == "higher" and val > avg_val):
            status = "Improved"
        else:
            status = "Worse"
        summary_counts[status] += 1
        temp_body += f"| {m} | {val:.2f} | {avg_val:.2f} | {status} |\n"
    temp_body += "\n"

summary_line = f"**Summary:** {summary_counts['Improved']} Improved, {summary_counts['Worse']} Worse, {summary_counts['Same']} Same\n\n"
full_md = md_header + summary_line + temp_body

g = Github(GITHUB_TOKEN)
repo = g.get_repo(GITHUB_REPOSITORY)
pr_number = None

if "pull_request" in event:
    pr_number = event["pull_request"]["number"]
elif "ref" in event and event["ref"].startswith("refs/heads/"):
    branch = event["ref"].split("/")[-1]
    prs = repo.get_pulls(state="open", head=f"{repo.owner.login}:{branch}")
    pr_number = prs[0].number if prs.totalCount > 0 else None

if pr_number:
    pr = repo.get_pull(pr_number)
    pr.create_issue_comment(full_md)
