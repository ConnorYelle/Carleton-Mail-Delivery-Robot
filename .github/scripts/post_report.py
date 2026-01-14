import os
import pandas as pd
import json
from github import Github

GITHUB_REPOSITORY = os.environ["GITHUB_REPOSITORY"]
GITHUB_EVENT_PATH = os.environ.get("GITHUB_EVENT_PATH")
GITHUB_TOKEN = os.environ["GITHUB_TOKEN"]

LOG_DIR = "src/tools/logs/runs"
EXEMPT_KEYS = ["temperature_level", "voltage_level"]
METRIC_RULES = { "delivery_time": "lower", "battery_used": "lower" }

runs = []
for file in sorted(os.listdir(LOG_DIR)):
    if file.endswith(".txt"):
        with open(os.path.join(LOG_DIR, file)) as f:
            data = {}
            for line in f:
                if "=" in line:
                    k, v = line.strip().split("=")
                    data[k.strip()] = float(v) if v.strip().replace(".","",1).isdigit() else v.strip()
            data["run"] = file
            parts = file.split("_")
            data["date"] = parts[1] if len(parts) > 1 else "unknown"
            runs.append(data)

if not runs:
    exit(0)

df = pd.DataFrame(runs)
metrics = [c for c in df.columns if c not in EXEMPT_KEYS + ["run", "trip_start_time", "trip_end_time", "battery_start", "battery_end", "date"]]

with open(GITHUB_EVENT_PATH) as f:
    event = json.load(f)

target_date = None
if "pull_request" in event:
    target_date = event["pull_request"]["created_at"][:10].replace("-", "")
elif "commits" in event:
    target_date = event["commits"][-1]["timestamp"][:10].replace("-", "")

if target_date in df["date"].values:
    latest = df[df["date"] == target_date].iloc[-1]
    report_date = target_date
else:
    latest = df.iloc[-1]
    report_date = latest["date"]

avg = df[metrics].mean()

comparison = []
for m in metrics:
    val = latest[m]
    avg_val = avg[m]
    rule = METRIC_RULES.get(m, "higher")
    if rule == "higher":
        status = "Improved" if val > avg_val else "Worse" if val < avg_val else "Same"
    else:
        status = "Improved" if val < avg_val else "Worse" if val > avg_val else "Same"
    comparison.append({"metric": m, "value": val, "avg": avg_val, "status": status})

md = f"## Robot Metrics Report - Run Date: {report_date}\n\n"
md += "| Metric | Latest Value | Average | Status |\n"
md += "|--------|-------------|---------|--------|\n"
for c in comparison:
    md += f"| {c['metric']} | {c['value']:.2f} | {c['avg']:.2f} | {c['status']} |\n"

improved_count = sum(1 for c in comparison if c['status'] == 'Improved')
worse_count = sum(1 for c in comparison if c['status'] == 'Worse')
same_count = sum(1 for c in comparison if c['status'] == 'Same')
md = f"**Summary:** {improved_count} improved, {worse_count} worse, {same_count} same\n\n" + md

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
    marker = "<!-- ROBOT_METRICS_COMMENT -->"
    existing_comments = pr.get_issue_comments()
    metrics_comment = None
    for c in existing_comments:
        if marker in c.body:
            metrics_comment = c
            break
    md_with_marker = f"{marker}\n\n{md}"
    if metrics_comment:
        metrics_comment.edit(md_with_marker)
    else:
        pr.create_issue_comment(md_with_marker)
