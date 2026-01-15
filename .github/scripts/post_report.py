import os
import pandas as pd
import json
from github import Github

GITHUB_REPOSITORY = os.environ.get("GITHUB_REPOSITORY")
GITHUB_EVENT_PATH = os.environ.get("GITHUB_EVENT_PATH")
GITHUB_TOKEN = os.environ.get("GITHUB_TOKEN")

LOG_DIR = "RobotDashboard/mail-delivery-robot/tools/logs/runs"
METADATA_KEYS = ["run", "date", "trip_start_time", "trip_end_time", "docked"]
METRIC_RULES = {"delivery_time": "lower", "battery_used": "lower", "wall_follow_time": "lower"}

if not os.path.isdir(LOG_DIR):
    exit(0)

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
                        if v_s.lower() in ("none", "n/a"):
                            data[k_s] = None
                        else:
                            data[k_s] = float(v_s)
                    except ValueError:
                        data[k_s] = v_s
            data["run"] = file
            parts = file.split("_")
            data["date"] = parts[1] if len(parts) > 1 else "unknown"
            runs.append(data)

if not runs:
    exit(0)

df = pd.DataFrame(runs)
numeric_cols = df.select_dtypes(include=["number"]).columns
metrics = [c for c in numeric_cols if c not in METADATA_KEYS]
avg = df[metrics].mean()

event = {}
if GITHUB_EVENT_PATH and os.path.isfile(GITHUB_EVENT_PATH):
    with open(GITHUB_EVENT_PATH) as f:
        event = json.load(f)

commit_sha = None
if "pull_request" in event:
    commit_sha = event["pull_request"]["head"]["sha"]
elif "after" in event:
    commit_sha = event["after"]
elif "commits" in event and event["commits"]:
    commit_sha = event["commits"][-1]["id"]

if not commit_sha:
    exit(0)

short_sha = commit_sha[:7]

target_date = None
if "pull_request" in event:
    target_date = event["pull_request"]["created_at"][:10].replace("-", "")
elif "commits" in event and event["commits"]:
    target_date = event["commits"][-1]["timestamp"][:10].replace("-", "")

if target_date and target_date in df["date"].values:
    day_runs = df[df["date"] == target_date].copy()
    report_date = target_date
    is_fallback = False
else:
    day_runs = pd.DataFrame([df.iloc[-1]])
    report_date = day_runs.iloc[0]["date"]
    is_fallback = True

md_header = "**NO TEST RUNS TODAY. SHOWING LATEST DATA.**\n\n" if is_fallback else ""
summary_counts = {"Improved": 0, "Worse": 0, "Same": 0}

body = f"## Robot Metrics Report\n\n**Commit:** `{short_sha}`\n\n"
body += md_header
body += f"**Report Date:** {report_date}\n\n"

for _, run in day_runs.iterrows():
    body += f"### Run: {run['run']}\n"
    body += "| Metric | Value | Average | Status |\n"
    body += "|--------|-------|--------|--------|\n"
    for m in metrics:
        val = run.get(m)
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
        body += f"| {m} | {val:.2f} | {avg_val:.2f} | {status} |\n"
    body += "\n"

body += f"**Summary:** {summary_counts['Improved']} Improved, {summary_counts['Worse']} Worse, {summary_counts['Same']} Same\n"

if not (GITHUB_TOKEN and GITHUB_REPOSITORY):
    exit(0)

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
    pr.create_issue_comment(body)
