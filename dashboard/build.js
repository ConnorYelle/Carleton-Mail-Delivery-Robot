const fs = require("fs");
const path = require("path");
const template = fs.readFileSync("dashboard/template.html", "utf8");

const OWNER = process.env.GITHUB_REPOSITORY.split("/")[0];
const REPO = process.env.GITHUB_REPOSITORY.split("/")[1];
const BRANCH = process.env.GITHUB_REF_NAME;
const SHA = process.env.GITHUB_SHA;
const ACTOR = process.env.GITHUB_ACTOR;
const TIME = new Date().toISOString();

async function fetchCommits() {
  const url = `https://api.github.com/repos/${OWNER}/${REPO}/commits?sha=${BRANCH}&per_page=5`;
  const res = await fetch(url);
  return await res.json();
}

async function fetchPRs() {
  const url = `https://api.github.com/search/issues?q=repo:${OWNER}/${REPO}+type:pr+is:merged+base:${BRANCH}&sort=updated&order=desc`;
  const res = await fetch(url);
  const data = await res.json();
  return data.items || [];
}

async function generateDashboard() {
  const commits = await fetchCommits();
  const prs = await fetchPRs();

  const commitsHtml = commits.map(c =>
    `<a href="${c.html_url}" target="_blank">${c.commit.message}</a>`).join("<br>");

  const prsHtml = prs.map(p =>
    `<a href="${p.html_url}" target="_blank">#${p.number} ${p.title}</a>`).join("<br>");

  const output = template
    .replace(/{{REPO}}/g, `${OWNER}/${REPO}`)
    .replace(/{{BRANCH}}/g, BRANCH)
    .replace(/{{SHA}}/g, SHA)
    .replace(/{{ACTOR}}/g, ACTOR)
    .replace(/{{TIME}}/g, TIME)
    .replace(/{{COMMITS}}/g, commitsHtml)
    .replace(/{{PRS}}/g, prsHtml);

  const branchFolder = path.join("dist", BRANCH);
  fs.mkdirSync(branchFolder, { recursive: true });
  fs.writeFileSync(path.join(branchFolder, "index.html"), output);

  const distDir = "dist";
  const branches = fs.readdirSync(distDir).filter(f => fs.lstatSync(path.join(distDir,f)).isDirectory());
  const indexHtml = `
  <h1>Branch Reports</h1>
  <ul>
  ${branches.map(b => `<li><a href="${b}/">${b}</a></li>`).join("\n")}
  </ul>`;
  fs.writeFileSync(path.join(distDir, "index.html"), indexHtml);

  console.log("Dashboard generated for branch:", BRANCH);
}

generateDashboard();
