// Checks the page's selfCrossing against src/geometry_cases.txt, the table geometry.rs also runs
// Run: node tests/page_geometry.mjs
import { readFileSync } from "node:fs";

const root = new URL("../src/", import.meta.url);
const page = readFileSync(new URL("page.html", root), "utf8");
const block = page.match(/\/\/ geometry:start\n([\s\S]*?)\/\/ geometry:end/);
if (!block) {
  console.error("no // geometry:start ... // geometry:end block in page.html");
  process.exit(1);
}
const selfCrossing = new Function(`${block[1]}\nreturn selfCrossing;`)();

let failed = 0;
let cases = 0;
for (const line of readFileSync(new URL("geometry_cases.txt", root), "utf8").split("\n")) {
  if (!line || line.startsWith("#")) continue;
  const [expected, list] = line.split("|");
  const points = list ? list.split(";").map((xy) => xy.split(",").map(Number)) : [];
  const got = selfCrossing(points) ? "cross" : "ok";
  cases++;
  if (got !== expected) {
    failed++;
    console.error(`FAIL ${line}: got ${got}`);
  }
}
console.log(`${cases - failed}/${cases} cases match`);
process.exit(failed ? 1 : 0);
