import { access, readFile, stat } from "node:fs/promises";
import path from "node:path";
import { fileURLToPath } from "node:url";

const root = fileURLToPath(new URL("..", import.meta.url));
const outputDirectory = path.join(root, "out");
const configuredBasePath = process.env.PAGES_BASE_PATH?.trim() ?? "";
const basePath = configuredBasePath === "/"
  ? ""
  : configuredBasePath.replace(/\/+$/, "");

if (basePath && !basePath.startsWith("/")) {
  throw new Error("PAGES_BASE_PATH must be empty or start with '/'.");
}

await Promise.all([
  access(path.join(outputDirectory, "index.html")),
  access(path.join(outputDirectory, "404.html")),
]);

const html = await readFile(path.join(outputDirectory, "index.html"), "utf8");
const localUrls = [
  ...html.matchAll(/\b(?:href|src)=["'](\/[^/][^"']*)["']/g),
].map((match) => match[1]);

if (localUrls.length === 0) {
  throw new Error("The static export does not contain any root-relative assets.");
}

for (const localUrl of new Set(localUrls)) {
  const pathname = decodeURIComponent(localUrl.split(/[?#]/, 1)[0]);

  if (basePath && pathname !== basePath && !pathname.startsWith(`${basePath}/`)) {
    throw new Error(`Asset URL is missing the Pages base path: ${localUrl}`);
  }

  const outputPathname = basePath
    ? pathname.slice(basePath.length)
    : pathname;
  const relativePath = outputPathname.replace(/^\/+/, "");
  const candidate = path.join(outputDirectory, relativePath);
  const candidateStat = await stat(candidate);

  if (!candidateStat.isFile() && !candidateStat.isDirectory()) {
    throw new Error(`Exported URL has no matching file: ${localUrl}`);
  }
}

console.log(
  `Verified GitHub Pages export (${localUrls.length} local asset references, base path ${basePath || "/"}).`,
);
