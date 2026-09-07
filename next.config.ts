import type { NextConfig } from "next";

const isGitHubPagesBuild = process.env.DEPLOY_TARGET === "github-pages";
const configuredBasePath = process.env.PAGES_BASE_PATH?.trim() ?? "";
const basePath = configuredBasePath === "/"
  ? ""
  : configuredBasePath.replace(/\/+$/, "");

if (basePath && !basePath.startsWith("/")) {
  throw new Error("PAGES_BASE_PATH must be empty or start with '/'.");
}

const nextConfig: NextConfig = isGitHubPagesBuild
  ? {
      // GitHub Pages can only serve static files. The workflow injects the
      // repository subpath returned by actions/configure-pages.
      output: "export",
      basePath,
      trailingSlash: true,
      images: { unoptimized: true },
    }
  : {};

export default nextConfig;
