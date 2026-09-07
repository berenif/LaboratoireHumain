import type { Metadata } from "next";
import "./globals.css";

const configuredBasePath = process.env.PAGES_BASE_PATH?.trim() ?? "";
const basePath = configuredBasePath === "/"
  ? ""
  : configuredBasePath.replace(/\/+$/, "");
const faviconPath = `${basePath}/favicon.svg`;

export const metadata: Metadata = {
  title: "Embodied Motion Physics Demo",
  description:
    "Directly manipulate a connected humanoid, provoke corrective steps, and pull it through a physical fall.",
  other: {
    "codex-preview": "development",
  },
  icons: {
    icon: faviconPath,
    shortcut: faviconPath,
  },
};

export default function RootLayout({
  children,
}: Readonly<{
  children: React.ReactNode;
}>) {
  return (
    <html lang="en">
      <body className="antialiased">{children}</body>
    </html>
  );
}
