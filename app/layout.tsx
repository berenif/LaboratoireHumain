import type { Metadata } from "next";
import "./globals.css";

const configuredBasePath = process.env.PAGES_BASE_PATH?.trim() ?? "";
const basePath = configuredBasePath === "/"
  ? ""
  : configuredBasePath.replace(/\/+$/, "");
const faviconPath = `${basePath}/favicon.svg`;

export const metadata: Metadata = {
  title: "Protocole d’arrêt — Laboratoire Humain",
  description:
    "Une procédure absurde, un mannequin obstiné : appliquez une percussion et observez son retour debout.",
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
    <html lang="fr">
      <body className="antialiased">{children}</body>
    </html>
  );
}
