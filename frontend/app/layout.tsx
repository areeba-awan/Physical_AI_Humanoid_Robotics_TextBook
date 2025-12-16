import type { Metadata } from "next";
import { Inter } from "next/font/google";
import "./globals.css";
import { Providers } from "@/components/providers";
import { Toaster } from "@/components/ui/toaster";

const inter = Inter({ subsets: ["latin"] });

export const metadata: Metadata = {
  title: "Physical AI & Humanoid Robotics",
  description: "A Comprehensive Guide to Embodied AI - Learn ROS 2, Gazebo, NVIDIA Isaac, and Vision-Language-Action systems",
  keywords: ["robotics", "AI", "ROS 2", "NVIDIA Isaac", "VLA", "humanoid robots", "simulation"],
  authors: [{ name: "Physical AI Team" }],
  openGraph: {
    title: "Physical AI & Humanoid Robotics",
    description: "A Comprehensive Guide to Embodied AI",
    type: "website",
  },
};

export default function RootLayout({
  children,
}: {
  children: React.ReactNode;
}) {
  return (
    <html lang="en" suppressHydrationWarning>
      <body className={inter.className}>
        <Providers>
          {children}
          <Toaster />
        </Providers>
      </body>
    </html>
  );
}
