"use client";

import { useEffect, useState } from "react";
import Link from "next/link";
import { BookOpen, Cpu, Layers, Zap, Eye, ArrowRight, Trophy, Clock, Target } from "lucide-react";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { useAuth } from "@/hooks/use-auth";
import { cn } from "@/lib/utils";

const modules = [
  {
    id: 1,
    title: "The Robotic Nervous System",
    subtitle: "ROS 2",
    icon: Cpu,
    color: "bg-blue-500",
    progress: 80,
    href: "/book/module-1-ros2/chapter-1-intro",
  },
  {
    id: 2,
    title: "The Digital Twin",
    subtitle: "Gazebo & Unity",
    icon: Layers,
    color: "bg-green-500",
    progress: 50,
    href: "/book/module-2-simulation/chapter-1-intro",
  },
  {
    id: 3,
    title: "The AI-Robot Brain",
    subtitle: "NVIDIA Isaac",
    icon: Zap,
    color: "bg-purple-500",
    progress: 25,
    href: "/book/module-3-isaac/chapter-1-intro",
  },
  {
    id: 4,
    title: "Vision-Language-Action",
    subtitle: "VLA Systems",
    icon: Eye,
    color: "bg-orange-500",
    progress: 0,
    href: "/book/module-4-vla/chapter-1-intro",
  },
];

export default function DashboardPage() {
  const { user, profile, fetchProfile } = useAuth();
  const [overallProgress, setOverallProgress] = useState(0);

  useEffect(() => {
    fetchProfile();
  }, [fetchProfile]);

  useEffect(() => {
    // Calculate overall progress
    const total = modules.reduce((sum, m) => sum + m.progress, 0);
    setOverallProgress(Math.round(total / modules.length));
  }, []);

  const getRecommendations = () => {
    if (!profile) return [];

    const recommendations = [];

    if (!profile.rosExperience) {
      recommendations.push({
        text: "Complete ROS 2 basics before advancing to Isaac",
        priority: "high",
      });
    }

    if (profile.linuxExperience === "none" || profile.linuxExperience === "basic") {
      recommendations.push({
        text: "Review Linux fundamentals for better simulation setup",
        priority: "medium",
      });
    }

    if (profile.softwareExperience === "beginner" || profile.softwareExperience === "none") {
      recommendations.push({
        text: "Try the beginner simulation lab first",
        priority: "medium",
      });
    }

    if (profile.hasJetsonKit) {
      recommendations.push({
        text: "Your Jetson kit will be useful in Module 3 - Isaac",
        priority: "info",
      });
    }

    return recommendations;
  };

  const recommendations = getRecommendations();

  return (
    <div className="space-y-8">
      {/* Welcome Section */}
      <div>
        <h1 className="text-3xl font-bold tracking-tight">
          Welcome back, {user?.name?.split(" ")[0]}!
        </h1>
        <p className="text-muted-foreground mt-1">
          Continue your Physical AI learning journey
        </p>
      </div>

      {/* Stats Cards */}
      <div className="grid gap-4 md:grid-cols-3">
        <Card>
          <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
            <CardTitle className="text-sm font-medium">Overall Progress</CardTitle>
            <Target className="h-4 w-4 text-muted-foreground" />
          </CardHeader>
          <CardContent>
            <div className="text-2xl font-bold">{overallProgress}%</div>
            <div className="mt-2 h-2 bg-muted rounded-full overflow-hidden">
              <div
                className="h-full bg-primary transition-all duration-500"
                style={{ width: `${overallProgress}%` }}
              />
            </div>
          </CardContent>
        </Card>

        <Card>
          <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
            <CardTitle className="text-sm font-medium">Chapters Completed</CardTitle>
            <BookOpen className="h-4 w-4 text-muted-foreground" />
          </CardHeader>
          <CardContent>
            <div className="text-2xl font-bold">
              {profile?.completedChapters?.length || 0} / 24
            </div>
            <p className="text-xs text-muted-foreground mt-1">
              Keep up the great work!
            </p>
          </CardContent>
        </Card>

        <Card>
          <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
            <CardTitle className="text-sm font-medium">Time Spent</CardTitle>
            <Clock className="h-4 w-4 text-muted-foreground" />
          </CardHeader>
          <CardContent>
            <div className="text-2xl font-bold">12h 30m</div>
            <p className="text-xs text-muted-foreground mt-1">
              This week
            </p>
          </CardContent>
        </Card>
      </div>

      {/* Continue Learning */}
      <Card>
        <CardHeader>
          <CardTitle>Continue Learning</CardTitle>
          <CardDescription>Pick up where you left off</CardDescription>
        </CardHeader>
        <CardContent>
          <Link href="/book/module-2-simulation/chapter-3-urdf">
            <div className="flex items-center justify-between p-4 rounded-lg border hover:bg-muted/50 transition-colors cursor-pointer">
              <div className="flex items-center gap-4">
                <div className="p-3 rounded-lg bg-green-500 text-white">
                  <Layers className="h-6 w-6" />
                </div>
                <div>
                  <h3 className="font-semibold">Chapter 2.3: URDF and Robot Models</h3>
                  <p className="text-sm text-muted-foreground">Module 2 - The Digital Twin</p>
                </div>
              </div>
              <div className="flex items-center gap-4">
                <div className="text-right">
                  <p className="text-sm font-medium">45 min remaining</p>
                  <p className="text-xs text-muted-foreground">60% complete</p>
                </div>
                <Button>
                  Continue
                  <ArrowRight className="ml-2 h-4 w-4" />
                </Button>
              </div>
            </div>
          </Link>
        </CardContent>
      </Card>

      {/* Module Progress */}
      <div>
        <h2 className="text-xl font-semibold mb-4">Your Progress</h2>
        <div className="grid gap-4 md:grid-cols-2">
          {modules.map((module) => (
            <Link key={module.id} href={module.href}>
              <Card className="h-full hover:shadow-md transition-shadow cursor-pointer">
                <CardContent className="pt-6">
                  <div className="flex items-start justify-between">
                    <div className="flex items-center gap-3">
                      <div className={cn("p-2 rounded-lg text-white", module.color)}>
                        <module.icon className="h-5 w-5" />
                      </div>
                      <div>
                        <h3 className="font-semibold">{module.title}</h3>
                        <p className="text-sm text-muted-foreground">{module.subtitle}</p>
                      </div>
                    </div>
                    <span className="text-sm font-medium">{module.progress}%</span>
                  </div>
                  <div className="mt-4 h-2 bg-muted rounded-full overflow-hidden">
                    <div
                      className={cn("h-full transition-all duration-500", module.color)}
                      style={{ width: `${module.progress}%` }}
                    />
                  </div>
                </CardContent>
              </Card>
            </Link>
          ))}
        </div>
      </div>

      {/* Recommendations */}
      {recommendations.length > 0 && (
        <Card>
          <CardHeader>
            <CardTitle className="flex items-center gap-2">
              <Trophy className="h-5 w-5 text-yellow-500" />
              Personalized Recommendations
            </CardTitle>
            <CardDescription>
              Based on your background ({profile?.softwareExperience || "beginner"} software, {profile?.rosExperience ? "has" : "no"} ROS experience)
            </CardDescription>
          </CardHeader>
          <CardContent>
            <ul className="space-y-3">
              {recommendations.map((rec, idx) => (
                <li key={idx} className="flex items-start gap-3">
                  <span
                    className={cn(
                      "w-2 h-2 rounded-full mt-2",
                      rec.priority === "high"
                        ? "bg-red-500"
                        : rec.priority === "medium"
                        ? "bg-yellow-500"
                        : "bg-blue-500"
                    )}
                  />
                  <span className="text-sm">{rec.text}</span>
                </li>
              ))}
            </ul>
          </CardContent>
        </Card>
      )}
    </div>
  );
}
