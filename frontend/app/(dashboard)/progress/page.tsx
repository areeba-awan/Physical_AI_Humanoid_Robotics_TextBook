"use client";

import { useEffect } from "react";
import Link from "next/link";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { useAuth } from "@/hooks/use-auth";
import { BookOpen, CheckCircle, Circle, Clock, Trophy, Target, ArrowRight } from "lucide-react";
import { cn } from "@/lib/utils";

const chapters = [
  {
    module: 1,
    moduleTitle: "The Robotic Nervous System (ROS 2)",
    color: "bg-blue-500",
    chapters: [
      { id: "1-1", title: "Introduction to ROS 2", completed: true, time: "45 min" },
      { id: "1-2", title: "Nodes, Topics, and Services", completed: true, time: "60 min" },
      { id: "1-3", title: "Actions and Parameters", completed: true, time: "45 min" },
      { id: "1-4", title: "Launch Files and Configuration", completed: true, time: "30 min" },
      { id: "1-5", title: "Building Custom Packages", completed: false, time: "50 min" },
      { id: "1-6", title: "Lab: Building a ROS 2 Robot", completed: false, time: "90 min" },
    ],
  },
  {
    module: 2,
    moduleTitle: "The Digital Twin (Gazebo & Unity)",
    color: "bg-green-500",
    chapters: [
      { id: "2-1", title: "Introduction to Simulation", completed: true, time: "40 min" },
      { id: "2-2", title: "Gazebo Basics", completed: true, time: "55 min" },
      { id: "2-3", title: "URDF and Robot Models", completed: false, time: "60 min" },
      { id: "2-4", title: "Unity Robotics Hub", completed: false, time: "50 min" },
      { id: "2-5", title: "Sensor Simulation", completed: false, time: "45 min" },
      { id: "2-6", title: "Lab: Digital Twin Pipeline", completed: false, time: "90 min" },
    ],
  },
  {
    module: 3,
    moduleTitle: "The AI-Robot Brain (NVIDIA Isaac)",
    color: "bg-purple-500",
    chapters: [
      { id: "3-1", title: "Introduction to NVIDIA Isaac", completed: true, time: "45 min" },
      { id: "3-2", title: "Isaac Sim Fundamentals", completed: false, time: "60 min" },
      { id: "3-3", title: "Perception with Isaac", completed: false, time: "55 min" },
      { id: "3-4", title: "Manipulation Planning", completed: false, time: "50 min" },
      { id: "3-5", title: "Isaac ROS Integration", completed: false, time: "45 min" },
      { id: "3-6", title: "Lab: AI-Powered Manipulation", completed: false, time: "90 min" },
    ],
  },
  {
    module: 4,
    moduleTitle: "Vision-Language-Action (VLA)",
    color: "bg-orange-500",
    chapters: [
      { id: "4-1", title: "Introduction to VLA", completed: false, time: "50 min" },
      { id: "4-2", title: "Vision Transformers for Robots", completed: false, time: "60 min" },
      { id: "4-3", title: "Language Models for Robotics", completed: false, time: "55 min" },
      { id: "4-4", title: "Action Generation", completed: false, time: "50 min" },
      { id: "4-5", title: "End-to-End VLA Systems", completed: false, time: "60 min" },
      { id: "4-6", title: "Capstone: VLA Robot Project", completed: false, time: "120 min" },
    ],
  },
];

export default function ProgressPage() {
  const { profile, fetchProfile } = useAuth();

  useEffect(() => {
    fetchProfile();
  }, [fetchProfile]);

  const totalChapters = chapters.reduce((sum, m) => sum + m.chapters.length, 0);
  const completedChapters = chapters.reduce(
    (sum, m) => sum + m.chapters.filter((c) => c.completed).length,
    0
  );
  const overallProgress = Math.round((completedChapters / totalChapters) * 100);

  const totalTime = chapters.reduce(
    (sum, m) =>
      sum + m.chapters.reduce((s, c) => s + parseInt(c.time), 0),
    0
  );
  const completedTime = chapters.reduce(
    (sum, m) =>
      sum + m.chapters.filter((c) => c.completed).reduce((s, c) => s + parseInt(c.time), 0),
    0
  );

  return (
    <div className="space-y-8">
      <div>
        <h1 className="text-3xl font-bold tracking-tight">Learning Progress</h1>
        <p className="text-muted-foreground mt-1">
          Track your journey through Physical AI & Humanoid Robotics
        </p>
      </div>

      {/* Stats Overview */}
      <div className="grid gap-4 md:grid-cols-4">
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
            <CardTitle className="text-sm font-medium">Chapters</CardTitle>
            <BookOpen className="h-4 w-4 text-muted-foreground" />
          </CardHeader>
          <CardContent>
            <div className="text-2xl font-bold">
              {completedChapters} / {totalChapters}
            </div>
            <p className="text-xs text-muted-foreground mt-1">
              {totalChapters - completedChapters} remaining
            </p>
          </CardContent>
        </Card>

        <Card>
          <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
            <CardTitle className="text-sm font-medium">Time Invested</CardTitle>
            <Clock className="h-4 w-4 text-muted-foreground" />
          </CardHeader>
          <CardContent>
            <div className="text-2xl font-bold">
              {Math.floor(completedTime / 60)}h {completedTime % 60}m
            </div>
            <p className="text-xs text-muted-foreground mt-1">
              of {Math.floor(totalTime / 60)}h total
            </p>
          </CardContent>
        </Card>

        <Card>
          <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
            <CardTitle className="text-sm font-medium">Achievements</CardTitle>
            <Trophy className="h-4 w-4 text-muted-foreground" />
          </CardHeader>
          <CardContent>
            <div className="text-2xl font-bold">3</div>
            <p className="text-xs text-muted-foreground mt-1">badges earned</p>
          </CardContent>
        </Card>
      </div>

      {/* Module Progress */}
      <div className="space-y-6">
        {chapters.map((module) => {
          const moduleCompleted = module.chapters.filter((c) => c.completed).length;
          const moduleProgress = Math.round(
            (moduleCompleted / module.chapters.length) * 100
          );

          return (
            <Card key={module.module}>
              <CardHeader>
                <div className="flex items-center justify-between">
                  <div className="flex items-center gap-3">
                    <div className={cn("w-3 h-3 rounded-full", module.color)} />
                    <div>
                      <CardTitle className="text-lg">
                        Module {module.module}: {module.moduleTitle}
                      </CardTitle>
                      <CardDescription>
                        {moduleCompleted} of {module.chapters.length} chapters completed
                      </CardDescription>
                    </div>
                  </div>
                  <div className="text-right">
                    <span className="text-2xl font-bold">{moduleProgress}%</span>
                  </div>
                </div>
                <div className="mt-3 h-2 bg-muted rounded-full overflow-hidden">
                  <div
                    className={cn("h-full transition-all duration-500", module.color)}
                    style={{ width: `${moduleProgress}%` }}
                  />
                </div>
              </CardHeader>
              <CardContent>
                <div className="space-y-2">
                  {module.chapters.map((chapter, idx) => (
                    <Link
                      key={chapter.id}
                      href={`/book/module-${module.module}/chapter-${idx + 1}`}
                      className="block"
                    >
                      <div
                        className={cn(
                          "flex items-center justify-between p-3 rounded-lg border transition-colors",
                          chapter.completed
                            ? "bg-muted/50"
                            : "hover:bg-muted/30"
                        )}
                      >
                        <div className="flex items-center gap-3">
                          {chapter.completed ? (
                            <CheckCircle className="h-5 w-5 text-green-500" />
                          ) : (
                            <Circle className="h-5 w-5 text-muted-foreground" />
                          )}
                          <div>
                            <p
                              className={cn(
                                "font-medium",
                                chapter.completed && "text-muted-foreground"
                              )}
                            >
                              {chapter.title}
                            </p>
                            <p className="text-xs text-muted-foreground">
                              {chapter.time}
                            </p>
                          </div>
                        </div>
                        <Button
                          variant={chapter.completed ? "ghost" : "outline"}
                          size="sm"
                        >
                          {chapter.completed ? "Review" : "Start"}
                          <ArrowRight className="ml-1 h-3 w-3" />
                        </Button>
                      </div>
                    </Link>
                  ))}
                </div>
              </CardContent>
            </Card>
          );
        })}
      </div>
    </div>
  );
}
