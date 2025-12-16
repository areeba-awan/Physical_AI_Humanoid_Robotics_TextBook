"use client";

import Link from "next/link";
import { Button } from "@/components/ui/button";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import { Bot, BookOpen, Cpu, Eye, Rocket, Code, Layers, Zap, Github, MessageSquare } from "lucide-react";
import { useAuth } from "@/hooks/use-auth";
import { LanguageToggle } from "@/components/shared/language-toggle";

const modules = [
  {
    id: 1,
    title: "The Robotic Nervous System",
    subtitle: "ROS 2",
    description: "Master the communication framework that powers modern robots",
    icon: Cpu,
    color: "bg-blue-500",
    href: "http://localhost:3001/docs/module-1-ros2/chapter-1-intro",
  },
  {
    id: 2,
    title: "The Digital Twin",
    subtitle: "Gazebo & Unity",
    description: "Build and test robots in realistic simulated environments",
    icon: Layers,
    color: "bg-green-500",
    href: "http://localhost:3001/docs/module-2-simulation/chapter-1-intro",
  },
  {
    id: 3,
    title: "The AI-Robot Brain",
    subtitle: "NVIDIA Isaac",
    description: "Leverage GPU-accelerated perception and manipulation",
    icon: Zap,
    color: "bg-purple-500",
    href: "http://localhost:3001/docs/module-3-isaac/chapter-1-intro",
  },
  {
    id: 4,
    title: "Vision-Language-Action",
    subtitle: "VLA Systems",
    description: "Create robots that see, understand, and act intelligently",
    icon: Eye,
    color: "bg-orange-500",
    href: "http://localhost:3001/docs/module-4-vla/chapter-1-intro",
  },
];

const quickStartCards = [
  {
    title: "Quickstart Guide",
    description: "Set up your development environment in minutes",
    icon: Rocket,
    href: "http://localhost:3001/docs/quickstart",
  },
  {
    title: "Simulation Templates",
    description: "Pre-built environments to jumpstart your projects",
    icon: Code,
    href: "http://localhost:3001/docs/templates",
  },
  {
    title: "Capstone Recipes",
    description: "End-to-end project guides for real applications",
    icon: BookOpen,
    href: "http://localhost:3001/docs/capstone",
  },
];

const features = [
  {
    title: "Hands-on Labs",
    description: "Real code, real robots, real results",
    icon: Code,
  },
  {
    title: "Interactive Visualizations",
    description: "See concepts come to life in 3D",
    icon: Eye,
  },
  {
    title: "VLA Integrations",
    description: "Cutting-edge AI for physical systems",
    icon: Bot,
  },
];

export default function HomePage() {
  const { user, isLoading } = useAuth();

  return (
    <div className="min-h-screen bg-gradient-to-b from-background to-muted">
      {/* Navigation */}
      <nav className="border-b bg-background/95 backdrop-blur supports-[backdrop-filter]:bg-background/60">
        <div className="container flex h-16 items-center justify-between">
          <div className="flex items-center gap-6">
            <Link href="/" className="flex items-center gap-2">
              <Bot className="h-8 w-8 text-primary" />
              <span className="font-bold text-xl hidden md:inline">Physical AI</span>
            </Link>
            <div className="hidden md:flex items-center gap-4">
              <Link href="http://localhost:3001/" className="text-muted-foreground hover:text-foreground transition-colors">
                Modules
              </Link>
              <Link href="http://localhost:3001/docs/intro" className="text-muted-foreground hover:text-foreground transition-colors">
                Book Overview
              </Link>
            </div>
          </div>
          <div className="flex items-center gap-4">
            <LanguageToggle />
            <Link href="https://github.com" target="_blank">
              <Button variant="ghost" size="icon">
                <Github className="h-5 w-5" />
              </Button>
            </Link>
            <Link href="/chat">
              <Button variant="ghost" size="icon">
                <MessageSquare className="h-5 w-5" />
              </Button>
            </Link>
            {isLoading ? (
              <div className="w-20 h-9 bg-muted animate-pulse rounded-md" />
            ) : user ? (
              <Link href="/dashboard">
                <Button>Dashboard</Button>
              </Link>
            ) : (
              <div className="flex items-center gap-2">
                <Link href="/signin">
                  <Button variant="ghost">Sign In</Button>
                </Link>
                <Link href="/signup">
                  <Button>Get Started</Button>
                </Link>
              </div>
            )}
          </div>
        </div>
      </nav>

      {/* Hero Section */}
      <section className="container py-24 text-center">
        <h1 className="text-4xl md:text-6xl font-bold tracking-tight mb-6">
          Physical AI &{" "}
          <span className="text-primary">Humanoid Robotics</span>
        </h1>
        <p className="text-xl text-muted-foreground max-w-2xl mx-auto mb-4">
          A Comprehensive Guide to Embodied AI
        </p>
        <p className="text-lg text-muted-foreground max-w-xl mx-auto mb-8">
          Hands-on labs and ready-to-run examples for building intelligent robots
        </p>
        <div className="flex flex-col sm:flex-row items-center justify-center gap-4">
          <Link href="/signup">
            <Button size="lg" className="text-lg px-8">
              Get Started
            </Button>
          </Link>
          <Link href="http://localhost:3001/">
            <Button size="lg" variant="outline" className="text-lg px-8">
              View Book
            </Button>
          </Link>
        </div>
      </section>

      {/* Quick Start Cards */}
      <section className="container py-12">
        <div className="grid md:grid-cols-3 gap-6">
          {quickStartCards.map((card) => (
            <Link key={card.title} href={card.href}>
              <Card className="h-full hover:shadow-lg transition-shadow cursor-pointer">
                <CardHeader>
                  <card.icon className="h-10 w-10 text-primary mb-2" />
                  <CardTitle>{card.title}</CardTitle>
                  <CardDescription>{card.description}</CardDescription>
                </CardHeader>
              </Card>
            </Link>
          ))}
        </div>
      </section>

      {/* Course Modules */}
      <section className="container py-16">
        <h2 className="text-3xl font-bold text-center mb-12">Course Modules</h2>
        <div className="grid md:grid-cols-2 gap-6">
          {modules.map((module) => (
            <Link key={module.id} href={module.href}>
              <Card className="h-full hover:shadow-lg transition-all hover:scale-[1.02] cursor-pointer">
                <CardHeader>
                  <div className="flex items-start justify-between">
                    <div className={`p-3 rounded-lg ${module.color} text-white`}>
                      <module.icon className="h-6 w-6" />
                    </div>
                    <span className="text-sm text-muted-foreground">Module {module.id}</span>
                  </div>
                  <CardTitle className="mt-4">{module.title}</CardTitle>
                  <p className="text-sm font-medium text-primary">{module.subtitle}</p>
                  <CardDescription className="mt-2">{module.description}</CardDescription>
                </CardHeader>
                <CardContent>
                  <Button variant="ghost" className="group">
                    Start Module
                    <Rocket className="ml-2 h-4 w-4 group-hover:translate-x-1 transition-transform" />
                  </Button>
                </CardContent>
              </Card>
            </Link>
          ))}
        </div>
      </section>

      {/* Features */}
      <section className="container py-16">
        <h2 className="text-3xl font-bold text-center mb-12">Key Features</h2>
        <div className="grid md:grid-cols-3 gap-8">
          {features.map((feature) => (
            <div key={feature.title} className="text-center">
              <div className="mx-auto w-16 h-16 rounded-full bg-primary/10 flex items-center justify-center mb-4">
                <feature.icon className="h-8 w-8 text-primary" />
              </div>
              <h3 className="text-xl font-semibold mb-2">{feature.title}</h3>
              <p className="text-muted-foreground">{feature.description}</p>
            </div>
          ))}
        </div>
      </section>

      {/* CTA Section */}
      <section className="container py-20">
        <div className="bg-primary/5 rounded-2xl p-12 text-center">
          <h2 className="text-3xl font-bold mb-4">Ready to build intelligent robots?</h2>
          <p className="text-muted-foreground mb-8 max-w-xl mx-auto">
            Join thousands of engineers learning Physical AI. Start your journey today.
          </p>
          <div className="flex flex-col sm:flex-row items-center justify-center gap-4">
            <Link href="/signup">
              <Button size="lg">Get Started</Button>
            </Link>
            <Link href="https://github.com" target="_blank">
              <Button size="lg" variant="outline">
                <Github className="mr-2 h-5 w-5" />
                Contribute
              </Button>
            </Link>
          </div>
        </div>
      </section>

      {/* Footer */}
      <footer className="border-t">
        <div className="container py-12">
          <div className="grid md:grid-cols-4 gap-8">
            <div>
              <div className="flex items-center gap-2 mb-4">
                <Bot className="h-6 w-6 text-primary" />
                <span className="font-bold">Physical AI</span>
              </div>
              <p className="text-sm text-muted-foreground">
                Building the future of robotics education.
              </p>
            </div>
            <div>
              <h4 className="font-semibold mb-4">Book</h4>
              <ul className="space-y-2 text-sm text-muted-foreground">
                <li><Link href="http://localhost:3001/" className="hover:text-foreground">Overview</Link></li>
                <li><Link href="http://localhost:3001/docs/module-1-ros2" className="hover:text-foreground">Modules</Link></li>
                <li><Link href="http://localhost:3001/docs/quickstart" className="hover:text-foreground">Quickstart</Link></li>
              </ul>
            </div>
            <div>
              <h4 className="font-semibold mb-4">Community</h4>
              <ul className="space-y-2 text-sm text-muted-foreground">
                <li><Link href="#" className="hover:text-foreground">Stack Overflow</Link></li>
                <li><Link href="#" className="hover:text-foreground">Discord</Link></li>
                <li><Link href="#" className="hover:text-foreground">X (Twitter)</Link></li>
              </ul>
            </div>
            <div>
              <h4 className="font-semibold mb-4">Social</h4>
              <ul className="space-y-2 text-sm text-muted-foreground">
                <li><Link href="https://github.com" className="hover:text-foreground">GitHub</Link></li>
                <li><Link href="#" className="hover:text-foreground">YouTube</Link></li>
                <li><Link href="#" className="hover:text-foreground">LinkedIn</Link></li>
              </ul>
            </div>
          </div>
          <div className="border-t mt-8 pt-8 text-center text-sm text-muted-foreground">
            <p>&copy; 2025 Physical AI Textbook. All rights reserved.</p>
          </div>
        </div>
      </footer>
    </div>
  );
}
