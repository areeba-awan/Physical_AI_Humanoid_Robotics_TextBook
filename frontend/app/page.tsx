"use client";

import Link from "next/link";
import { Button } from "@/components/ui/button";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import { Bot, BookOpen, Cpu, Eye, Rocket, Code, Layers, Zap, Github, MessageSquare } from "lucide-react";
import { useAuth } from "@/hooks/use-auth";
import { LanguageToggle } from "@/components/shared/language-toggle";
import { TranslatedText } from "@/components/shared/translation-provider";
import { usePreloadHomeTranslations } from "@/hooks/use-preload-home-translations";
import { useEffect } from "react";

const modules = [
  {
    id: 1,
    title: "The Robotic Nervous System",
    subtitle: "ROS 2",
    description: "Master the communication framework that powers modern robots",
    icon: Cpu,
    color: "bg-blue-500",
    href: "http://localhost:3001/docs/module-1-ros2/chapter-1-intro",
    titleKey: "module1-title",
    subtitleKey: "module1-subtitle",
    descriptionKey: "module1-description"
  },
  {
    id: 2,
    title: "The Digital Twin",
    subtitle: "Gazebo & Unity",
    description: "Build and test robots in realistic simulated environments",
    icon: Layers,
    color: "bg-green-500",
    href: "http://localhost:3001/docs/module-2-simulation/chapter-1-intro",
    titleKey: "module2-title",
    subtitleKey: "module2-subtitle",
    descriptionKey: "module2-description"
  },
  {
    id: 3,
    title: "The AI-Robot Brain",
    subtitle: "NVIDIA Isaac",
    description: "Leverage GPU-accelerated perception and manipulation",
    icon: Zap,
    color: "bg-purple-500",
    href: "http://localhost:3001/docs/module-3-isaac/chapter-1-intro",
    titleKey: "module3-title",
    subtitleKey: "module3-subtitle",
    descriptionKey: "module3-description"
  },
  {
    id: 4,
    title: "Vision-Language-Action",
    subtitle: "VLA Systems",
    description: "Create robots that see, understand, and act intelligently",
    icon: Eye,
    color: "bg-orange-500",
    href: "http://localhost:3001/docs/module-4-vla/chapter-1-intro",
    titleKey: "module4-title",
    subtitleKey: "module4-subtitle",
    descriptionKey: "module4-description"
  },
];

const quickStartCards = [
  {
    title: "Quickstart Guide",
    description: "Set up your development environment in minutes",
    icon: Rocket,
    href: "http://localhost:3001/docs/quickstart",
    titleKey: "quickstart-title",
    descriptionKey: "quickstart-description"
  },
  {
    title: "Simulation Templates",
    description: "Pre-built environments to jumpstart your projects",
    icon: Code,
    href: "http://localhost:3001/docs/templates",
    titleKey: "templates-title",
    descriptionKey: "templates-description"
  },
  {
    title: "Capstone Recipes",
    description: "End-to-end project guides for real applications",
    icon: BookOpen,
    href: "http://localhost:3001/docs/capstone",
    titleKey: "capstone-title",
    descriptionKey: "capstone-description"
  },
];

const features = [
  {
    title: "Hands-on Labs",
    description: "Real code, real robots, real results",
    icon: Code,
    titleKey: "feature1-title",
    descriptionKey: "feature1-description"
  },
  {
    title: "Interactive Visualizations",
    description: "See concepts come to life in 3D",
    icon: Eye,
    titleKey: "feature2-title",
    descriptionKey: "feature2-description"
  },
  {
    title: "VLA Integrations",
    description: "Cutting-edge AI for physical systems",
    icon: Bot,
    titleKey: "feature3-title",
    descriptionKey: "feature3-description"
  },
];

export default function HomePage() {
  const { user, isLoading } = useAuth();
  const { preloadHomeContent } = usePreloadHomeTranslations();

  useEffect(() => {
    if (user) {
      preloadHomeContent();
    }
  }, [user, preloadHomeContent]);

  return (
    <div className="min-h-screen bg-gradient-to-b from-background to-muted">
      {/* Navigation */}
      <nav className="border-b bg-background/95 backdrop-blur supports-[backdrop-filter]:bg-background/60">
        <div className="container flex h-16 items-center justify-between">
          <div className="flex items-center gap-6">
            <Link href="/" className="flex items-center gap-2">
              <Bot className="h-8 w-8 text-primary" />
              <span className="font-bold text-xl hidden md:inline">
                <TranslatedText id="nav-title">Physical AI</TranslatedText>
              </span>
            </Link>
            <div className="hidden md:flex items-center gap-4">
              <Link href="http://localhost:3001/" className="text-muted-foreground hover:text-foreground transition-colors">
                <TranslatedText id="nav-modules">Modules</TranslatedText>
              </Link>
              <Link href="http://localhost:3001/docs/intro" className="text-muted-foreground hover:text-foreground transition-colors">
                <TranslatedText id="nav-book-overview">Book Overview</TranslatedText>
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
                <Button>
                  <TranslatedText id="dashboard-btn">Dashboard</TranslatedText>
                </Button>
              </Link>
            ) : (
              <div className="flex items-center gap-2">
                <Link href="/signin">
                  <Button variant="ghost">
                    <TranslatedText id="sign-in-btn">Sign In</TranslatedText>
                  </Button>
                </Link>
                <Link href="/signup">
                  <Button>
                    <TranslatedText id="get-started-btn">Get Started</TranslatedText>
                  </Button>
                </Link>
              </div>
            )}
          </div>
        </div>
      </nav>

      {/* Hero Section */}
      <section className="container py-24 text-center">
        <h1 className="text-4xl md:text-6xl font-bold tracking-tight mb-6">
          <TranslatedText id="hero-title">Physical AI &</TranslatedText>
          {" "}
          <span className="text-primary">
            <TranslatedText id="hero-subtitle">Humanoid Robotics</TranslatedText>
          </span>
        </h1>
        <p className="text-xl text-muted-foreground max-w-2xl mx-auto mb-4">
          <TranslatedText id="hero-description">A Comprehensive Guide to Embodied AI</TranslatedText>
        </p>
        <p className="text-lg text-muted-foreground max-w-xl mx-auto mb-8">
          <TranslatedText id="hero-tagline">Hands-on labs and ready-to-run examples for building intelligent robots</TranslatedText>
        </p>
        <div className="flex flex-col sm:flex-row items-center justify-center gap-4">
          <Link href="/signup">
            <Button size="lg" className="text-lg px-8">
              <TranslatedText id="hero-get-started">Get Started</TranslatedText>
            </Button>
          </Link>
          <Link href="http://localhost:3001/">
            <Button size="lg" variant="outline" className="text-lg px-8">
              <TranslatedText id="hero-view-book">View Book</TranslatedText>
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
                  <CardTitle>
                    <TranslatedText id={card.titleKey}>{card.title}</TranslatedText>
                  </CardTitle>
                  <CardDescription>
                    <TranslatedText id={card.descriptionKey}>{card.description}</TranslatedText>
                  </CardDescription>
                </CardHeader>
              </Card>
            </Link>
          ))}
        </div>
      </section>

      {/* Course Modules */}
      <section className="container py-16">
        <h2 className="text-3xl font-bold text-center mb-12">
          <TranslatedText id="modules-section-title">Course Modules</TranslatedText>
        </h2>
        <div className="grid md:grid-cols-2 gap-6">
          {modules.map((module) => (
            <Link key={module.id} href={module.href}>
              <Card className="h-full hover:shadow-lg transition-all hover:scale-[1.02] cursor-pointer">
                <CardHeader>
                  <div className="flex items-start justify-between">
                    <div className={`p-3 rounded-lg ${module.color} text-white`}>
                      <module.icon className="h-6 w-6" />
                    </div>
                    <span className="text-sm text-muted-foreground">
                      <TranslatedText id={`module-${module.id}-label`}>Module {module.id}</TranslatedText>
                    </span>
                  </div>
                  <CardTitle className="mt-4">
                    <TranslatedText id={module.titleKey}>{module.title}</TranslatedText>
                  </CardTitle>
                  <p className="text-sm font-medium text-primary">
                    <TranslatedText id={module.subtitleKey}>{module.subtitle}</TranslatedText>
                  </p>
                  <CardDescription className="mt-2">
                    <TranslatedText id={module.descriptionKey}>{module.description}</TranslatedText>
                  </CardDescription>
                </CardHeader>
                <CardContent>
                  <Button variant="ghost" className="group">
                    <TranslatedText id={`module-${module.id}-btn`}>Start Module</TranslatedText>
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
        <h2 className="text-3xl font-bold text-center mb-12">
          <TranslatedText id="features-section-title">Key Features</TranslatedText>
        </h2>
        <div className="grid md:grid-cols-3 gap-8">
          {features.map((feature) => (
            <div key={feature.title} className="text-center">
              <div className="mx-auto w-16 h-16 rounded-full bg-primary/10 flex items-center justify-center mb-4">
                <feature.icon className="h-8 w-8 text-primary" />
              </div>
              <h3 className="text-xl font-semibold mb-2">
                <TranslatedText id={feature.titleKey}>{feature.title}</TranslatedText>
              </h3>
              <p className="text-muted-foreground">
                <TranslatedText id={feature.descriptionKey}>{feature.description}</TranslatedText>
              </p>
            </div>
          ))}
        </div>
      </section>

      {/* CTA Section */}
      <section className="container py-20">
        <div className="bg-primary/5 rounded-2xl p-12 text-center">
          <h2 className="text-3xl font-bold mb-4">
            <TranslatedText id="cta-title">Ready to build intelligent robots?</TranslatedText>
          </h2>
          <p className="text-muted-foreground mb-8 max-w-xl mx-auto">
            <TranslatedText id="cta-description">Join thousands of engineers learning Physical AI. Start your journey today.</TranslatedText>
          </p>
          <div className="flex flex-col sm:flex-row items-center justify-center gap-4">
            <Link href="/signup">
              <Button size="lg">
                <TranslatedText id="cta-get-started">Get Started</TranslatedText>
              </Button>
            </Link>
            <Link href="https://github.com" target="_blank">
              <Button size="lg" variant="outline">
                <Github className="mr-2 h-5 w-5" />
                <TranslatedText id="cta-contribute">Contribute</TranslatedText>
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
                <span className="font-bold">
                  <TranslatedText id="footer-title">Physical AI</TranslatedText>
                </span>
              </div>
              <p className="text-sm text-muted-foreground">
                <TranslatedText id="footer-description">Building the future of robotics education.</TranslatedText>
              </p>
            </div>
            <div>
              <h4 className="font-semibold mb-4">
                <TranslatedText id="footer-book-title">Book</TranslatedText>
              </h4>
              <ul className="space-y-2 text-sm text-muted-foreground">
                <li>
                  <Link href="http://localhost:3001/" className="hover:text-foreground">
                    <TranslatedText id="footer-overview">Overview</TranslatedText>
                  </Link>
                </li>
                <li>
                  <Link href="http://localhost:3001/docs/module-1-ros2" className="hover:text-foreground">
                    <TranslatedText id="footer-modules">Modules</TranslatedText>
                  </Link>
                </li>
                <li>
                  <Link href="http://localhost:3001/docs/quickstart" className="hover:text-foreground">
                    <TranslatedText id="footer-quickstart">Quickstart</TranslatedText>
                  </Link>
                </li>
              </ul>
            </div>
            <div>
              <h4 className="font-semibold mb-4">
                <TranslatedText id="footer-community-title">Community</TranslatedText>
              </h4>
              <ul className="space-y-2 text-sm text-muted-foreground">
                <li>
                  <Link href="#" className="hover:text-foreground">
                    <TranslatedText id="footer-stackoverflow">Stack Overflow</TranslatedText>
                  </Link>
                </li>
                <li>
                  <Link href="#" className="hover:text-foreground">
                    <TranslatedText id="footer-discord">Discord</TranslatedText>
                  </Link>
                </li>
                <li>
                  <Link href="#" className="hover:text-foreground">
                    <TranslatedText id="footer-twitter">X (Twitter)</TranslatedText>
                  </Link>
                </li>
              </ul>
            </div>
            <div>
              <h4 className="font-semibold mb-4">
                <TranslatedText id="footer-social-title">Social</TranslatedText>
              </h4>
              <ul className="space-y-2 text-sm text-muted-foreground">
                <li>
                  <Link href="https://github.com" className="hover:text-foreground">
                    <TranslatedText id="footer-github">GitHub</TranslatedText>
                  </Link>
                </li>
                <li>
                  <Link href="#" className="hover:text-foreground">
                    <TranslatedText id="footer-youtube">YouTube</TranslatedText>
                  </Link>
                </li>
                <li>
                  <Link href="#" className="hover:text-foreground">
                    <TranslatedText id="footer-linkedin">LinkedIn</TranslatedText>
                  </Link>
                </li>
              </ul>
            </div>
          </div>
          <div className="border-t mt-8 pt-8 text-center text-sm text-muted-foreground">
            <p>
              <TranslatedText id="footer-copyright">&copy; 2025 Physical AI Textbook. All rights reserved.</TranslatedText>
            </p>
          </div>
        </div>
      </footer>
    </div>
  );
}
