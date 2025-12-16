"use client";

import { useEffect } from "react";
import { useRouter } from "next/navigation";
import Link from "next/link";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { BackgroundQuestions } from "@/components/auth/background-questions";
import { useAuth } from "@/hooks/use-auth";

export default function OnboardingPage() {
  const router = useRouter();
  const { user, isLoading } = useAuth();

  useEffect(() => {
    if (!isLoading && !user) {
      router.push('/signin');
    }
  }, [user, isLoading, router]);

  if (isLoading) {
    return (
      <div className="flex items-center justify-center min-h-[400px]">
        <div className="animate-spin rounded-full h-8 w-8 border-b-2 border-primary" />
      </div>
    );
  }

  return (
    <Card className="w-full max-w-2xl">
      <CardHeader className="space-y-1">
        <div className="flex items-center justify-between">
          <div>
            <CardTitle className="text-2xl font-bold">Tell us about your background</CardTitle>
            <CardDescription className="mt-1">
              This helps us personalize your learning experience
            </CardDescription>
          </div>
          <Link href="/dashboard">
            <Button variant="ghost" size="sm">
              Skip
            </Button>
          </Link>
        </div>
      </CardHeader>
      <CardContent>
        <BackgroundQuestions />
      </CardContent>
    </Card>
  );
}
