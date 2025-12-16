"use client";

import { useEffect, useState } from "react";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import { useAuth } from "@/hooks/use-auth";
import { useToast } from "@/hooks/use-toast";
import { User, Settings, Mail, Laptop, Code, Monitor } from "lucide-react";

const softwareLevels = [
  { value: "none", label: "None - Completely new to programming" },
  { value: "beginner", label: "Beginner - Basic understanding" },
  { value: "intermediate", label: "Intermediate - Comfortable coding" },
  { value: "advanced", label: "Advanced - Professional developer" },
];

const linuxLevels = [
  { value: "none", label: "None" },
  { value: "basic", label: "Basic" },
  { value: "comfortable", label: "Comfortable" },
  { value: "expert", label: "Expert" },
];

export default function ProfilePage() {
  const { user, profile, fetchProfile, updateProfile } = useAuth();
  const { toast } = useToast();
  const [isEditing, setIsEditing] = useState(false);
  const [formData, setFormData] = useState({
    softwareExperience: "",
    programmingLanguages: [] as string[],
    rosExperience: false,
    linuxExperience: "",
    hardwareExperience: "",
    hasGpuWorkstation: false,
    gpuModel: "",
    hasJetsonKit: false,
    jetsonModel: "",
    hasRobotHardware: false,
    robotDescription: "",
    preferredLanguage: "en",
    theme: "system",
  });

  useEffect(() => {
    fetchProfile();
  }, [fetchProfile]);

  useEffect(() => {
    if (profile) {
      setFormData({
        softwareExperience: profile.softwareExperience || "beginner",
        programmingLanguages: profile.programmingLanguages || [],
        rosExperience: profile.rosExperience || false,
        linuxExperience: profile.linuxExperience || "basic",
        hardwareExperience: profile.hardwareExperience || "none",
        hasGpuWorkstation: profile.hasGpuWorkstation || false,
        gpuModel: profile.gpuModel || "",
        hasJetsonKit: profile.hasJetsonKit || false,
        jetsonModel: profile.jetsonModel || "",
        hasRobotHardware: profile.hasRobotHardware || false,
        robotDescription: profile.robotDescription || "",
        preferredLanguage: profile.preferredLanguage || "en",
        theme: profile.theme || "system",
      });
    }
  }, [profile]);

  const handleSave = async () => {
    try {
      await updateProfile(formData);
      toast({
        title: "Profile updated",
        description: "Your profile has been saved successfully.",
      });
      setIsEditing(false);
    } catch (error) {
      toast({
        title: "Error",
        description: "Failed to update profile. Please try again.",
        variant: "destructive",
      });
    }
  };

  const toggleLanguage = (lang: string) => {
    setFormData((prev) => ({
      ...prev,
      programmingLanguages: prev.programmingLanguages.includes(lang)
        ? prev.programmingLanguages.filter((l) => l !== lang)
        : [...prev.programmingLanguages, lang],
    }));
  };

  return (
    <div className="space-y-8">
      <div className="flex items-center justify-between">
        <div>
          <h1 className="text-3xl font-bold tracking-tight">Profile</h1>
          <p className="text-muted-foreground mt-1">
            Manage your account and learning preferences
          </p>
        </div>
        <Button
          onClick={() => (isEditing ? handleSave() : setIsEditing(true))}
        >
          {isEditing ? "Save Changes" : "Edit Profile"}
        </Button>
      </div>

      {/* Account Info */}
      <Card>
        <CardHeader>
          <CardTitle className="flex items-center gap-2">
            <User className="h-5 w-5" />
            Account Information
          </CardTitle>
        </CardHeader>
        <CardContent className="space-y-4">
          <div className="grid gap-4 md:grid-cols-2">
            <div>
              <Label>Name</Label>
              <Input value={user?.name || ""} disabled />
            </div>
            <div>
              <Label>Email</Label>
              <Input value={user?.email || ""} disabled />
            </div>
          </div>
        </CardContent>
      </Card>

      {/* Software Background */}
      <Card>
        <CardHeader>
          <CardTitle className="flex items-center gap-2">
            <Code className="h-5 w-5" />
            Software Background
          </CardTitle>
          <CardDescription>
            This helps us personalize your learning experience
          </CardDescription>
        </CardHeader>
        <CardContent className="space-y-4">
          <div>
            <Label>Programming Experience</Label>
            <select
              className="w-full mt-1 p-2 border rounded-md bg-background"
              value={formData.softwareExperience}
              onChange={(e) =>
                setFormData({ ...formData, softwareExperience: e.target.value })
              }
              disabled={!isEditing}
            >
              {softwareLevels.map((level) => (
                <option key={level.value} value={level.value}>
                  {level.label}
                </option>
              ))}
            </select>
          </div>

          <div>
            <Label>Programming Languages</Label>
            <div className="flex flex-wrap gap-2 mt-2">
              {["Python", "C++", "JavaScript", "Rust", "Go", "Java"].map(
                (lang) => (
                  <button
                    key={lang}
                    onClick={() => isEditing && toggleLanguage(lang)}
                    className={`px-3 py-1 rounded-full text-sm border transition-colors ${
                      formData.programmingLanguages.includes(lang)
                        ? "bg-primary text-primary-foreground border-primary"
                        : "bg-background border-border hover:border-primary"
                    } ${!isEditing && "cursor-not-allowed opacity-70"}`}
                    disabled={!isEditing}
                  >
                    {lang}
                  </button>
                )
              )}
            </div>
          </div>

          <div className="grid gap-4 md:grid-cols-2">
            <div>
              <Label>ROS Experience</Label>
              <div className="flex gap-4 mt-2">
                <label className="flex items-center gap-2">
                  <input
                    type="radio"
                    checked={formData.rosExperience}
                    onChange={() =>
                      setFormData({ ...formData, rosExperience: true })
                    }
                    disabled={!isEditing}
                  />
                  Yes
                </label>
                <label className="flex items-center gap-2">
                  <input
                    type="radio"
                    checked={!formData.rosExperience}
                    onChange={() =>
                      setFormData({ ...formData, rosExperience: false })
                    }
                    disabled={!isEditing}
                  />
                  No
                </label>
              </div>
            </div>

            <div>
              <Label>Linux Experience</Label>
              <select
                className="w-full mt-1 p-2 border rounded-md bg-background"
                value={formData.linuxExperience}
                onChange={(e) =>
                  setFormData({ ...formData, linuxExperience: e.target.value })
                }
                disabled={!isEditing}
              >
                {linuxLevels.map((level) => (
                  <option key={level.value} value={level.value}>
                    {level.label}
                  </option>
                ))}
              </select>
            </div>
          </div>
        </CardContent>
      </Card>

      {/* Equipment */}
      <Card>
        <CardHeader>
          <CardTitle className="flex items-center gap-2">
            <Laptop className="h-5 w-5" />
            Equipment
          </CardTitle>
          <CardDescription>
            Let us know what hardware you have available
          </CardDescription>
        </CardHeader>
        <CardContent className="space-y-4">
          <div className="flex items-center gap-4">
            <input
              type="checkbox"
              checked={formData.hasGpuWorkstation}
              onChange={(e) =>
                setFormData({ ...formData, hasGpuWorkstation: e.target.checked })
              }
              disabled={!isEditing}
              className="w-4 h-4"
            />
            <div className="flex-1">
              <Label>GPU Workstation</Label>
              {formData.hasGpuWorkstation && (
                <Input
                  placeholder="e.g., RTX 3080"
                  value={formData.gpuModel}
                  onChange={(e) =>
                    setFormData({ ...formData, gpuModel: e.target.value })
                  }
                  disabled={!isEditing}
                  className="mt-2"
                />
              )}
            </div>
          </div>

          <div className="flex items-center gap-4">
            <input
              type="checkbox"
              checked={formData.hasJetsonKit}
              onChange={(e) =>
                setFormData({ ...formData, hasJetsonKit: e.target.checked })
              }
              disabled={!isEditing}
              className="w-4 h-4"
            />
            <div className="flex-1">
              <Label>NVIDIA Jetson Kit</Label>
              {formData.hasJetsonKit && (
                <Input
                  placeholder="e.g., Jetson Orin Nano"
                  value={formData.jetsonModel}
                  onChange={(e) =>
                    setFormData({ ...formData, jetsonModel: e.target.value })
                  }
                  disabled={!isEditing}
                  className="mt-2"
                />
              )}
            </div>
          </div>

          <div className="flex items-center gap-4">
            <input
              type="checkbox"
              checked={formData.hasRobotHardware}
              onChange={(e) =>
                setFormData({ ...formData, hasRobotHardware: e.target.checked })
              }
              disabled={!isEditing}
              className="w-4 h-4"
            />
            <div className="flex-1">
              <Label>Robot Hardware</Label>
              {formData.hasRobotHardware && (
                <Input
                  placeholder="Describe your robot..."
                  value={formData.robotDescription}
                  onChange={(e) =>
                    setFormData({ ...formData, robotDescription: e.target.value })
                  }
                  disabled={!isEditing}
                  className="mt-2"
                />
              )}
            </div>
          </div>
        </CardContent>
      </Card>

      {/* Preferences */}
      <Card>
        <CardHeader>
          <CardTitle className="flex items-center gap-2">
            <Settings className="h-5 w-5" />
            Preferences
          </CardTitle>
        </CardHeader>
        <CardContent className="space-y-4">
          <div className="grid gap-4 md:grid-cols-2">
            <div>
              <Label>Preferred Language</Label>
              <select
                className="w-full mt-1 p-2 border rounded-md bg-background"
                value={formData.preferredLanguage}
                onChange={(e) =>
                  setFormData({ ...formData, preferredLanguage: e.target.value })
                }
                disabled={!isEditing}
              >
                <option value="en">English</option>
                <option value="ur">Urdu (اردو)</option>
              </select>
            </div>

            <div>
              <Label>Theme</Label>
              <select
                className="w-full mt-1 p-2 border rounded-md bg-background"
                value={formData.theme}
                onChange={(e) =>
                  setFormData({ ...formData, theme: e.target.value })
                }
                disabled={!isEditing}
              >
                <option value="light">Light</option>
                <option value="dark">Dark</option>
                <option value="system">System</option>
              </select>
            </div>
          </div>
        </CardContent>
      </Card>

      {isEditing && (
        <div className="flex gap-4 justify-end">
          <Button variant="outline" onClick={() => setIsEditing(false)}>
            Cancel
          </Button>
          <Button onClick={handleSave}>Save Changes</Button>
        </div>
      )}
    </div>
  );
}
