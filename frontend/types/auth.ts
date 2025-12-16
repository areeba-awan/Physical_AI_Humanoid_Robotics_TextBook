export interface User {
  id: string;
  email: string;
  name: string;
  emailVerified: boolean;
  createdAt: string;
  updatedAt: string;
}

export interface Session {
  token: string;
  expiresAt: string;
}

export interface UserProfile {
  id: string;
  userId: string;

  // Software Background
  softwareExperience: 'none' | 'beginner' | 'intermediate' | 'advanced';
  programmingLanguages: string[];
  rosExperience: boolean;
  linuxExperience: 'none' | 'basic' | 'comfortable' | 'expert';

  // Hardware Background
  hardwareExperience: 'none' | 'hobbyist' | 'professional';
  roboticsExperience: boolean;
  previousProjects: string;

  // Equipment
  hasGpuWorkstation: boolean;
  gpuModel?: string;
  hasJetsonKit: boolean;
  jetsonModel?: string;
  hasRobotHardware: boolean;
  robotDescription?: string;

  // Preferences
  preferredLanguage: 'en' | 'ur';
  theme: 'light' | 'dark' | 'system';
  notificationsEnabled: boolean;

  // Progress
  currentChapter?: string;
  completedChapters: string[];

  createdAt: string;
  updatedAt: string;
}

export interface SignupRequest {
  email: string;
  password: string;
  name: string;
}

export interface SigninRequest {
  email: string;
  password: string;
}

export interface AuthResponse {
  user: User;
  session: Session;
  requiresOnboarding?: boolean;
}

export interface BackgroundData {
  softwareExperience: 'none' | 'beginner' | 'intermediate' | 'advanced';
  programmingLanguages: string[];
  rosExperience: boolean;
  linuxExperience: 'none' | 'basic' | 'comfortable' | 'expert';
  hardwareExperience: 'none' | 'hobbyist' | 'professional';
  roboticsExperience: boolean;
  previousProjects: string;
  hasGpuWorkstation: boolean;
  gpuModel?: string;
  hasJetsonKit: boolean;
  jetsonModel?: string;
  hasRobotHardware: boolean;
  robotDescription?: string;
}
