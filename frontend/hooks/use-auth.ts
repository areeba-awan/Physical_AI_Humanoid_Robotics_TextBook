"use client";

import { useCallback, useEffect } from 'react';
import { useRouter } from 'next/navigation';
import { useAuthStore } from '@/store/auth-store';
import { authApi, profileApi, ApiError } from '@/lib/api';
import { toast } from '@/hooks/use-toast';
import type { SignupRequest, SigninRequest, BackgroundData } from '@/types/auth';

export function useAuth() {
  const router = useRouter();
  const {
    user,
    session,
    profile,
    isLoading,
    error,
    setUser,
    setSession,
    setProfile,
    setLoading,
    setError,
    logout: storeLogout,
  } = useAuthStore();

  // Check session on mount
  useEffect(() => {
    const checkSession = async () => {
      if (!session?.token) {
        setLoading(false);
        return;
      }

      try {
        const data = await authApi.getSession(session.token);
        setUser(data.user);
        setSession(data.session);
      } catch (error) {
        // Session invalid, clear store
        storeLogout();
      } finally {
        setLoading(false);
      }
    };

    checkSession();
  }, []);

  const signup = useCallback(async (data: SignupRequest) => {
    setLoading(true);
    setError(null);

    try {
      const response: any = await authApi.signup(data);
      setUser(response.user);
      setSession(response.session);

      toast({
        title: "Account created!",
        description: "Welcome to Physical AI Textbook.",
      });

      // Redirect to onboarding
      router.push('/onboarding');

      return response;
    } catch (error) {
      const message = error instanceof ApiError
        ? error.data?.error?.message || error.message
        : 'Failed to create account';
      setError(message);
      toast({
        title: "Error",
        description: message,
        variant: "destructive",
      });
      throw error;
    } finally {
      setLoading(false);
    }
  }, [router, setUser, setSession, setError, setLoading]);

  const signin = useCallback(async (data: SigninRequest) => {
    setLoading(true);
    setError(null);

    try {
      const response: any = await authApi.signin(data);
      setUser(response.user);
      setSession(response.session);

      toast({
        title: "Welcome back!",
        description: "Successfully signed in.",
      });

      // Check if user needs onboarding
      if (response.requiresOnboarding) {
        router.push('/onboarding');
      } else {
        router.push('/dashboard');
      }

      return response;
    } catch (error) {
      const message = error instanceof ApiError
        ? error.data?.error?.message || error.message
        : 'Failed to sign in';
      setError(message);
      toast({
        title: "Error",
        description: message,
        variant: "destructive",
      });
      throw error;
    } finally {
      setLoading(false);
    }
  }, [router, setUser, setSession, setError, setLoading]);

  const logout = useCallback(async () => {
    try {
      if (session?.token) {
        await authApi.signout(session.token);
      }
    } catch (error) {
      // Ignore errors during signout
    } finally {
      storeLogout();
      router.push('/');
      toast({
        title: "Signed out",
        description: "You have been signed out.",
      });
    }
  }, [session, storeLogout, router]);

  const updateBackground = useCallback(async (data: BackgroundData) => {
    if (!session?.token) {
      throw new Error('Not authenticated');
    }

    setLoading(true);

    try {
      const response: any = await profileApi.updateBackground(session.token, data);
      setProfile(response.profile);

      toast({
        title: "Profile updated",
        description: "Your background information has been saved.",
      });

      router.push('/dashboard');

      return response;
    } catch (error) {
      const message = error instanceof ApiError
        ? error.data?.error?.message || error.message
        : 'Failed to update profile';
      toast({
        title: "Error",
        description: message,
        variant: "destructive",
      });
      throw error;
    } finally {
      setLoading(false);
    }
  }, [session, setProfile, setLoading, router]);

  const fetchProfile = useCallback(async () => {
    if (!session?.token) return null;

    try {
      const response: any = await profileApi.get(session.token);
      setProfile(response);
      return response;
    } catch (error) {
      console.error('Failed to fetch profile:', error);
      return null;
    }
  }, [session, setProfile]);

  return {
    user,
    session,
    profile,
    isLoading,
    error,
    isAuthenticated: !!user && !!session,
    signup,
    signin,
    logout,
    updateBackground,
    fetchProfile,
  };
}
