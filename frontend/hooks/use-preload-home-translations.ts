import { useTranslation } from '@/hooks/use-translation';

/**
 * Preloads translations for static content on the homepage
 */
export const usePreloadHomeTranslations = () => {
  const { preloadTranslations } = useTranslation();

  const preloadHomeContent = () => {
    const homeTexts = [
      // Navigation
      { text: 'Physical AI', key: 'nav-title' },
      { text: 'Modules', key: 'nav-modules' },
      { text: 'Book Overview', key: 'nav-book-overview' },
      { text: 'Dashboard', key: 'dashboard-btn' },
      { text: 'Sign In', key: 'sign-in-btn' },
      { text: 'Get Started', key: 'get-started-btn' },
      
      // Hero section
      { text: 'Physical AI &', key: 'hero-title' },
      { text: 'Humanoid Robotics', key: 'hero-subtitle' },
      { text: 'A Comprehensive Guide to Embodied AI', key: 'hero-description' },
      { text: 'Hands-on labs and ready-to-run examples for building intelligent robots', key: 'hero-tagline' },
      { text: 'Get Started', key: 'hero-get-started' },
      { text: 'View Book', key: 'hero-view-book' },
      
      // Quick start cards
      { text: 'Quickstart Guide', key: 'quickstart-title' },
      { text: 'Set up your development environment in minutes', key: 'quickstart-description' },
      { text: 'Simulation Templates', key: 'templates-title' },
      { text: 'Pre-built environments to jumpstart your projects', key: 'templates-description' },
      { text: 'Capstone Recipes', key: 'capstone-title' },
      { text: 'End-to-end project guides for real applications', key: 'capstone-description' },
      
      // Modules
      { text: 'Course Modules', key: 'modules-section-title' },
      { text: 'The Robotic Nervous System', key: 'module1-title' },
      { text: 'ROS 2', key: 'module1-subtitle' },
      { text: 'Master the communication framework that powers modern robots', key: 'module1-description' },
      { text: 'The Digital Twin', key: 'module2-title' },
      { text: 'Gazebo & Unity', key: 'module2-subtitle' },
      { text: 'Build and test robots in realistic simulated environments', key: 'module2-description' },
      { text: 'The AI-Robot Brain', key: 'module3-title' },
      { text: 'NVIDIA Isaac', key: 'module3-subtitle' },
      { text: 'Leverage GPU-accelerated perception and manipulation', key: 'module3-description' },
      { text: 'Vision-Language-Action', key: 'module4-title' },
      { text: 'VLA Systems', key: 'module4-subtitle' },
      { text: 'Create robots that see, understand, and act intelligently', key: 'module4-description' },
      { text: 'Module 1', key: 'module-1-label' },
      { text: 'Module 2', key: 'module-2-label' },
      { text: 'Module 3', key: 'module-3-label' },
      { text: 'Module 4', key: 'module-4-label' },
      { text: 'Start Module', key: 'module-1-btn' },
      { text: 'Start Module', key: 'module-2-btn' },
      { text: 'Start Module', key: 'module-3-btn' },
      { text: 'Start Module', key: 'module-4-btn' },
      
      // Features
      { text: 'Key Features', key: 'features-section-title' },
      { text: 'Hands-on Labs', key: 'feature1-title' },
      { text: 'Real code, real robots, real results', key: 'feature1-description' },
      { text: 'Interactive Visualizations', key: 'feature2-title' },
      { text: 'See concepts come to life in 3D', key: 'feature2-description' },
      { text: 'VLA Integrations', key: 'feature3-title' },
      { text: 'Cutting-edge AI for physical systems', key: 'feature3-description' },
      
      // CTA section
      { text: 'Ready to build intelligent robots?', key: 'cta-title' },
      { text: 'Join thousands of engineers learning Physical AI. Start your journey today.', key: 'cta-description' },
      { text: 'Get Started', key: 'cta-get-started' },
      { text: 'Contribute', key: 'cta-contribute' },
      
      // Footer
      { text: 'Building the future of robotics education.', key: 'footer-description' },
      { text: 'Book', key: 'footer-book-title' },
      { text: 'Overview', key: 'footer-overview' },
      { text: 'Modules', key: 'footer-modules' },
      { text: 'Quickstart', key: 'footer-quickstart' },
      { text: 'Community', key: 'footer-community-title' },
      { text: 'Stack Overflow', key: 'footer-stackoverflow' },
      { text: 'Discord', key: 'footer-discord' },
      { text: 'X (Twitter)', key: 'footer-twitter' },
      { text: 'Social', key: 'footer-social-title' },
      { text: 'GitHub', key: 'footer-github' },
      { text: 'YouTube', key: 'footer-youtube' },
      { text: 'LinkedIn', key: 'footer-linkedin' },
      { text: '© 2025 Physical AI Textbook. All rights reserved.', key: 'footer-copyright' },
    ];

    return preloadTranslations(homeTexts);
  };

  return { preloadHomeContent };
};