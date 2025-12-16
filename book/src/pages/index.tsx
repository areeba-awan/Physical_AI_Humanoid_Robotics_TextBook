import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import useBaseUrl from '@docusaurus/useBaseUrl';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';
import Translate, {translate} from '@docusaurus/Translate';
import styles from './index.module.css';

const modules = {
  en: [
    {
      id: 1,
      title: 'The Robotic Nervous System',
      subtitle: 'ROS 2',
      description: 'Master the communication framework that powers modern robots. Learn nodes, topics, services, and actions.',
      color: '#3b82f6',
      href: 'docs/module-1-ros2/chapter-1-intro',
    },
    {
      id: 2,
      title: 'The Digital Twin',
      subtitle: 'Gazebo & Unity',
      description: 'Build and test robots in realistic simulated environments before deploying to hardware.',
      color: '#10b981',
      href: 'docs/module-2-simulation/chapter-1-intro',
    },
    {
      id: 3,
      title: 'The AI-Robot Brain',
      subtitle: 'NVIDIA Isaac',
      description: 'Leverage GPU-accelerated perception, navigation, and manipulation for intelligent robots.',
      color: '#8b5cf6',
      href: 'docs/module-3-isaac/chapter-1-intro',
    },
    {
      id: 4,
      title: 'Vision-Language-Action',
      subtitle: 'VLA Systems',
      description: 'Create robots that see, understand natural language, and act intelligently in the real world.',
      color: '#f59e0b',
      href: 'docs/module-4-vla/chapter-1-intro',
    },
  ],
  ur: [
    {
      id: 1,
      title: 'روبوٹک اعصابی نظام',
      subtitle: 'آر او ایس 2',
      description: 'جدید روبوٹس کو طاقت دینے والے کمیونیکیشن فریم ورک میں مہارت حاصل کریں۔ نوڈز، ٹاپکس، سروسز، اور ایکشنز سیکھیں۔',
      color: '#3b82f6',
      href: 'docs/module-1-ros2/chapter-1-intro',
    },
    {
      id: 2,
      title: 'ڈیجیٹل ٹوئن',
      subtitle: 'گزیبو اور یونٹی',
      description: 'حقیقت پسندانہ سمیولیٹڈ ماحول میں روبوٹس بنائیں اور ٹیسٹ کریں۔',
      color: '#10b981',
      href: 'docs/module-2-simulation/chapter-1-intro',
    },
    {
      id: 3,
      title: 'اے آئی روبوٹ دماغ',
      subtitle: 'این ویڈیا آئزک',
      description: 'ذہین روبوٹس کے لیے جی پی یو سے تیز کردہ ادراک، نیویگیشن، اور ہیرا پھیری کا فائدہ اٹھائیں۔',
      color: '#8b5cf6',
      href: 'docs/module-3-isaac/chapter-1-intro',
    },
    {
      id: 4,
      title: 'بصارت-زبان-عمل',
      subtitle: 'وی ایل اے سسٹمز',
      description: 'ایسے روبوٹس بنائیں جو دیکھتے ہیں، قدرتی زبان سمجھتے ہیں، اور حقیقی دنیا میں ذہانت سے عمل کرتے ہیں۔',
      color: '#f59e0b',
      href: 'docs/module-4-vla/chapter-1-intro',
    },
  ],
};

const quickstartCards = {
  en: [
    {
      title: 'Quickstart Guide',
      description: 'One-command setup to run a simulation locally',
      href: 'docs/quickstart',
      emoji: '🚀',
    },
    {
      title: 'Simulation Templates',
      description: 'Pre-built Gazebo and Unity scenes ready to customize',
      href: 'docs/module-2-simulation/chapter-2-gazebo',
      emoji: '🎮',
    },
    {
      title: 'Capstone Recipes',
      description: 'End-to-end demonstrations combining vision, language, and action',
      href: 'docs/module-4-vla/chapter-6-capstone',
      emoji: '🏆',
    },
  ],
  ur: [
    {
      title: 'کوئیک اسٹارٹ گائیڈ',
      description: 'مقامی طور پر سمیولیشن چلانے کے لیے ایک کمانڈ سیٹ اپ',
      href: 'docs/quickstart',
      emoji: '🚀',
    },
    {
      title: 'سمیولیشن ٹیمپلیٹس',
      description: 'اپنی مرضی کے مطابق بنانے کے لیے تیار گزیبو اور یونٹی سینز',
      href: 'docs/module-2-simulation/chapter-2-gazebo',
      emoji: '🎮',
    },
    {
      title: 'کیپسٹون ریسیپیز',
      description: 'بصارت، زبان اور عمل کو یکجا کرنے والے اینڈ ٹو اینڈ مظاہرے',
      href: 'docs/module-4-vla/chapter-6-capstone',
      emoji: '🏆',
    },
  ],
};

const features = {
  en: [
    {
      title: 'Hands-on Labs',
      description: 'Real code, real robots, real results. Every chapter includes practical exercises.',
      emoji: '🔬',
    },
    {
      title: 'Interactive Visualizations',
      description: 'High-fidelity digital twins and 3D visualizations for complex concepts.',
      emoji: '📊',
    },
    {
      title: 'VLA Integration',
      description: 'Cutting-edge Vision-Language-Action pipelines for intelligent robotics.',
      emoji: '🤖',
    },
  ],
  ur: [
    {
      title: 'عملی لیبز',
      description: 'حقیقی کوڈ، حقیقی روبوٹس، حقیقی نتائج۔ ہر باب میں عملی مشقیں شامل ہیں۔',
      emoji: '🔬',
    },
    {
      title: 'انٹرایکٹو ویژولائزیشنز',
      description: 'پیچیدہ تصورات کے لیے اعلیٰ معیار کے ڈیجیٹل ٹوئنز اور 3D ویژولائزیشنز۔',
      emoji: '📊',
    },
    {
      title: 'وی ایل اے انٹیگریشن',
      description: 'ذہین روبوٹکس کے لیے جدید ترین بصارت-زبان-عمل پائپ لائنز۔',
      emoji: '🤖',
    },
  ],
};

const translations = {
  en: {
    heroDescription: 'Hands-on labs and ready-to-run examples for building intelligent humanoid robots',
    getStarted: 'Get Started',
    exploreChapters: 'Explore Chapters',
    courseModules: 'Course Modules',
    keyFeatures: 'Key Features',
    startModule: 'Start Module →',
    module: 'Module',
    ctaTitle: 'Ready to build intelligent robots?',
    ctaDescription: 'Start your journey into Physical AI today. Join engineers worldwide learning to build the next generation of humanoid robots.',
  },
  ur: {
    heroDescription: 'ذہین ہیومنائیڈ روبوٹس بنانے کے لیے عملی لیبز اور چلانے کے لیے تیار مثالیں',
    getStarted: 'شروع کریں',
    exploreChapters: 'ابواب دریافت کریں',
    courseModules: 'کورس ماڈیولز',
    keyFeatures: 'اہم خصوصیات',
    startModule: '← ماڈیول شروع کریں',
    module: 'ماڈیول',
    ctaTitle: 'ذہین روبوٹس بنانے کے لیے تیار ہیں؟',
    ctaDescription: 'آج ہی فزیکل اے آئی میں اپنا سفر شروع کریں۔ دنیا بھر کے انجینئرز کے ساتھ مل کر اگلی نسل کے ہیومنائیڈ روبوٹس بنانا سیکھیں۔',
  },
};

function HomepageHeader() {
  const {siteConfig, i18n} = useDocusaurusContext();
  const currentLocale = i18n.currentLocale as 'en' | 'ur';
  const t = translations[currentLocale] || translations.en;

  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <Heading as="h1" className="hero__title">
          {siteConfig.title}
        </Heading>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <p className={styles.heroDescription}>
          {t.heroDescription}
        </p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to={useBaseUrl('docs/intro')}>
            {t.getStarted}
          </Link>
          <Link
            className="button button--outline button--lg button--secondary"
            to={useBaseUrl('docs/intro')}>
            {t.exploreChapters}
          </Link>
        </div>
      </div>
    </header>
  );
}

function QuickstartCard({card}: {card: {title: string; description: string; href: string; emoji: string}}) {
  return (
    <div className="col col--4">
      <Link to={useBaseUrl(card.href)} className={styles.quickstartCard}>
        <div className={styles.quickstartEmoji}>{card.emoji}</div>
        <h3>{card.title}</h3>
        <p>{card.description}</p>
      </Link>
    </div>
  );
}

function QuickstartSection() {
  const {i18n} = useDocusaurusContext();
  const currentLocale = i18n.currentLocale as 'en' | 'ur';
  const cards = quickstartCards[currentLocale] || quickstartCards.en;

  return (
    <section className={styles.quickstart}>
      <div className="container">
        <div className="row">
          {cards.map((card, idx) => (
            <QuickstartCard key={idx} card={card} />
          ))}
        </div>
      </div>
    </section>
  );
}

function ModuleCard({module, t}: {module: {id: number; title: string; subtitle: string; description: string; color: string; href: string}; t: typeof translations.en}) {
  return (
    <div className="col col--6">
      <Link to={useBaseUrl(module.href)} className={styles.moduleCard}>
        <div className={styles.moduleHeader}>
          <div
            className={styles.moduleIcon}
            style={{ backgroundColor: module.color }}
          >
            {module.id}
          </div>
          <span className={styles.moduleNumber}>{t.module} {module.id}</span>
        </div>
        <h3 className={styles.moduleTitle}>{module.title}</h3>
        <p className={styles.moduleSubtitle}>{module.subtitle}</p>
        <p className={styles.moduleDescription}>{module.description}</p>
        <div className={styles.moduleLink}>
          {t.startModule}
        </div>
      </Link>
    </div>
  );
}

function ModulesSection() {
  const {i18n} = useDocusaurusContext();
  const currentLocale = i18n.currentLocale as 'en' | 'ur';
  const modulesList = modules[currentLocale] || modules.en;
  const t = translations[currentLocale] || translations.en;

  return (
    <section className={styles.modules}>
      <div className="container">
        <Heading as="h2" className={styles.sectionTitle}>
          {t.courseModules}
        </Heading>
        <div className="row">
          {modulesList.map((module) => (
            <ModuleCard key={module.id} module={module} t={t} />
          ))}
        </div>
      </div>
    </section>
  );
}

function FeaturesSection() {
  const {i18n} = useDocusaurusContext();
  const currentLocale = i18n.currentLocale as 'en' | 'ur';
  const featuresList = features[currentLocale] || features.en;
  const t = translations[currentLocale] || translations.en;

  return (
    <section className={styles.features}>
      <div className="container">
        <Heading as="h2" className={styles.sectionTitle}>
          {t.keyFeatures}
        </Heading>
        <div className="row">
          {featuresList.map((feature, idx) => (
            <div key={idx} className="col col--4">
              <div className={styles.featureCard}>
                <div className={styles.featureEmoji}>{feature.emoji}</div>
                <h3>{feature.title}</h3>
                <p>{feature.description}</p>
              </div>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

function CTASection() {
  const {i18n} = useDocusaurusContext();
  const currentLocale = i18n.currentLocale as 'en' | 'ur';
  const t = translations[currentLocale] || translations.en;

  return (
    <section className={styles.cta}>
      <div className="container">
        <div className={styles.ctaContent}>
          <Heading as="h2">{t.ctaTitle}</Heading>
          <p>{t.ctaDescription}</p>
          <div className={styles.ctaButtons}>
            <Link
              className="button button--primary button--lg"
              to={useBaseUrl('docs/intro')}>
              {t.getStarted}
            </Link>
            <Link
              className="button button--outline button--lg"
              to={useBaseUrl('docs/module-1-ros2/chapter-1-intro')}>
              {t.exploreChapters}
            </Link>
          </div>
        </div>
      </div>
    </section>
  );
}

export default function Home(): JSX.Element {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`${siteConfig.title}`}
      description="A comprehensive guide to Physical AI and Humanoid Robotics. Learn ROS 2, Gazebo, NVIDIA Isaac, and Vision-Language-Action systems.">
      <HomepageHeader />
      <main>
        <QuickstartSection />
        <ModulesSection />
        <FeaturesSection />
        <CTASection />
      </main>
    </Layout>
  );
}
