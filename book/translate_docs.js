const fs = require('fs');
const path = require('path');

// Define file paths
const SOURCE_DIR = './docs';
const URDU_DIR = './i18n/ur/docusaurus-plugin-content-docs/current';

// Mapping of English terms to Urdu translations
const TRANSLATION_MAP = {
  'Introduction': 'تعارف',
  'Quickstart Guide': 'فوراً شروع کریں گائیڈ',
  'Chapter': 'باب',
  'Learning Objectives': 'سیکھنے کے مقاصد',
  'Prerequisites': 'شرائطِ لازمہ',
  'What is ROS 2?': 'ROS 2 کیا ہے؟',
  'Setting Up Your Workspace': 'اپنا ماحول تیار کریں',
  'Your First ROS 2 Program': 'آپ کا پہلا ROS 2 پروگرام',
  'ROS 2 Command Line Tools': 'ROS 2 کمانڈ لائن ٹولز',
  'Hands-on Lab': 'ہاتھ سے کام کرنے والی لیب',
  'Knowledge Check': 'نالج چیک',
  'Summary': 'خلاصہ',
  'Further Reading': 'مزید پڑھائی',
  'Next Steps': 'اگلے اقدامات',
  'Introduction to ROS 2': 'ROS 2 کا تعارف',
  'Understanding the Robot Operating System 2 architecture and philosophy': 'روبوٹ آپریٹنگ سسٹم 2 کے معماری اور فلسفے کو سمجھنا',
  'Robot Operating System 2': 'روبوٹ آپریٹنگ سسٹم 2',
  'Essential commands you\'ll use daily': 'ضروری کمانڈز جو آپ روزانہ استعمال کریں گے',
  'Simulation': 'سیمیولیشن',
  'The Digital Twin': 'ڈیجیٹل ٹوئن',
  'AI-Robot Brain': 'AI-روبوٹ دماغ',
  'Vision-Language-Action': 'وژن-زبان-کارروائی',
  'Nodes, Topics, and Services': 'نوڈز، ٹاپکس، اور سروسز',
  'Actions and Parameters': 'ایکشنز اور پیرامیٹر',
  'Launch Files and Parameters': 'لانچ فائلز اور پیرامیٹر',
  'Creating Custom Packages': 'اپنی مرضی کے پیکجز تیار کرنا',
  'Lab Exercise': 'لیب مشق',
  'Gazebo': 'گیزیبو',
  'Unity': 'یونٹی',
  'NVIDIA Isaac': 'این ویڈیا ایزیک',
  'VLA Systems': 'VLA نظام',
  'Personalize': 'ذاتی نوعیت کے مطابق',
  'Translate': 'ترجمہ',
  'AI Assistant': 'AI اسسٹنٹ',
  'Get Started': 'شروع کریں',
  'Book Overview': 'کتاب کا جائزہ',
  'Chapters': 'ابواب',
  'Community': 'کمیونٹی',
  'More': 'مزید',
  'Overview': 'جائزہ',
  'GitHub': 'گیتھب',
  'Stack Overflow': 'سٹیک اOVERFLOW',
  'Discord': 'ڈسکارڈ',
  'X (Twitter)': 'ایکس (ٹویٹر)',
  'ROS 2': 'ROS 2',
  'Module': 'مواد',
  'The Robotic Nervous System': 'روبوٹکس کا عصبی نظام',
  'Master the communication framework that powers modern robots': 'جدید روبوٹس کو طاقت دینے والے مواصلاتی ڈھانچے کو مسلط کریں',
  'Build and test robots in realistic simulated environments': ' حقیقت پسندانہ تنصیبات میں روبوٹس بنائیں اور ان کی جانچ کریں',
  'Leverage GPU-accelerated perception and manipulation': ' GPU کی مدد سے تیز کارروائی اور ہیرا پھیری کا فائدہ اٹھائیں',
  'Create robots that see, understand, and act intelligently': 'ایسے روبوٹس بنائیں جو دیکھ سکیں، سمجھ سکیں، اور ذہین انداز میں کام کر سکیں',
  'Real-time': 'ریل ٹائم',
  'Security': 'سیکیورٹی',
  'Platforms': 'پلیٹ فارم',
  'Middleware': 'مڈل ویئر',
  'Multi-robot': 'متعدد روبوٹ',
  'Data Distribution Service': 'ڈیٹا تقسیم کی سروس',
  'Communication Infrastructure': 'مواصلاتی انفراسٹرکچر',
  'Hardware Abstraction': 'ہارڈ ویئر امتصال',
  'Quality of Service': 'سروس کی معیار',
  'Type Safety': 'ٹائپ سیفٹی',
  'Automatic Discovery': 'خودکار دریافت',
  'Publishers and Subscribers': 'شائع کنندہ اور مسیحین',
  'Services and Actions': 'سروسز اور ایکشنز',
  'Parameters and Launch Files': 'پیرامیٹر اور لانچ فائلز',
  'Package.xml and Setup': 'Package.xml اور ترتیب',
  'Simulation Architecture': 'سیمیولیشن کی معماری',
  'Physics Engines': 'طبیعات انجن',
  'Sensor Integration': 'سینسر انضمام',
  'Isaac Sim': 'ایزیک سیم',
  'Omniverse': 'اومنی ورس',
  'ROS Bridge': 'ROS برج',
  'Perception Pipelines': 'ادراک کے پائپ لائنز',
  'Navigation and Manipulation': 'نیویگیشن اور ہیرا پھیری',
  'Vision Language Action': 'وژن زبان کارروائی',
  'Multimodal AI': 'ملٹی ماڈل AI',
  'Real-world Applications': ' حقیقی دنیا کی درخواستیں'
};

/**
 * Translates text using the translation map
 */
function translateText(text) {
  let translated = text;
  
  // Sort keys by length (descending) to replace longer phrases first
  const sortedKeys = Object.keys(TRANSLATION_MAP).sort((a, b) => b.length - a.length);
  
  for (const key of sortedKeys) {
    const regex = new RegExp(key, 'g');
    translated = translated.replace(regex, TRANSLATION_MAP[key]);
  }
  
  return translated;
}

/**
 * Translates an entire markdown file
 */
function translateFile(filePath) {
  const content = fs.readFileSync(filePath, 'utf8');
  
  // Parse frontmatter
  const frontmatterRegex = /^---\n([\s\S]*?)\n---\n/;
  const match = content.match(frontmatterRegex);
  
  let frontmatter = '';
  let body = content;
  
  if (match) {
    frontmatter = match[0];
    body = content.substring(match[0].length);
  }
  
  // Translate frontmatter
  let translatedFrontmatter = frontmatter;
  if (frontmatter) {
    // Translate title in frontmatter
    translatedFrontmatter = translatedFrontmatter.replace(
      /(title: )(.*)/g,
      (match, prefix, title) => prefix + translateText(title)
    );
    
    // Translate description in frontmatter
    translatedFrontmatter = translatedFrontmatter.replace(
      /(description: )(.*)/g,
      (match, prefix, description) => prefix + translateText(description)
    );
    
    // Translate keywords in frontmatter
    translatedFrontmatter = translatedFrontmatter.replace(
      /(keywords: \[)([^\]]+)(\])/g,
      (match, prefix, keywords, suffix) => {
        const translatedKeywords = keywords.split(',').map(k => k.trim()).map(k => translateText(k.replace(/"/g, ''))).join(', ');
        return prefix + translatedKeywords + suffix;
      }
    );
  }
  
  // Translate the main body
  let translatedBody = body;
  
  // Translate headings (#, ##, ###)
  translatedBody = translatedBody.replace(/^(#+\s+)(.*)$/gm, (match, prefix, heading) => {
    return prefix + translateText(heading);
  });
  
  // Translate bold text
  translatedBody = translatedBody.replace(/\*\*(.*?)\*\*/g, (match, text) => {
    return '**' + translateText(text) + '**';
  });
  
  // Translate italic text
  translatedBody = translatedBody.replace(/\*(.*?)\*/g, (match, text) => {
    return '*' + translateText(text) + '*';
  });
  
  // Translate blockquotes
  translatedBody = translatedBody.replace(/^(\s*> )(.*)$/gm, (match, prefix, text) => {
    return prefix + translateText(text);
  });
  
  // Translate list items
  translatedBody = translatedBody.replace(/^(\s*[-*] )(.*)$/gm, (match, prefix, text) => {
    return prefix + translateText(text);
  });
  
  // Translate numbered lists
  translatedBody = translatedBody.replace(/^(\s*\d+\.\s+)(.*)$/gm, (match, prefix, text) => {
    return prefix + translateText(text);
  });
  
  // Translate table headers (simplified)
  translatedBody = translatedBody.replace(/^\|(.+)\|$/gm, (match) => {
    return match.replace(/\|(.*?)\|/g, (m, cell) => {
      if (cell.trim() && !['--', ':', '-', ':', '-:', ':-', '---'].includes(cell.trim())) {
        return '|' + translateText(cell) + '|';
      }
      return m;
    });
  });
  
  // Translate regular paragraphs
  // This is a simplified approach - in practice, more complex parsing may be needed
  translatedBody = translatedBody.replace(/^([^-#\s>].*)$/gm, (match, line) => {
    // Only translate lines that are not code blocks or other special elements
    if (!line.trim().startsWith('```') && !line.trim().startsWith(':::')) {
      return translateText(line);
    }
    return match;
  });
  
  // Translate table content (simplified)
  translatedBody = translatedBody.replace(/\|([^|\n\r]+?)\|/g, (match, cell) => {
    // Don't translate separators
    if (!cell.includes(':')) {
      return '|' + translateText(cell) + '|';
    }
    return match;
  });
  
  return translatedFrontmatter + translatedBody;
}

/**
 * Recursively process all markdown files in source directory
 */
function processDirectory(srcDir, destDir) {
  if (!fs.existsSync(destDir)) {
    fs.mkdirSync(destDir, { recursive: true });
  }
  
  const entries = fs.readdirSync(srcDir, { withFileTypes: true });
  
  for (const entry of entries) {
    const srcPath = path.join(srcDir, entry.name);
    const destPath = path.join(destDir, entry.name);
    
    if (entry.isDirectory()) {
      processDirectory(srcPath, destPath);
    } else if (entry.isFile() && path.extname(entry.name) === '.md') {
      console.log(`Translating: ${srcPath}`);
      try {
        const translatedContent = translateFile(srcPath);
        fs.writeFileSync(destPath, translatedContent, 'utf8');
        console.log(`Saved: ${destPath}`);
      } catch (error) {
        console.error(`Error translating ${srcPath}:`, error);
      }
    }
  }
}

// Run the translation process
console.log('Starting translation process...');
processDirectory(SOURCE_DIR, URDU_DIR);
console.log('Translation process completed!');