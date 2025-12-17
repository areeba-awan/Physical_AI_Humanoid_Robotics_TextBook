const fs = require('fs');
const path = require('path');

// Define file paths
const SOURCE_DIR = './docs';
const URDU_DIR = './i18n/ur/docusaurus-plugin-content-docs/current';

// Enhanced mapping of English terms to Urdu translations
const TRANSLATION_MAP = {
  // General terms
  'Introduction': 'تعارف',
  'Quickstart Guide': 'فوراً شروع کریں گائیڈ',
  'Chapter': 'باب',
  'Learning Objectives': 'سیکھنے کے مقاصد',
  'Prerequisites': 'شرائطِ لازمہ',
  'What is': 'کیا ہے',
  'Understanding': 'کو سمجھنا',
  'Setting Up': 'ماحول تیار کرنا',
  'Your First': 'آپ کا پہلا',
  'Command Line': 'کمانڈ لائن',
  'Essential commands you\'ll use daily': 'ضروری کمانڈز جو آپ روزانہ استعمال کریں گے',
  'Hands-on Lab': 'ہاتھ سے کام کرنے والی لیب',
  'Knowledge Check': 'نالej چیک',
  'Summary': 'خلاصہ',
  'Further Reading': 'مزید پڑھائی',
  'Next Steps': 'اگلے اقدامات',
  'Architecture and Philosophy': 'کے معماری اور فلسفے',
  'Overview': 'جائزہ',
  'Essential': 'ضروری',
  
  // ROS 2 specific terms
  'ROS 2': 'ROS 2',
  'Robot Operating System 2': 'روبوٹ آپریٹنگ سسٹم 2',
  'Introduction to ROS 2': 'ROS 2 کا تعارف',
  'Understanding the Robot Operating System 2 architecture and philosophy': 'روبوٹ آپریٹنگ سسٹم 2 کے معماری اور فلسفے کو سمجھنا',
  'Nodes, Topics, and Services': 'نوڈز، ٹاپکس، اور سروسز',
  'Actions and Parameters': 'ایکشنز اور پیرامیٹر',
  'Launch Files and Parameters': 'لانچ فائلز اور پیرامیٹر',
  'Creating Custom Packages': 'اپنی مرضی کے پیکجز تیار کرنا',
  'Lab Exercise': 'لیب مشق',
  
  // Simulation terms
  'Simulation': 'سیمیولیشن',
  'Simulations': 'سیمیولیشنز',
  'The Digital Twin': 'ڈیجیٹل ٹوئن',
  'Digital Twin': 'ڈیجیٹل ٹوئن',
  'Introduction to Simulation': 'سیمیولیشن کا تعارف',
  'Why simulation is essential for robotics development': 'روبوٹکس کی ترقی کے لیے سیمیولیشن کیوں ضروری ہے',
  'Simulation Platforms': 'سیمیولیشن پلیٹ فارم',
  'Setting Up Gazebo': 'گیزیبو تیار کرنا',
  'Gazebo': 'گیزیبو',
  'Unity': 'یونٹی',
  'AI Assistant': 'AI اسسٹنٹ',
  'Unity Robotics Hub': 'یونٹی روبوٹکس ہب',
  'NVIDIA Isaac': 'این ویڈیا ایزیک',
  'Isaac Sim': 'ایزیک سیم',
  'Omniverse': 'اومنی ورس',
  'ROS Bridge': 'ROS برج',
  'Perception Pipelines': 'ادراک کے پائپ لائنز',
  'Navigation and Manipulation': 'نیویگیشن اور ہیرا پھیری',
  'Vision Language Action': 'وژن زبان کارروائی',
  'Vision-Language-Action': 'وژن-زبان-کارروائی',
  'VLA Systems': 'VLA نظام',
  'VLA': 'VLA',
  'Multimodal AI': 'ملٹی ماڈل AI',
  'Real-world Applications': ' حقیقی دنیا کی درخواستیں',
  
  // Technical terms
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
  'Perception': 'ادراک',
  'Manipulation': 'ہیرا پھیری',
  'Navigation': 'نیویگیشن',
  'Synthetic Data': 'مصنوعی ڈیٹا',
  'Photorealistic': 'فوٹو ریئلیسٹک',
  'Ray Tracing': 'رے ٹریسنگ',
  'Domain Randomization': 'ڈومین رینڈمائزیشن',
  
  // Educational terms
  'Module': 'مودیول',
  'The Robotic Nervous System': 'روبوٹکس کا عصبی نظام',
  'Build and test robots in realistic simulated environments': ' حقیقی نظر آنے والے ماحول میں روبوٹس تیار کریں اور ان کی جانچ کریں',
  'Leverage GPU-accelerated perception and manipulation': ' GPU کی مدد سے تیز ادراک اور ہیرا پھیری کا فائدہ اٹھائیں',
  'Create robots that see, understand, and act intelligently': 'ایسے روبوٹس بنائیں جو دیکھ سکیں، سمجھ سکیں، اور ذہین انداز میں کام کر سکیں',
  'Master the communication framework that powers modern robots': 'جدید روبوٹس کو طاقت دینے والے مواصلاتی ڈھانچے کو مسلط کریں',
  
  // Platform comparisons
  'Real-time': 'ریل ٹائم',
  'Security': 'سیکیورٹی',
  'Platforms': 'پلیٹ فارم',
  'Middleware': 'مڈل ویئر',
  'Multi-robot': 'متعدد روبوٹ',
  'Physics Accuracy': 'طبیعات کی درستی',
  'Fidelity': 'وفاداری',
  
  // Common verbs and actions
  'Get Started': 'شروع کریں',
  'Book Overview': 'کتاب کا جائزہ',
  'Chapters': 'ابواب',
  'Community': 'کمیونٹی',
  'More': 'مزید',
  'GitHub': 'گیتھب',
  'Stack Overflow': 'سٹیک اوورفلو',
  'Discord': 'ڈسکارڈ',
  'X (Twitter)': 'ایکس (ٹویٹر)',
  'Personalize': 'ذاتی نوعیت کے مطابق',
  'Translate': 'ترجمہ',
  'Run': 'چلائیں',
  'Launch': 'لانچ کریں',
  'Install': 'انسٹال کریں',
  'Create': 'تخلیق کریں',
  'Build': 'تعمیر کریں',
  'Source': 'ماخذ',
  'Test': 'جانچ',
  'Learn': 'سیکھیں',
  'Understand': 'سمجھیں',
  'Compare': ' موازنہ کریں',
  'Set up': ' تیار کریں',
  'Add': 'شامل کریں',
  'Apply': 'اپلائی کریں',
  'Observe': ' مشاہدہ کریں',
  'See': 'دیکھیں',
  'Use': 'استعمال کریں',
  'Enable': 'فعال کریں',
  'Choose': 'منتخب کریں',
  'Develop': 'ترقی دیں',
  'Design': ' ڈیزائن',
  'Validate': 'توثیق کریں',
  
  // Common nouns
  'Environment': 'ماحول',
  'Framework': 'ڈھانچہ',
  'System': 'سسٹم',
  'Software': ' سافٹ ویئر',
  'Tools': 'اوزار',
  'Libraries': ' لائبریری',
  'Visualization': ' وژولائزیشن',
  'Debugging': ' ڈیبگنگ',
  'Simulation': 'سیمیولیشن',
  'Hardware': 'ہارڈ ویئر',
  'Sensors': 'سینسرز',
  'Actuators': ' ایکچو ایٹرز',
  'Processes': ' عمل',
  'Messages': 'پیغامات',
  'Topics': 'ٹاپکس',
  'Services': ' سروسز',
  'Actions': 'ایکشنز',
  'Nodes': 'نوڈز',
  'Packages': 'پیکجز',
  'Workspace': ' ورک سپیس',
  'Commands': ' کمانڈز',
  'Code': 'کوڈ',
  'Program': 'پروگرام',
  'Publisher': 'شائع کنندہ',
  'Subscriber': ' مسیحین',
  'Master': ' ماسٹر',
  'Master Node': ' ماسٹر نوڈ',
  'Client Library': 'کلائنٹ لائبریری',
  'Middleware': 'مڈل ویئر',
  'Quality of Service': 'سروس کی معیار',
  'Reliability': 'قابل اعتمادی',
  'Durability': ' دوام',
  'Shape': 'شکل',
  'Forces': ' قوتیں',
  'Physics': 'طبیعات',
  'World': 'دنیا',
  'Model': ' ماڈل',
  'World File': 'دنیا فائل',
  'Launch File': ' لانچ فائل',
  'Parameters': ' پیرامیٹر',
  'Custom Packages': 'اپنی مرضی کے پیکجز',
  'Development': ' ترقی',
  'Validation': ' توثیق',
  'Testing': ' جانچ',
  'Scenarios': ' منظرنامے',
  'Concept': ' تصور',
  'Platform': ' پلیٹ فارم',
  'Integration': ' انضمام',
  'Training': ' تربیت',
  'Synthetic Data': ' مصنوعی ڈیٹا',
  'Rendering': ' رینڈرنگ',
  'Ray Tracing': ' رے ٹریسنگ',
  'Domain Randomization': ' ڈومین رینڈمائزیشن',
  
  // Specific to the text
  'to سیمیولیشن': 'کا تعارف',
  'to گیزیبو': 'کا تعارف',
  'to یونٹی': 'کا تعارف',
  'to سیمیولیشنز': 'کا تعارف',
  'to ایزیک سیم': 'کا تعارف',
  'to VLA': 'کا تعارف',
  'to ایزیک': 'کا تعارف',
  'to ROS 2': 'کا تعارف',
  'to Nodes': 'کا تعارف',
  'to Actions': 'کا تعارف',
  'to Launch': 'کا تعارف',
  'to Custom': 'کا تعارف',
  'to Lab': 'کا تعارف'
};

/**
 * Translates text using the translation map
 */
function translateText(text) {
  let translated = text;
  
  // Sort keys by length (descending) to replace longer phrases first
  const sortedKeys = Object.keys(TRANSLATION_MAP).sort((a, b) => b.length - a.length);
  
  for (const key of sortedKeys) {
    // Use word boundaries to avoid partial matches within longer words
    const regex = new RegExp('\\b' + escapeRegExp(key) + '\\b', 'g');
    translated = translated.replace(regex, TRANSLATION_MAP[key]);
  }
  
  return translated;
}

/**
 * Escapes special regex characters in a string
 */
function escapeRegExp(string) {
  return string.replace(/[.*+?^${}()|[\]\\]/g, '\\$&');
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
      /(title: )(".*?"|'.*?'|[^|\n\r]+)/g,
      (match, prefix, title) => {
        // Extract the actual title text (removing quotes)
        const cleanTitle = title.replace(/^['"]|['"]$/g, '');
        const translatedTitle = translateText(cleanTitle);
        return prefix + `"${translatedTitle}"`;
      }
    );
    
    // Translate description in frontmatter
    translatedFrontmatter = translatedFrontmatter.replace(
      /(description: )(".*?"|'.*?'|[^|\n\r]+)/g,
      (match, prefix, description) => {
        const cleanDescription = description.replace(/^['"]|['"]$/g, '');
        const translatedDescription = translateText(cleanDescription);
        return prefix + `"${translatedDescription}"`;
      }
    );
    
    // Translate keywords in frontmatter
    translatedFrontmatter = translatedFrontmatter.replace(
      /(keywords: \[)([^\]]+)(\])/g,
      (match, prefix, keywords, suffix) => {
        const translatedKeywords = keywords.split(',').map(k => k.trim()).map(k => translateText(k.replace(/"/g, ''))).join('", "');
        return prefix + '"' + translatedKeywords + '"' + suffix;
      }
    );
  }
  
  // Translate the main body
  let translatedBody = body;
  
  // Translate headings (#, ##, ###)
  translatedBody = translatedBody.replace(/^(#+\s+)(".*?"|'.*?'|[^|\n\r]+)$/gm, (match, prefix, heading) => {
    const cleanHeading = heading.replace(/^['"]|['"]$/g, '');
    return prefix + translateText(cleanHeading);
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
    return match.replace(/\|([^|\n\r]+?)\|/g, (m, cell) => {
      if (cell.trim() && !['--', ':', '-', ':', '-:', ':-', '---', '-------', '----------', '-----------'].includes(cell.trim())) {
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
    if (!cell.includes(':') && cell.trim().length > 0 && 
        !['--', ':', '-', ':', '-:', ':-', '---', '-------', '----------', '-----------'].includes(cell.trim())) {
      return '|' + translateText(cell) + '|';
    }
    return match;
  });
  
  // Update internal links to include /ur/ prefix
  translatedBody = translatedBody.replace(/\[([^\]]+)\]\((\/docs\/[^\)]+)\)/g, (match, linkText, url) => {
    const updatedUrl = url.replace('/docs/', '/ur/docs/');
    return `[${linkText}](${updatedUrl})`;
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
console.log('Starting comprehensive translation process...');
processDirectory(SOURCE_DIR, URDU_DIR);
console.log('Comprehensive translation process completed!');