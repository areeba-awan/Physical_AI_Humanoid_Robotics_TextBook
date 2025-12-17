const fs = require('fs');
const path = require('path');

// Define file paths
const SOURCE_DIR = './docs';
const URDU_DIR = './i18n/ur/docusaurus-plugin-content-docs/current';

/**
 * Updates internal links to include /ur/ prefix
 */
function updateLinks(content) {
  let updatedContent = content;
  
  // For absolute links starting with /docs/
  updatedContent = updatedContent.replace(/\[([^\]]+)\]\((\/docs\/[^\)]+)\)/g, (match, linkText, url) => {
    const updatedUrl = url.replace('/docs/', '/ur/docs/');
    return `[${linkText}](${updatedUrl})`;
  });
  
  // For relative links to module chapters like ./module-1-ros2/chapter-1-intro
  updatedContent = updatedContent.replace(/\[([^\]]+)\]\(\.\/(module-\d+-[a-z-]+)\/([a-z0-9-]+)\)/g, (match, linkText, module, page) => {
    return `[${linkText}](/ur/docs/${module}/${page})`;
  });
  
  // For relative links that go up a directory and then to modules like ../module-2-simulation/chapter-1-intro
  updatedContent = updatedContent.replace(/\[([^\]]+)\]\(\.\.\/(module-\d+-[a-z-]+)\/([a-z0-9-]+)\)/g, (match, linkText, module, page) => {
    return `[${linkText}](/ur/docs/${module}/${page})`;
  });
  
  // For relative links to quickstart, intro, etc.
  updatedContent = updatedContent.replace(/\[([^\]]+)\]\(\.\/(quickstart|intro|about|overview|index)\)/g, (match, linkText, page) => {
    return `[${linkText}](/ur/docs/${page})`;
  });
  
  // For relative links that go up and to quickstart, intro, etc.
  updatedContent = updatedContent.replace(/\[([^\]]+)\]\(\.\.\/(quickstart|intro|about|overview|index)\)/g, (match, linkText, page) => {
    return `[${linkText}](/ur/docs/${page})`;
  });
  
  return updatedContent;
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
      console.log(`Updating links: ${srcPath}`);
      try {
        const content = fs.readFileSync(srcPath, 'utf8');
        const updatedContent = updateLinks(content);
        fs.writeFileSync(destPath, updatedContent, 'utf8');
        console.log(`Updated links in: ${destPath}`);
      } catch (error) {
        console.error(`Error updating links in ${srcPath}:`, error);
      }
    }
  }
}

// Run the link update process
console.log('Starting link update process...');
processDirectory(SOURCE_DIR, URDU_DIR);
console.log('Link update process completed!');