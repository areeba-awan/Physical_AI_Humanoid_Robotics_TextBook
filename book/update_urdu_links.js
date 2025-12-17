const fs = require('fs');
const path = require('path');

// Define the Urdu directory path
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
  updatedContent = updatedContent.replace(/\[([^\]]+)\]\(\.\/(module-\d+-[a-z0-9-]+)\/([a-z0-9-]+)\)/g, (match, linkText, module, page) => {
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
 * Recursively process all markdown files in Urdu directory
 */
function processDirectory(dir) {
  const entries = fs.readdirSync(dir, { withFileTypes: true });
  
  for (const entry of entries) {
    const filePath = path.join(dir, entry.name);
    
    if (entry.isDirectory()) {
      processDirectory(filePath);
    } else if (entry.isFile() && path.extname(entry.name) === '.md') {
      console.log(`Updating links: ${filePath}`);
      try {
        const content = fs.readFileSync(filePath, 'utf8');
        const updatedContent = updateLinks(content);
        fs.writeFileSync(filePath, updatedContent, 'utf8');
        console.log(`Updated links in: ${filePath}`);
      } catch (error) {
        console.error(`Error updating links in ${filePath}:`, error);
      }
    }
  }
}

// Run the link update process for Urdu files only
console.log('Starting Urdu link update process...');
processDirectory(URDU_DIR);
console.log('Urdu link update process completed!');