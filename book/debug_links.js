const fs = require('fs');

// Read the intro file
const content = fs.readFileSync('./i18n/ur/docusaurus-plugin-content-docs/current/intro.md', 'utf8');

// Check for relative links
const relativeLinkPattern = /\[([^\]]+)\]\(\.\/(module-\d+-[a-z-]+)\/([a-z0-9-]+)\)/g;
const matches = content.match(relativeLinkPattern);

console.log('Found relative links:');
if (matches) {
  matches.forEach(match => {
    console.log(match);
  });
} else {
  console.log('No relative links found with the pattern');
}

// Check for all links that start with ./
const allRelativePattern = /\[([^\]]+)\]\(\.[^)]*\)/g;
const allRelativeMatches = content.match(allRelativePattern);

console.log('\nAll relative links (starting with ./):');
if (allRelativeMatches) {
  allRelativeMatches.forEach(match => {
    console.log(match);
  });
} else {
  console.log('No relative links found');
}