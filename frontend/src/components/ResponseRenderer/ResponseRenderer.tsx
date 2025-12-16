import React from 'react';
import { Box, Typography, Chip } from '@mui/material';
import { ContentReference } from '../../services/api-client';

interface ResponseRendererProps {
  response: string;
  references: ContentReference[];
  onReferenceClick?: (reference: ContentReference) => void;
}

const ResponseRenderer: React.FC<ResponseRendererProps> = ({ response, references, onReferenceClick }) => {
  const handleReferenceClick = (reference: ContentReference, index: number) => {
    if (onReferenceClick) {
      onReferenceClick(reference);
    } else {
      // Default action if no custom handler is provided
      console.log(`Navigating to reference ${index + 1}`, reference);
      // In a real implementation, this could trigger navigation to the referenced section
    }
  };

  return (
    <Box sx={{ p: 2 }}>
      <Typography variant="body1" paragraph>
        {response}
      </Typography>

      {references && references.length > 0 && (
        <Box sx={{ mt: 2 }}>
          <Typography variant="subtitle2" gutterBottom>
            Referenced Content:
          </Typography>
          <Box sx={{ display: 'flex', flexWrap: 'wrap', gap: 1 }}>
            {references.map((reference, index) => (
              <Chip
                key={`${reference.content_id}-${index}`}
                label={
                  reference.section_title ||
                  (reference.text_snippet ? reference.text_snippet.substring(0, 30) + '...' : `Reference ${index + 1}`)
                }
                variant="outlined"
                size="small"
                clickable={true}
                onClick={() => handleReferenceClick(reference, index)}
                sx={{ cursor: 'pointer' }}
                title={`Navigate to referenced content: ${reference.text_snippet || 'No preview available'}`}
              />
            ))}
          </Box>
        </Box>
      )}
    </Box>
  );
};

export default ResponseRenderer;