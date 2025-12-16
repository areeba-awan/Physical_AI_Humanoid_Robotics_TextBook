import React, { useState } from 'react';
import { Box, TextField, Button, FormControlLabel, Switch, Typography } from '@mui/material';
import SendIcon from '@mui/icons-material/Send';

interface QueryInputProps {
  onSubmit: (query: string, selectedTextOnly: boolean) => void;
  placeholder?: string;
  selectedText?: string;
}

const QueryInput: React.FC<QueryInputProps> = ({ 
  onSubmit, 
  placeholder = "Ask a question about the book...",
  selectedText = "" 
}) => {
  const [inputValue, setInputValue] = useState('');
  const [selectedTextOnlyMode, setSelectedTextOnlyMode] = useState(false);

  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    if (inputValue.trim()) {
      onSubmit(inputValue, selectedTextOnlyMode);
      setInputValue('');
    }
  };

  const handleKeyDown = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSubmit(e as any);
    }
  };

  // Automatically enable selected text mode if there's selected text
  React.useEffect(() => {
    if (selectedText && selectedText.trim() !== "") {
      setSelectedTextOnlyMode(true);
    }
  }, [selectedText]);

  return (
    <Box component="form" onSubmit={handleSubmit} sx={{ width: '100%' }}>
      {selectedText && selectedText.trim() !== "" && (
        <Box sx={{ mb: 1, p: 1, border: 1, borderColor: 'primary.main', borderRadius: 1, bgcolor: 'primary.light' }}>
          <Typography variant="caption" sx={{ fontWeight: 'bold' }}>
            Selected text for query:
          </Typography>
          <Typography variant="body2" sx={{ mt: 0.5, fontStyle: 'italic' }}>
            {selectedText.length > 100 ? selectedText.substring(0, 100) + '...' : selectedText}
          </Typography>
        </Box>
      )}
      
      <Box sx={{ display: 'flex', alignItems: 'flex-end', gap: 1 }}>
        <TextField
          fullWidth
          multiline
          maxRows={4}
          value={inputValue}
          onChange={(e) => setInputValue(e.target.value)}
          onKeyDown={handleKeyDown}
          placeholder={placeholder}
          variant="outlined"
        />
        <Button
          type="submit"
          variant="contained"
          color="primary"
          endIcon={<SendIcon />}
          disabled={!inputValue.trim()}
          sx={{ height: 'fit-content' }}
        >
          Send
        </Button>
      </Box>
      
      <Box sx={{ mt: 1, display: 'flex', alignItems: 'center' }}>
        <FormControlLabel
          control={
            <Switch
              checked={selectedTextOnlyMode}
              onChange={(e) => setSelectedTextOnlyMode(e.target.checked)}
              disabled={!selectedText || selectedText.trim() === ""}
            />
          }
          label="Use selected text only"
        />
      </Box>
    </Box>
  );
};

export default QueryInput;