import React, { useState, useRef, useEffect } from 'react';
import { Box, TextField, Button, Typography, Paper, Stack } from '@mui/material';
import SendIcon from '@mui/icons-material/Send';
import ResponseRenderer from '../ResponseRenderer/ResponseRenderer';
import { ContentReference } from '../../services/api-client';

interface ChatMessage {
  id: string;
  text: string;
  sender: 'user' | 'bot';
  timestamp: Date;
  references?: ContentReference[];
}

interface ChatInterfaceProps {
  bookId: string;
  sessionId?: string;
  onNewSessionId?: (sessionId: string) => void;
  onNavigateToReference?: (reference: ContentReference) => void;
  context?: any; // Context information about the current reading position
}

const ChatInterface: React.FC<ChatInterfaceProps> = ({ bookId, sessionId, onNewSessionId, onNavigateToReference, context }) => {
  const [messages, setMessages] = useState<ChatMessage[]>([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const messagesEndRef = useRef<HTMLDivElement>(null);

  // Scroll to bottom of messages
  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  const handleSend = async () => {
    if (!inputValue.trim() || isLoading) return;

    // Add user message to the chat
    const userMessage: ChatMessage = {
      id: Date.now().toString(),
      text: inputValue,
      sender: 'user',
      timestamp: new Date(),
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      // Make API call to backend to get response
      const requestPayload: any = {
        query: inputValue,
      };

      // Add context if available
      if (context) {
        requestPayload.context = context;
      }

      const response = await fetch('/v1/book/' + bookId + '/query', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(requestPayload),
      });

      if (!response.ok) {
        throw new Error(`API request failed with status ${response.status}`);
      }

      const data = await response.json();

      // Add bot response to the chat
      const botMessage: ChatMessage = {
        id: (Date.now() + 1).toString(),
        text: data.response,
        sender: 'bot',
        timestamp: new Date(),
        references: data.references,  // Include references if they exist
      };

      setMessages(prev => [...prev, botMessage]);
      
      // Set session ID if provided in response and callback exists
      if (data.conversation_id && onNewSessionId) {
        onNewSessionId(data.conversation_id);
      }
    } catch (error) {
      console.error('Error sending message:', error);
      
      // Add error message to the chat
      const errorMessage: ChatMessage = {
        id: (Date.now() + 1).toString(),
        text: 'Sorry, there was an error processing your request. Please try again.',
        sender: 'bot',
        timestamp: new Date(),
      };
      
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyDown = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSend();
    }
  };

  return (
    <Paper 
      elevation={3} 
      sx={{ 
        height: '100%', 
        display: 'flex', 
        flexDirection: 'column',
        overflow: 'hidden'
      }}
    >
      <Box sx={{ p: 2, borderBottom: 1, borderColor: 'divider' }}>
        <Typography variant="h6" component="h2">
          Book Assistant
        </Typography>
      </Box>
      
      <Box 
        sx={{ 
          flex: 1, 
          overflowY: 'auto',
          p: 2,
          display: 'flex',
          flexDirection: 'column',
          gap: 2
        }}
      >
        {messages.length === 0 ? (
          <Box 
            sx={{ 
              flex: 1, 
              display: 'flex', 
              alignItems: 'center', 
              justifyContent: 'center' 
            }}
          >
            <Typography variant="body1" color="text.secondary">
              Ask me anything about this book!
            </Typography>
          </Box>
        ) : (
          <Stack spacing={2}>
            {messages.map((message) => (
              <Box
                key={message.id}
                sx={{
                  display: 'flex',
                  justifyContent: message.sender === 'user' ? 'flex-end' : 'flex-start',
                }}
              >
                <Box
                  sx={{
                    maxWidth: '80%',
                    p: 1.5,
                    borderRadius: 2,
                    bgcolor: message.sender === 'user' ? 'primary.main' : 'grey.200',
                    color: message.sender === 'user' ? 'primary.contrastText' : 'text.primary',
                  }}
                >
                  {message.sender === 'bot' && message.references && message.references.length > 0 ? (
                    <ResponseRenderer
                      response={message.text}
                      references={message.references}
                      onReferenceClick={(reference) => {
                        if (onNavigateToReference) {
                          onNavigateToReference(reference);
                        }
                      }}
                    />
                  ) : (
                    <>
                      <Typography variant="body1">{message.text}</Typography>
                      <Typography
                        variant="caption"
                        sx={{
                          display: 'block',
                          color: message.sender === 'user' ? 'primary.light' : 'text.secondary',
                          textAlign: 'right',
                          mt: 0.5
                        }}
                      >
                        {message.timestamp.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
                      </Typography>
                    </>
                  )}
                </Box>
              </Box>
            ))}
            {isLoading && (
              <Box
                sx={{
                  display: 'flex',
                  justifyContent: 'flex-start',
                }}
              >
                <Box
                  sx={{
                    maxWidth: '80%',
                    p: 1.5,
                    borderRadius: 2,
                    bgcolor: 'grey.200',
                    color: 'text.primary',
                  }}
                >
                  <Typography variant="body1">Thinking...</Typography>
                </Box>
              </Box>
            )}
            <div ref={messagesEndRef} />
          </Stack>
        )}
      </Box>
      
      <Box sx={{ p: 2, borderTop: 1, borderColor: 'divider' }}>
        <Stack direction="row" spacing={1}>
          <TextField
            fullWidth
            multiline
            maxRows={4}
            variant="outlined"
            placeholder="Ask a question about the book..."
            value={inputValue}
            onChange={(e) => setInputValue(e.target.value)}
            onKeyDown={handleKeyDown}
            disabled={isLoading}
          />
          <Button
            variant="contained"
            color="primary"
            onClick={handleSend}
            disabled={!inputValue.trim() || isLoading}
            sx={{ height: 'fit-content' }}
          >
            <SendIcon />
          </Button>
        </Stack>
      </Box>
    </Paper>
  );
};

export default ChatInterface;