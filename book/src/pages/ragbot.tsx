import React, { useState, useRef, useEffect } from 'react';
import Layout from '@theme/Layout';
import styles from './ragbot.module.css';

interface Message {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  sources?: Array<{
    chapterId: string;
    section: string;
    relevance: number;
  }>;
  timestamp: Date;
}

// API URL - change this to your backend URL in production
const API_URL = 'http://localhost:8000';

export default function RAGBot(): JSX.Element {
  const [messages, setMessages] = useState<Message[]>([]);
  const [input, setInput] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const messagesEndRef = useRef<HTMLDivElement>(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const sendMessage = async () => {
    if (!input.trim() || isLoading) return;

    const userMessage: Message = {
      id: Date.now().toString(),
      role: 'user',
      content: input,
      timestamp: new Date(),
    };

    setMessages(prev => [...prev, userMessage]);
    setInput('');
    setIsLoading(true);

    try {
      const response = await fetch(`${API_URL}/api/v1/chat`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          message: input,
        }),
      });

      const data = await response.json();

      const assistantMessage: Message = {
        id: (Date.now() + 1).toString(),
        role: 'assistant',
        content: data.content || data.answer || 'Sorry, I could not process your request.',
        sources: data.sources,
        timestamp: new Date(),
      };

      setMessages(prev => [...prev, assistantMessage]);
    } catch (error) {
      console.error('Chat error:', error);
      const errorMessage: Message = {
        id: (Date.now() + 1).toString(),
        role: 'assistant',
        content: 'Sorry, there was an error connecting to the server. Please try again later.',
        timestamp: new Date(),
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  const clearHistory = () => {
    setMessages([]);
  };

  const suggestedQuestions = [
    'What is ROS 2 and why should I use it?',
    'How do I create a URDF robot model?',
    'Explain the VLA architecture',
    'What is NVIDIA Isaac used for?',
    'How do I set up Gazebo simulation?',
    'What are ROS 2 actions vs services?',
  ];

  return (
    <Layout title="AI Assistant" description="Chat with our AI assistant about Physical AI & Robotics">
      <div className={styles.container}>
        <div className={styles.sidebar}>
          <div className={styles.sidebarHeader}>
            <h2>🤖 AI Assistant</h2>
            <p>Ask questions about Physical AI & Robotics</p>
          </div>

          <div className={styles.suggestions}>
            <h3>Suggested Questions</h3>
            {suggestedQuestions.map((q, i) => (
              <button
                key={i}
                onClick={() => setInput(q)}
                className={styles.suggestionBtn}
              >
                {q}
              </button>
            ))}
          </div>

          <div className={styles.actions}>
            <button onClick={clearHistory} className={styles.clearBtn}>
              🗑️ Clear History
            </button>
          </div>
        </div>

        <div className={styles.chatArea}>
          <div className={styles.messages}>
            {messages.length === 0 && (
              <div className={styles.emptyState}>
                <div className={styles.emptyIcon}>🤖</div>
                <h3>Welcome to the AI Assistant!</h3>
                <p>
                  I'm here to help you learn about Physical AI & Humanoid Robotics.
                  Ask me anything about ROS 2, Gazebo, NVIDIA Isaac, or VLA systems.
                </p>
              </div>
            )}

            {messages.map((msg) => (
              <div
                key={msg.id}
                className={`${styles.message} ${styles[msg.role]}`}
              >
                <div className={styles.messageAvatar}>
                  {msg.role === 'user' ? '👤' : '🤖'}
                </div>
                <div className={styles.messageBody}>
                  <div className={styles.messageContent}>
                    {msg.content}
                  </div>
                  {msg.sources && msg.sources.length > 0 && (
                    <div className={styles.sources}>
                      <span className={styles.sourcesLabel}>📚 Sources:</span>
                      <div className={styles.sourcesList}>
                        {msg.sources.map((source, idx) => (
                          <a
                            key={idx}
                            href={`/docs/${source.chapterId}`}
                            className={styles.sourceLink}
                          >
                            {source.section}
                          </a>
                        ))}
                      </div>
                    </div>
                  )}
                  <div className={styles.timestamp}>
                    {msg.timestamp.toLocaleTimeString()}
                  </div>
                </div>
              </div>
            ))}

            {isLoading && (
              <div className={`${styles.message} ${styles.assistant}`}>
                <div className={styles.messageAvatar}>🤖</div>
                <div className={styles.messageBody}>
                  <div className={styles.typing}>
                    <span></span>
                    <span></span>
                    <span></span>
                  </div>
                </div>
              </div>
            )}

            <div ref={messagesEndRef} />
          </div>

          <div className={styles.inputArea}>
            <textarea
              value={input}
              onChange={(e) => setInput(e.target.value)}
              onKeyPress={handleKeyPress}
              placeholder="Ask a question about Physical AI & Robotics..."
              className={styles.input}
              disabled={isLoading}
              rows={2}
            />
            <button
              onClick={sendMessage}
              className={styles.sendButton}
              disabled={isLoading || !input.trim()}
            >
              {isLoading ? (
                <span className={styles.spinner}></span>
              ) : (
                'Send →'
              )}
            </button>
          </div>
        </div>
      </div>
    </Layout>
  );
}
