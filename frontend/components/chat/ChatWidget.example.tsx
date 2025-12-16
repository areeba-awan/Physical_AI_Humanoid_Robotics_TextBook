/**
 * Example Chat Widget Component with RAG Integration
 *
 * Features:
 * - Global book queries with RAG
 * - Selected text queries (strict mode)
 * - Streaming responses
 * - Source citations
 * - Session-based memory
 */

'use client';

import React, { useState, useRef, useEffect, useCallback } from 'react';
import { ChatAPIClient, Citation, StreamChunk } from '@/lib/chat-api';

interface Message {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  sources?: Citation[];
  isStreaming?: boolean;
}

interface ChatWidgetProps {
  token: string;
  contextChapter?: string;
  selectedText?: string;
  onClearSelectedText?: () => void;
}

export function ChatWidget({
  token,
  contextChapter,
  selectedText,
  onClearSelectedText,
}: ChatWidgetProps) {
  const [messages, setMessages] = useState<Message[]>([]);
  const [input, setInput] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [isOpen, setIsOpen] = useState(false);
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const clientRef = useRef<ChatAPIClient | null>(null);

  // Initialize client
  useEffect(() => {
    clientRef.current = new ChatAPIClient(token);
  }, [token]);

  // Auto-scroll to bottom
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  // Handle streaming response
  const handleStreamingResponse = useCallback(async (
    messageId: string,
    streamGenerator: AsyncGenerator<StreamChunk>
  ) => {
    let fullContent = '';
    let sources: Citation[] = [];

    for await (const chunk of streamGenerator) {
      if (chunk.type === 'content' && chunk.content) {
        fullContent += chunk.content;
        setMessages(prev =>
          prev.map(msg =>
            msg.id === messageId
              ? { ...msg, content: fullContent }
              : msg
          )
        );
      } else if (chunk.type === 'done') {
        sources = chunk.sources || [];
        setMessages(prev =>
          prev.map(msg =>
            msg.id === messageId
              ? { ...msg, sources, isStreaming: false }
              : msg
          )
        );
      }
    }
  }, []);

  // Send message handler
  const handleSendMessage = useCallback(async () => {
    if (!input.trim() || !clientRef.current || isLoading) return;

    const userMessage: Message = {
      id: `user-${Date.now()}`,
      role: 'user',
      content: input,
    };

    const assistantMessageId = `assistant-${Date.now()}`;
    const assistantMessage: Message = {
      id: assistantMessageId,
      role: 'assistant',
      content: '',
      isStreaming: true,
    };

    setMessages(prev => [...prev, userMessage, assistantMessage]);
    setInput('');
    setIsLoading(true);

    try {
      // Determine which mode to use
      if (selectedText) {
        // STRICT Selected Text Mode
        const stream = clientRef.current.streamMessageWithSelectedText(
          input,
          selectedText,
          contextChapter
        );
        await handleStreamingResponse(assistantMessageId, stream);
        onClearSelectedText?.();
      } else {
        // Global Book Query Mode
        const stream = clientRef.current.streamMessage(input, contextChapter);
        await handleStreamingResponse(assistantMessageId, stream);
      }
    } catch (error) {
      console.error('Chat error:', error);
      setMessages(prev =>
        prev.map(msg =>
          msg.id === assistantMessageId
            ? {
                ...msg,
                content: 'Sorry, there was an error processing your request.',
                isStreaming: false,
              }
            : msg
        )
      );
    } finally {
      setIsLoading(false);
    }
  }, [input, selectedText, contextChapter, isLoading, handleStreamingResponse, onClearSelectedText]);

  // Clear chat handler
  const handleClearChat = useCallback(async () => {
    if (!clientRef.current) return;
    try {
      await clientRef.current.clearHistory();
      setMessages([]);
    } catch (error) {
      console.error('Failed to clear chat:', error);
    }
  }, []);

  if (!isOpen) {
    return (
      <button
        onClick={() => setIsOpen(true)}
        className="fixed bottom-6 right-6 bg-blue-600 text-white p-4 rounded-full shadow-lg hover:bg-blue-700 transition-colors"
      >
        <svg className="w-6 h-6" fill="none" viewBox="0 0 24 24" stroke="currentColor">
          <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M8 10h.01M12 10h.01M16 10h.01M9 16H5a2 2 0 01-2-2V6a2 2 0 012-2h14a2 2 0 012 2v8a2 2 0 01-2 2h-5l-5 5v-5z" />
        </svg>
      </button>
    );
  }

  return (
    <div className="fixed bottom-6 right-6 w-96 h-[600px] bg-white rounded-lg shadow-2xl flex flex-col border border-gray-200">
      {/* Header */}
      <div className="bg-blue-600 text-white p-4 rounded-t-lg flex justify-between items-center">
        <div>
          <h3 className="font-semibold">Book Assistant</h3>
          <p className="text-xs text-blue-100">
            {selectedText ? 'Selected Text Mode' : 'Ask about the book'}
          </p>
        </div>
        <div className="flex gap-2">
          <button
            onClick={handleClearChat}
            className="text-blue-100 hover:text-white"
            title="Clear chat"
          >
            <svg className="w-5 h-5" fill="none" viewBox="0 0 24 24" stroke="currentColor">
              <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M19 7l-.867 12.142A2 2 0 0116.138 21H7.862a2 2 0 01-1.995-1.858L5 7m5 4v6m4-6v6m1-10V4a1 1 0 00-1-1h-4a1 1 0 00-1 1v3M4 7h16" />
            </svg>
          </button>
          <button
            onClick={() => setIsOpen(false)}
            className="text-blue-100 hover:text-white"
          >
            <svg className="w-5 h-5" fill="none" viewBox="0 0 24 24" stroke="currentColor">
              <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M6 18L18 6M6 6l12 12" />
            </svg>
          </button>
        </div>
      </div>

      {/* Selected text indicator */}
      {selectedText && (
        <div className="bg-yellow-50 p-2 border-b border-yellow-200 flex items-center justify-between">
          <div className="flex items-center gap-2">
            <span className="text-yellow-600 text-xs font-medium">
              Answering from selection only
            </span>
          </div>
          <button
            onClick={onClearSelectedText}
            className="text-yellow-600 hover:text-yellow-800 text-xs"
          >
            Clear
          </button>
        </div>
      )}

      {/* Messages */}
      <div className="flex-1 overflow-y-auto p-4 space-y-4">
        {messages.length === 0 && (
          <div className="text-center text-gray-500 mt-8">
            <p className="text-sm">Ask me anything about the textbook!</p>
            <p className="text-xs mt-2 text-gray-400">
              Select text in the book for specific explanations
            </p>
          </div>
        )}

        {messages.map((message) => (
          <div
            key={message.id}
            className={`flex ${message.role === 'user' ? 'justify-end' : 'justify-start'}`}
          >
            <div
              className={`max-w-[80%] rounded-lg p-3 ${
                message.role === 'user'
                  ? 'bg-blue-600 text-white'
                  : 'bg-gray-100 text-gray-800'
              }`}
            >
              <p className="text-sm whitespace-pre-wrap">{message.content}</p>

              {/* Streaming indicator */}
              {message.isStreaming && (
                <span className="inline-block w-2 h-4 bg-gray-400 animate-pulse ml-1" />
              )}

              {/* Sources */}
              {message.sources && message.sources.length > 0 && (
                <div className="mt-2 pt-2 border-t border-gray-200">
                  <p className="text-xs text-gray-500 mb-1">Sources:</p>
                  <div className="flex flex-wrap gap-1">
                    {message.sources.map((source, idx) => (
                      <span
                        key={idx}
                        className="text-xs bg-blue-100 text-blue-700 px-2 py-0.5 rounded"
                        title={source.snippet}
                      >
                        {source.chapter_id}: {source.section}
                      </span>
                    ))}
                  </div>
                </div>
              )}
            </div>
          </div>
        ))}
        <div ref={messagesEndRef} />
      </div>

      {/* Input */}
      <div className="p-4 border-t border-gray-200">
        <div className="flex gap-2">
          <input
            type="text"
            value={input}
            onChange={(e) => setInput(e.target.value)}
            onKeyPress={(e) => e.key === 'Enter' && handleSendMessage()}
            placeholder={selectedText ? 'Ask about the selected text...' : 'Ask a question...'}
            className="flex-1 border border-gray-300 rounded-lg px-3 py-2 text-sm focus:outline-none focus:ring-2 focus:ring-blue-500"
            disabled={isLoading}
          />
          <button
            onClick={handleSendMessage}
            disabled={isLoading || !input.trim()}
            className="bg-blue-600 text-white px-4 py-2 rounded-lg hover:bg-blue-700 disabled:opacity-50 disabled:cursor-not-allowed transition-colors"
          >
            {isLoading ? (
              <svg className="w-5 h-5 animate-spin" fill="none" viewBox="0 0 24 24">
                <circle className="opacity-25" cx="12" cy="12" r="10" stroke="currentColor" strokeWidth="4" />
                <path className="opacity-75" fill="currentColor" d="M4 12a8 8 0 018-8V0C5.373 0 0 5.373 0 12h4z" />
              </svg>
            ) : (
              <svg className="w-5 h-5" fill="none" viewBox="0 0 24 24" stroke="currentColor">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M12 19l9 2-9-18-9 18 9-2zm0 0v-8" />
              </svg>
            )}
          </button>
        </div>
      </div>
    </div>
  );
}

export default ChatWidget;
